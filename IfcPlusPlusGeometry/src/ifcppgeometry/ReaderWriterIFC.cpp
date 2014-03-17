/* -*-c++-*- IfcPlusPlus - www.ifcplusplus.com  - Copyright (C) 2011 Fabian Gerold
 *
 * This library is open source and may be redistributed and/or modified under  
 * the terms of the OpenSceneGraph Public License (OSGPL) version 0.0 or 
 * (at your option) any later version.  The full license is in LICENSE file
 * included with this distribution, and on the openscenegraph.org website.
 * 
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the 
 * OpenSceneGraph Public License for more details.
*/

#include <utility>
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#ifdef IFCPP_OPENMP
#include <omp.h>
#endif

#include <osgDB/Registry>
#include <osgDB/ReadFile>
#include <osgDB/ReaderWriter>
#include <osgDB/FileNameUtils>
#include <osg/CullFace>

#include <ifcpp/IFC4/include/IfcProduct.h>
#include <ifcpp/IFC4/include/IfcProject.h>
#include <ifcpp/IFC4/include/IfcElement.h>
#include <ifcpp/IFC4/include/IfcRelVoidsElement.h>
#include <ifcpp/IFC4/include/IfcRelAggregates.h>
#include <ifcpp/IFC4/include/IfcRelContainedInSpatialStructure.h>
#include <ifcpp/IFC4/include/IfcFeatureElementSubtraction.h>
#include <ifcpp/IFC4/include/IfcRepresentation.h>
#include <ifcpp/IFC4/include/IfcProductRepresentation.h>
#include <ifcpp/IFC4/include/IfcPropertySet.h>
#include <ifcpp/IFC4/include/IfcPropertySetDefinition.h>
#include <ifcpp/IFC4/include/IfcPropertySetDefinitionSet.h>
#include <ifcpp/IFC4/include/IfcRelDefinesByProperties.h>
#include <ifcpp/IFC4/include/IfcSpace.h>
#include <ifcpp/IFC4/include/IfcBeam.h>
#include <ifcpp/IFC4/include/IfcCurtainWall.h>
#include <ifcpp/IFC4/include/IfcWindow.h>
#include <ifcpp/IFC4/include/IfcBuildingStorey.h>
#include <ifcpp/IFC4/include/IfcSite.h>
#include <ifcpp/IFC4/include/IfcLengthMeasure.h>
#include <ifcpp/IFC4/include/IfcLabel.h>

#include <ifcpp/model/IfcPPModel.h>
#include <ifcpp/model/IfcPPException.h>
#include <ifcpp/reader/IfcStepReader.h>
#include <ifcpp/reader/IfcPlusPlusReader.h>
#include <ifcpp/writer/IfcStepWriter.h>
#include <ifcpp/model/UnitConverter.h>

#include "GeometrySettings.h"
#include "GeomUtils.h"
#include "UnhandledRepresentationException.h"
#include "RepresentationConverter.h"
#include "PlacementConverter.h"
#include "SolidModelConverter.h"
#include "ConverterOSG.h"
#include "ReaderWriterIFC.h"

ReaderWriterIFC::ReaderWriterIFC()
{
	osgDB::ReaderWriter::supportsExtension("ifc","Industry Foundation Classes");
	osgDB::ReaderWriter::supportsExtension("stp","Step");
	m_ifc_model		= shared_ptr<IfcPPModel>(new IfcPPModel());
	m_step_reader	= shared_ptr<IfcStepReader>(new IfcStepReader());
	m_step_writer	= shared_ptr<IfcStepWriter>(new IfcStepWriter());
	
	m_step_reader->setProgressCallBack( this, &ReaderWriterIFC::slotProgressValueWrapper );
	m_step_reader->setMessageCallBack( this, &ReaderWriterIFC::slotMessageWrapper );
	m_step_reader->setErrorCallBack( this, &ReaderWriterIFC::slotErrorWrapper );

	m_geom_settings = shared_ptr<GeometrySettings>( new GeometrySettings() );
	resetNumVerticesPerCircle();
	m_unit_converter = shared_ptr<UnitConverter>( new UnitConverter() );
	m_representation_converter = shared_ptr<RepresentationConverter>( new RepresentationConverter( m_geom_settings, m_unit_converter ) );

	m_glass_stateset = new osg::StateSet();
	m_glass_stateset->setMode( GL_BLEND, osg::StateAttribute::ON );
	m_glass_stateset->setRenderingHint( osg::StateSet::TRANSPARENT_BIN );

	m_group_result = new osg::Group();
}

ReaderWriterIFC::~ReaderWriterIFC()
{
}

void ReaderWriterIFC::resetModel()
{
	progressTextCallback( "Unloading model, cleaning up memory..." );
	m_err.str(std::string());
	m_messages.str(std::string());

	deleteInputCache();
	m_processed_products.clear();

	m_group_result->removeChildren( 0, m_group_result->getNumChildren() );
	m_recent_progress = 0.0;

	progressTextCallback( "Unloading model done" );
	m_ignored_types.clear();
	m_selected_types.clear();	
}

void ReaderWriterIFC::deleteInputCache()
{
	m_shape_input_data.clear();
}

void ReaderWriterIFC::resetNumVerticesPerCircle()
{
	m_geom_settings->m_num_vertices_per_circle = 20;
}

void ReaderWriterIFC::setModel( shared_ptr<IfcPPModel> model )
{
	if( m_ifc_model )
	{
		m_ifc_model->clearIfcModel();
	}
	m_ifc_model = model;

	m_unit_converter = m_ifc_model->getUnitConverter();
	m_representation_converter = shared_ptr<RepresentationConverter>( new RepresentationConverter( m_geom_settings, m_unit_converter ) );
}

void ReaderWriterIFC::setNumVerticesPerCircle( int num_vertices )
{
	//if( num_vertices < 6 ) { num_vertices = 6; }
	m_geom_settings->m_num_vertices_per_circle = num_vertices;
}

osgDB::ReaderWriter::ReadResult ReaderWriterIFC::readNode(const std::string& filename, const osgDB::ReaderWriter::Options* options)
{
	resetModel();

	// Split the supplied options (putting them in lower case) to know which types to ignore and which types to select when reading the IFC file
	std::string options_string = options->getOptionString();
	for (std::string::iterator c = options_string.begin(); c != options_string.end(); ++c) 
	{
		*c = tolower(*c);
	}

	// dispatch the options to  either ignored or selected types
	std::istringstream buf(options_string);
	std::istream_iterator<std::string> beg(buf), end;
	std::vector<std::string> tokens(beg, end);
	for(int i = 0; i < tokens.size(); i++)
	{        
		size_t sepPosition = tokens[i].find (':');
		std::string whatToDo = tokens[i].substr(0, sepPosition);
		std::string withWhat = tokens[i].substr(sepPosition + 1);
		if (whatToDo == "ignore")
		{
			m_ignored_types.push_back(withWhat);
		}
		else
		if (whatToDo == "select")
		{
			m_selected_types.push_back(withWhat);
		}
		else
		{
			assert("unkown option");
		}
	}

#if _DEBUG
	std::cout << "Filtering out types: " << std::endl;
	for (std::vector<std::string>::iterator it = m_ignored_types.begin(); it != m_ignored_types.end(); ++it)
	{
		std::cout << "   " << *it << std::endl;
	}
	std::cout << "Selecting types: " << std::endl;
	for (std::vector<std::string>::iterator it = m_selected_types.begin(); it != m_selected_types.end(); ++it)
	{
		std::cout << "   " << *it << std::endl;
	}
#endif

	std::string ext = osgDB::getFileExtension(filename);
	if( !acceptsExtension(ext) ) return osgDB::ReaderWriter::ReadResult::FILE_NOT_HANDLED;
	
	// open file
	std::ifstream infile;
	infile.open ( filename.c_str(), std::ifstream::in);
	
	if( !infile.is_open() )
	{
		return osgDB::ReaderWriter::ReadResult::FILE_NOT_FOUND;
	}

	// get length of file:
	infile.seekg (0, std::ios::end);
	const int length = infile.tellg();
	infile.seekg (0, std::ios::beg);

	// allocate memory:
	std::string buffer( length, '\0' );

	// read data as a block:
	infile.read (&buffer[0],length);
	infile.close();

	m_ifc_model->clearIfcModel();
	m_step_reader->setModel( m_ifc_model );
	m_step_reader->readStreamHeader( buffer );
	std::string file_schema_version = m_ifc_model->getFileSchema();
	std::map<int,shared_ptr<IfcPPEntity> > map_entities;
	
	m_err.clear();
	m_err.str( std::string() );

	progressTextCallback( "Reading STEP data..." );

	try
	{
		m_step_reader->readStreamData( buffer, map_entities );
	}
	catch( IfcPPException& e )
	{
		m_err << e.what();
	}
	catch( std::exception& e )
	{
		m_err << e.what();
	}
	catch( ... )
	{
		m_err << "An error occured in ReaderWriterIFC::readNode";
	}

	m_group_result->removeChildren( 0, m_group_result->getNumChildren() );

	try
	{
		// insert entities into model
		std::map<int,shared_ptr<IfcPPEntity> >::iterator it_model;
		for( it_model=map_entities.begin(); it_model != map_entities.end(); ++it_model )
		{
			shared_ptr<IfcPPEntity>& ent = it_model->second;
				
			try
			{
				m_ifc_model->insertEntity( ent );
			}
			catch( IfcPPException& e )
			{
				m_err << e.what();
			}

		}

		m_ifc_model->resolveInverseAttributes();
		m_ifc_model->updateCache();

		m_unit_converter = m_ifc_model->getUnitConverter();
		m_representation_converter = shared_ptr<RepresentationConverter>( new RepresentationConverter( m_geom_settings, m_unit_converter ) );
	}
	catch( IfcPPException& e )
	{
		m_err << e.what();
	}
	catch( std::exception& e )
	{
		m_err << e.what();
	}
	catch( ... )
	{
		m_err << "An error occured in ReaderWriterIFC::readNode";
	}

	progressTextCallback( "Creating geometry..." );
	try
	{
		createGeometry();
	}
	catch( IfcPPException& e )
	{
		m_err << e.what();
	}
	catch( std::exception& e )
	{
		m_err << e.what();
	}
	catch( ... )
	{
		m_err << "An error occured in ReaderWriterIFC::readNode";
	}

	osgDB::ReaderWriter::ReadResult::ReadStatus status = osgDB::ReaderWriter::ReadResult::FILE_LOADED;
	if( m_err.tellp() > 0 )
	{
		status = osgDB::ReaderWriter::ReadResult::ERROR_IN_READING_FILE;
		std::cout << m_err.str().c_str();
	}

	return osgDB::ReaderWriter::ReadResult( m_group_result, status );
}


//#define SHOW_ENTITIES_OUTSIDE_SPATIAL_STRUCTURE
void ReaderWriterIFC::createGeometry()
{
	shared_ptr<IfcProject> project = m_ifc_model->getIfcProject();
	if( !project )
	{
		throw IfcPPException( "ReaderWriterIFC: no valid IfcProject in model." , __func__ );
	}

	m_shape_input_data.clear();

	std::vector<shared_ptr<IfcProduct> > vec_products;
	std::stringstream err;

	double length_to_meter_factor = m_ifc_model->getUnitConverter()->getLengthInMeterFactor();
	carve::setEpsilon( 1.4901161193847656e-08*length_to_meter_factor );

	const std::map<int,shared_ptr<IfcPPEntity> >& map = m_ifc_model->getMapIfcObjects();
	std::map<int,shared_ptr<IfcPPEntity> >::const_iterator it;
	for( it=map.begin(); it!=map.end(); ++it )
	{
		shared_ptr<IfcPPEntity> obj = it->second;
		shared_ptr<IfcProduct> product = dynamic_pointer_cast<IfcProduct>( obj );
		if( product )
		{
			vec_products.push_back( product );
		}
	}
	
	// create geometry for for each IfcProduct independently, spatial structure will be resolved later

#ifdef IFCPP_OPENMP
	{
		std::map<int,shared_ptr<ShapeInputData> > *map_products_ptr = &m_shape_input_data;
		const int num_products = vec_products.size();
		
		omp_lock_t writelock_map;
		omp_init_lock(&writelock_map);
		
		#pragma omp parallel firstprivate(num_products) shared(map_products_ptr)
		{
			// time for one product may vary significantly, so schedule not so many
			#pragma omp for schedule(dynamic,10)
			for( int i=0; i<num_products; ++i )
			{
				shared_ptr<IfcProduct> product = vec_products[i];
				std::stringstream thread_err;
				if( dynamic_pointer_cast<IfcFeatureElementSubtraction>(product) )
				{
					// geometry will be created in method subtractOpenings
					continue;
				}
				if( !product->m_Representation )
				{
					continue;
				}

				const int product_id = product->getId();
				shared_ptr<ShapeInputData> product_shape( new ShapeInputData() );
				product_shape->ifc_product = product;

				try
				{
					// (Compare types in lowercase to avoid typos)
					std::string lowercaseType = product->classname();
					for (std::string::iterator c = lowercaseType.begin(); c != lowercaseType.end(); ++c)
					{
						*c = tolower(*c);
					}

					// Filter out element types found in m_ignored_types, select elements found in m_selected_types
					if ((std::find(m_ignored_types.begin(),  m_ignored_types.end(),  lowercaseType) == m_ignored_types.end()) &&
						((m_selected_types.size() == 0) || (std::find(m_selected_types.begin(), m_selected_types.end(), lowercaseType) != m_selected_types.end())))
					{
						// Build the shape representing the product, if required
						convertIfcProduct( product, product_shape );
					}
					else
					{
						vec_products.pop_back();
					}

				}
				catch( IfcPPException& e)
				{
					thread_err << e.what();
				}		
				catch( carve::exception& e)
				{
					thread_err << e.str();
				}
				catch( std::exception& e )
				{
					thread_err << e.what();
				}
				catch( ... )
				{
					thread_err << "error in " << __FUNC__ << ", product id " << product_id << std::endl;
				}

				//#pragma omp critical
				omp_set_lock(&writelock_map);
				{
					map_products_ptr->insert( std::make_pair( product_id, product_shape ) );
					m_processed_products[product_id] = product;
			
					//for( int opening_i = 0; opening_i<product_shape->vec_openings.size(); ++opening_i )
					//{
					//	shared_ptr<IfcProduct>& opening_product = product_shape->vec_openings[opening_i];
					//	m_processed_products[opening_product->getId()] = opening_product;
					//}
					if( thread_err.tellp() > 0 )
					{
						err << thread_err.str().c_str();
					}
				}
				omp_unset_lock(&writelock_map);

				if( omp_get_thread_num() == 0 )
				{
					// progress callback
					double progress = (double)i/(double)num_products;
					if( progress - m_recent_progress > 0.02 )
					{
						// leave 10% of progress to openscenegraph internals
						progressCallback( progress*0.9 );
						m_recent_progress = progress;
					}
				}
			}
		}
#pragma omp barrier
		omp_destroy_lock(&writelock_map);
	}
#else
	const int num_products = vec_products.size();
	for( int i=0; i<num_products; ++i )
	{
		shared_ptr<IfcProduct> product = vec_products[i];
		
		if( dynamic_pointer_cast<IfcFeatureElementSubtraction>(product) )
		{
			// geometry will be created in method subtractOpenings
			continue;
		}
		if( !product->m_Representation )
		{
			continue;
		}

		const int product_id = product->getId();
		shared_ptr<ShapeInputData> product_shape( new ShapeInputData() );
		product_shape->ifc_product = product;

		try
		{
			// (Compare types in lowercase to avoid typos)
			std::string lowercaseType = product->classname();
			for (std::string::iterator c = lowercaseType.begin(); c != lowercaseType.end(); ++c)
			{
				*c = tolower(*c);
			}

			// Filter out element types found in m_ignored_types, select elements found in m_selected_types
			if ((std::find(m_ignored_types.begin(),  m_ignored_types.end(),  lowercaseType) == m_ignored_types.end()) &&
				((m_selected_types.size() == 0) || (std::find(m_selected_types.begin(), m_selected_types.end(), lowercaseType) != m_selected_types.end())))
			{
				// Build the shape representing the product, if required
				convertIfcProduct( product, product_shape );
			}
			else
			{
				vec_products.pop_back();
			}

		}
		catch( IfcPPException& e)
		{
			err << e.what();
		}		
		catch( carve::exception& e)
		{
			err << e.str();
		}
#ifdef _DEBUG
		catch( DebugBreakException& dbge )
		{
			throw dbge;
		}
#endif
		catch( std::exception& e )
		{
			err << e.what();
		}
		catch( ... )
		{
			err << "error in " << __FUNC__ << ", product id " << product_id << std::endl;
		}

		m_shape_input_data[product_id] = product_shape;
		m_processed_products[product_id] = product;
		// progress callback
		double progress = (double)i/(double)num_products;
		if( progress - m_recent_progress > 0.03 )
		{
			// leave 10% of progress to openscenegraph internals
			progressCallback( progress*0.9 );
			m_recent_progress = progress;
		}
	}
#endif

	
	try
	{
		// now resolve spatial structure
		shared_ptr<IfcProject> ifc_project = m_ifc_model->getIfcProject();
		if( ifc_project )
		{
			m_map_visited.clear();
			resolveProjectStructure( ifc_project, m_group_result );
		}

		// check if there are items outside of spatial structure
		osg::ref_ptr<osg::Group> group_outside_spatial_structure = new osg::Group();
		std::string group_name = group_outside_spatial_structure->getName();
		group_outside_spatial_structure->setName( group_name.append( ", IfcProduct outside spatial structure" ) );
		m_group_result->addChild( group_outside_spatial_structure );

		for( std::map<int,shared_ptr<ShapeInputData> >::iterator it_product_shapes = m_shape_input_data.begin(); it_product_shapes!=m_shape_input_data.end(); ++it_product_shapes )
		{
			shared_ptr<ShapeInputData> product_shape = it_product_shapes->second;
			shared_ptr<IfcProduct> ifc_product = product_shape->ifc_product;
			if( !product_shape->added_to_storey )
			{
				shared_ptr<IfcFeatureElementSubtraction> opening = dynamic_pointer_cast<IfcFeatureElementSubtraction>(ifc_product);
				if( opening )
				{
					continue;
				}

				osg::ref_ptr<osg::Switch> product_switch = product_shape->product_switch;
				if( product_switch.valid() )
				{
					group_outside_spatial_structure->addChild( product_switch );
				}
				product_shape->added_to_storey = true;
			}
		}
	}
	catch( IfcPPException& e)
	{
		err << e.what();
	}
	catch(std::exception& e)
	{
		err << e.what();
	}
	catch( ... )
	{
		err << "error in " << __FUNC__ << std::endl;
	}

	progressCallback( 1.0 );
	progressTextCallback( "Loading file done" );

	if( err.tellp() > 0 )
	{
		throw IfcPPException( err.str().c_str(), __func__ );
	}
}

void ReaderWriterIFC::resolveProjectStructure( const shared_ptr<IfcPPObject>& obj, osg::Group* parent_group )
{
	shared_ptr<IfcObjectDefinition> obj_def = dynamic_pointer_cast<IfcObjectDefinition>(obj);
	if( !obj_def )
	{
		return;
	}

	int entity_id = obj_def->getId();
	if( m_map_visited.find( entity_id ) != m_map_visited.end() )
	{
		return;
	}
	m_map_visited[entity_id] = obj_def;

	osg::Group* item_grp = NULL;

	shared_ptr<IfcBuildingStorey> building_storey = dynamic_pointer_cast<IfcBuildingStorey>(obj_def);
	if( building_storey )
	{
		int building_storey_id = building_storey->getId();
		double elevation = 0.0;
		if( building_storey->m_Elevation )
		{
			//elevation = building_storey->m_Elevation->m_value*m_unit_converter->getLengthInMeterFactor();
		}
		osg::Switch* switch_building_storey = new osg::Switch();
		std::stringstream storey_switch_name;
		storey_switch_name << "#" << building_storey_id << " IfcBuildingStorey switch";
		switch_building_storey->setName( storey_switch_name.str().c_str() );

		osg::ref_ptr<osg::MatrixTransform> transform_building_storey = new osg::MatrixTransform( osg::Matrix::translate( 0, 0, elevation ) );
		switch_building_storey->addChild( transform_building_storey );
				
		std::stringstream storey_name;
		storey_name << "#" << building_storey_id << " IfcBuildingStorey transform";
		transform_building_storey->setName( storey_name.str().c_str() );

		item_grp = transform_building_storey;
		parent_group->addChild( switch_building_storey );
	}
		
	shared_ptr<IfcProduct> ifc_product = dynamic_pointer_cast<IfcProduct>(obj_def);
	if( ifc_product )
	{
		std::map<int,shared_ptr<ShapeInputData> >::iterator it_product_map = m_shape_input_data.find(entity_id);
		if( it_product_map != m_shape_input_data.end() )
		{
			shared_ptr<ShapeInputData>& product_shape = it_product_map->second;
			product_shape->added_to_storey = true;
			osg::ref_ptr<osg::Switch> product_switch = product_shape->product_switch;
			if( product_switch.valid() )
			{
				//item_grp->addChild( product_switch );
				if( item_grp == NULL )
				{
					item_grp = product_switch;
				}
				else
				{
					item_grp->addChild( product_switch );
				}
				parent_group->addChild( item_grp );
			}
		}
	}

	if( item_grp == NULL )
	{
		item_grp = new osg::Switch();
		parent_group->addChild( item_grp );
	}
	
	
	if( item_grp->getName().size() < 1 )
	{
		std::stringstream switch_name;
		switch_name << "#" << entity_id << "=" << obj_def->classname();
		item_grp->setName( switch_name.str().c_str() );
	}
	
	if( obj_def->m_IsDecomposedBy_inverse.size() > 0 )
	{
		std::vector<weak_ptr<IfcRelAggregates> >& vec_IsDecomposedBy = obj_def->m_IsDecomposedBy_inverse;
		for( std::vector<weak_ptr<IfcRelAggregates> >::iterator it=vec_IsDecomposedBy.begin(); it!=vec_IsDecomposedBy.end(); ++it )
		{
			shared_ptr<IfcRelAggregates> rel_agg( *it );
			std::vector<shared_ptr<IfcObjectDefinition> >& vec = rel_agg->m_RelatedObjects;
	
			for( std::vector<shared_ptr<IfcObjectDefinition> >::iterator it_object_def=vec.begin(); it_object_def!=vec.end(); ++it_object_def )
			{
				shared_ptr<IfcObjectDefinition> child_obj_def = (*it_object_def);
				resolveProjectStructure( child_obj_def, item_grp );
			}
		}
	}

	shared_ptr<IfcSpatialStructureElement> spatial_ele = dynamic_pointer_cast<IfcSpatialStructureElement>(obj_def);
	if( spatial_ele )
	{
		std::vector<weak_ptr<IfcRelContainedInSpatialStructure> >& vec_contained = spatial_ele->m_ContainsElements_inverse;
		if( vec_contained.size() > 0 )
		{
			std::vector<weak_ptr<IfcRelContainedInSpatialStructure> >::iterator it_rel_contained;
			for( it_rel_contained=vec_contained.begin(); it_rel_contained!=vec_contained.end(); ++it_rel_contained )
			{

				shared_ptr<IfcRelContainedInSpatialStructure> rel_contained( *it_rel_contained );
				std::vector<shared_ptr<IfcProduct> >& vec_related_elements = rel_contained->m_RelatedElements;
				std::vector<shared_ptr<IfcProduct> >::iterator it;
			
				for( it=vec_related_elements.begin(); it!=vec_related_elements.end(); ++it )
				{
					shared_ptr<IfcProduct> related_product = (*it);
					resolveProjectStructure( related_product, item_grp );
				}
			}
		}
	}
}


// @brief creates geometry objects from an IfcProduct object
// caution: when using OpenMP, this method runs in parallel threads, so every write access to member variables needs a write lock
void ReaderWriterIFC::convertIfcProduct( const shared_ptr<IfcProduct>& product, shared_ptr<ShapeInputData>& product_shape )
{
	if( !product->m_Representation )
	{
		// IfcProduct needs to have a representation
		return;
	}
	
	const int product_id = product->getId();
	std::stringstream strs_err;
	osg::ref_ptr<osg::Switch> product_switch = new osg::Switch();
	std::stringstream group_name;
	group_name << "#" << product_id << " IfcProduct group";
	product_switch->setName( group_name.str().c_str() );

	// IfcProduct has an ObjectPlacement that can be local or global
	double length_factor = m_unit_converter->getLengthInMeterFactor();
	carve::math::Matrix product_placement_matrix( carve::math::Matrix::IDENT() );

	if( product->m_ObjectPlacement )
	{
		// IfcPlacement2Matrix follows related placements in case of local coordinate systems
		std::set<int> placement_already_applied;
		PlacementConverter::convertIfcObjectPlacement( product->m_ObjectPlacement, product_placement_matrix, length_factor, placement_already_applied );
	}

	// evaluate IFC geometry
	shared_ptr<IfcProductRepresentation> product_representation = product->m_Representation;
	std::vector<shared_ptr<IfcRepresentation> >& vec_representations = product_representation->m_Representations;
	std::vector<shared_ptr<IfcRepresentation> >::iterator it_representations;
	for( it_representations=vec_representations.begin(); it_representations!=vec_representations.end(); ++it_representations )
	{
		shared_ptr<IfcRepresentation> representation = (*it_representations);
		std::set<int> visited_representation;
		m_representation_converter->convertIfcRepresentation( representation, product_placement_matrix, product_shape, visited_representation, strs_err );
	}

	std::vector<shared_ptr<ShapeInputData> > vec_opening_data;
	const shared_ptr<IfcElement> ifc_element = dynamic_pointer_cast<IfcElement>(product);
	if( ifc_element )
	{
		m_representation_converter->convertOpenings( ifc_element, vec_opening_data, strs_err );
	}


	std::vector<shared_ptr<ItemData> >& product_items = product_shape->vec_item_data;
	for( int i_item=0; i_item<product_items.size(); ++i_item )
	{
		shared_ptr<ItemData> item_data = product_items[i_item];
		// create shape for closed shells
		item_data->createMeshSetsFromClosedPolyhedrons();
		osg::Group* item_group = new osg::Group();

		// create shape for open shells
		for( int i=0; i<item_data->open_polyhedrons.size(); ++i )
		{
			shared_ptr<carve::input::PolyhedronData>& shell_data = item_data->open_polyhedrons[i];
			if( shell_data->getVertexCount() < 3 )
			{
				continue;
			}
			
			shared_ptr<carve::mesh::MeshSet<3> > open_shell_meshset( shell_data->createMesh(carve::input::opts()) );
			osg::ref_ptr<osg::Geode> geode = new osg::Geode();
			ConverterOSG::drawMeshSet( open_shell_meshset.get(), geode );
			item_group->addChild(geode);

			// disable back face culling for open meshes
			osg::ref_ptr<osg::CullFace> cull_back_off = new osg::CullFace( osg::CullFace::BACK );
			geode->getOrCreateStateSet()->setAttributeAndModes( cull_back_off.get(), osg::StateAttribute::OFF );
		}

		// create shape for open or closed shells
		for( int i_poly=0; i_poly<item_data->open_or_closed_polyhedrons.size(); ++i_poly )
		{
			shared_ptr<carve::input::PolyhedronData>& shell_data = item_data->open_or_closed_polyhedrons[i_poly];
			if( shell_data->getVertexCount() < 3 )
			{
				continue;
			}

			shared_ptr<carve::mesh::MeshSet<3> > open_or_closed_shell_meshset( shell_data->createMesh(carve::input::opts()) );
			osg::ref_ptr<osg::Geode> geode = new osg::Geode();
			ConverterOSG::drawMeshSet( open_or_closed_shell_meshset.get(), geode );
			item_group->addChild(geode);
		}

		// cut out openings like windows etc.
		if( ifc_element )
		{
			m_representation_converter->subtractOpenings( ifc_element, item_data, vec_opening_data, strs_err );
		}

		// create shape for meshsets
		for( std::vector<shared_ptr<carve::mesh::MeshSet<3> > >::iterator it_meshsets = item_data->meshsets.begin(); it_meshsets != item_data->meshsets.end(); ++it_meshsets )
		{
			shared_ptr<carve::mesh::MeshSet<3> >& item_meshset = (*it_meshsets);

			if( m_geom_settings->m_use_mesh_simplifier_before_draw )
			{
				try
				{
					m_representation_converter->getSolidConverter()->simplifyMesh( item_meshset.get() );
				}
				catch( carve::exception& e )
				{
					strs_err << "simplifyMesh failed (" << e.str() << ") on product id " << product_id << std::endl;
				}
			}

			osg::ref_ptr<osg::Geode> geode_result = new osg::Geode();
			ConverterOSG::drawMeshSet( item_meshset.get(), geode_result );
			item_group->addChild(geode_result);
		}

		
		// create shape for polylines
		for( int polyline_i = 0; polyline_i < item_data->polylines.size(); ++polyline_i )
		{
			shared_ptr<carve::input::PolylineSetData>& polyline_data = item_data->polylines.at(polyline_i);
			osg::ref_ptr<osg::Geode> geode = new osg::Geode();
			ConverterOSG::drawPolyline( polyline_data.get(), geode );
			item_group->addChild(geode);
		}

		// apply statesets if there are any
		if( item_data->statesets.size() > 0 )
		{
			for( int i=0; i<item_data->statesets.size(); ++i )
			{
				osg::StateSet* next_item_stateset = item_data->statesets[i];
				if( !next_item_stateset )
				{
					continue;
				}

				osg::StateSet* existing_item_stateset = item_group->getStateSet();

				if( existing_item_stateset )
				{
					osg::StateSet* merged_product_stateset = new osg::StateSet( *existing_item_stateset );
					merged_product_stateset->merge( *next_item_stateset );
					item_group->setStateSet( merged_product_stateset );
				}
				else
				{
					item_group->setStateSet( next_item_stateset );
				}
			}
		}

		// If anything has been created, add it to the product group
		if( item_group->getNumChildren() > 0 )
		{
			product_switch->addChild(item_group);
		}
	}

	// Fetch the IFCProduct relationships
	if( product->m_IsDefinedBy_inverse.size() > 0 )
	{
		std::vector<weak_ptr<IfcRelDefinesByProperties> >& vec_IsDefinedBy_inverse = product->m_IsDefinedBy_inverse;
		for( int i=0; i<vec_IsDefinedBy_inverse.size(); ++i )
		{
			shared_ptr<IfcRelDefinesByProperties> rel_def( vec_IsDefinedBy_inverse[i] );
			shared_ptr<IfcPropertySetDefinitionSelect> relating_property_definition_select = rel_def->m_RelatingPropertyDefinition;

			// TYPE IfcPropertySetDefinitionSelect = SELECT	(IfcPropertySetDefinition	,IfcPropertySetDefinitionSet);
			shared_ptr<IfcPropertySetDefinition> property_set_def = dynamic_pointer_cast<IfcPropertySetDefinition>(relating_property_definition_select );
			if( property_set_def )
			{
				shared_ptr<IfcPropertySet> property_set = dynamic_pointer_cast<IfcPropertySet>(property_set_def);
				if( property_set )
				{
					m_representation_converter->convertIfcPropertySet( property_set, product_switch.get() );
				}
				continue;
			}

			shared_ptr<IfcPropertySetDefinitionSet> property_set_def_set = dynamic_pointer_cast<IfcPropertySetDefinitionSet>(relating_property_definition_select );
			if( property_set_def_set )
			{
				std::vector<shared_ptr<IfcPropertySetDefinition> >& vec_propterty_set_def = property_set_def_set->m_vec;
				std::vector<shared_ptr<IfcPropertySetDefinition> >::iterator it_property_set_def;
				for( it_property_set_def=vec_propterty_set_def.begin(); it_property_set_def!=vec_propterty_set_def.end(); ++it_property_set_def )
				{
					shared_ptr<IfcPropertySetDefinition> property_set_def = (*it_property_set_def);
					if( property_set_def )
					{
						shared_ptr<IfcPropertySet> property_set = dynamic_pointer_cast<IfcPropertySet>(property_set_def);
						if( property_set )
						{
							m_representation_converter->convertIfcPropertySet( property_set, product_switch.get() );
						}
					}
				}
				
				continue;
			}
		}
	}

	// Simplify the OSG statesets
	for( int i=0; i<product_shape->vec_statesets.size(); ++i )
	{
		osg::StateSet* next_product_stateset = product_shape->vec_statesets[i];
		if( !next_product_stateset )
		{
			continue;
		}

		osg::StateSet* existing_item_stateset = product_switch->getStateSet();
		if( existing_item_stateset )
		{
			osg::StateSet* merged_product_stateset = new osg::StateSet( *existing_item_stateset );
			merged_product_stateset->merge( *next_product_stateset );
			product_switch->setStateSet( merged_product_stateset );
		}
		else
		{
			product_switch->setStateSet( next_product_stateset );
		}
	}

	// enable transparency for certain objects
	if( dynamic_pointer_cast<IfcSpace>(product) )
	{
		//product_switch->setStateSet( m_glass_stateset );
	}
	else if( dynamic_pointer_cast<IfcCurtainWall>(product) || dynamic_pointer_cast<IfcWindow>(product) )
	{
		// TODO: make only glass part of window transparent
		//product_switch->setStateSet( m_glass_stateset );
	}
	else if( dynamic_pointer_cast<IfcSite>(product) )
	{
		std::stringstream group_name;
		group_name << "#" << product_id << " IfcSite";
		product_switch->setName( group_name.str().c_str() );
	}
	// TODO: if no color or material is given, set color 231/219/169 for walls, 140/140/140 for slabs 

	if( product_switch->getNumChildren() > 0 )
	{
		product_shape->product_switch = product_switch;
	}
	
	// Filter out objects which types are listed in the provided options    
    std::string product_class_name = product->classname();
    for (std::string::iterator c = product_class_name.begin(); c != product_class_name.end(); ++c) 
    {
        *c = tolower(*c);
    }
    if (std::find(m_ignored_types.begin(), m_ignored_types.end(), std::string(product_class_name)) != m_ignored_types.end())
    {
		product_switch->removeChildren(0, product_switch->getNumChildren() );
	}
	
	if( strs_err.tellp() > 0 )
	{
		throw IfcPPException( strs_err.str().c_str(), __func__ );
	}
}

void ReaderWriterIFC::slotProgressValueWrapper( void* ptr, double progress_value )
{
	ReaderWriterIFC* myself = (ReaderWriterIFC*)ptr;

	myself->progressCallback( progress_value );
}

void ReaderWriterIFC::slotProgressTextWrapper( void* ptr, const std::string& str )
{
	ReaderWriterIFC* myself = (ReaderWriterIFC*)ptr;
	myself->progressTextCallback( str );
}

void ReaderWriterIFC::slotMessageWrapper( void* ptr, const std::string& str )
{
	ReaderWriterIFC* myself = (ReaderWriterIFC*)ptr;
	if( myself )
	{
		myself->messageCallback( str );
	}
}

void ReaderWriterIFC::slotErrorWrapper( void* ptr, const std::string& str )
{
	ReaderWriterIFC* myself = (ReaderWriterIFC*)ptr;
	if( myself )
	{
		myself->errorCallback( str );
	}
}

// register with Registry to instantiate the above reader/writer.
REGISTER_OSGPLUGIN(ifc, ReaderWriterIFC)
