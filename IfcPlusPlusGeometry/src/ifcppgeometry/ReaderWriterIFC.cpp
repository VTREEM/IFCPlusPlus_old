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

#include <carve/carve.hpp>
#include <carve/csg.hpp>
#include <carve/csg_triangulator.hpp>
#include <carve/faceloop.hpp>
#include <carve/geom3d.hpp>
#include <carve/input.hpp>
#include <carve/polyhedron_base.hpp>
#include <carve/poly.hpp>
#include <carve/polyhedron_base.hpp>
#include <carve/poly.hpp>
#include <carve/triangulator.hpp>
#include <common/geometry.hpp>
#include <common/rgb.hpp>


#include <osgDB/Registry>
#include <osgDB/ReadFile>
#include <osgDB/ReaderWriter>
#include <osgDB/FileNameUtils>
#include <osg/Material>
#include <osg/Depth>
#include <osg/LightModel>

#include <utility>
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#ifdef IFCPP_OPENMP
#include <omp.h>
#endif


#include "ifcpp/IFC4/include/IfcProduct.h"
#include "ifcpp/IFC4/include/IfcProject.h"
#include "ifcpp/IFC4/include/IfcElement.h"
#include "ifcpp/IFC4/include/IfcRelVoidsElement.h"
#include "ifcpp/IFC4/include/IfcRelAggregates.h"
#include "ifcpp/IFC4/include/IfcRelContainedInSpatialStructure.h"
#include "ifcpp/IFC4/include/IfcFeatureElementSubtraction.h"
#include "ifcpp/IFC4/include/IfcRepresentation.h"
#include "ifcpp/IFC4/include/IfcProductRepresentation.h"
#include "ifcpp/IFC4/include/IfcPropertySet.h"
#include "ifcpp/IFC4/include/IfcPropertySetDefinition.h"
#include "ifcpp/IFC4/include/IfcPropertySetDefinitionSet.h"
#include "ifcpp/IFC4/include/IfcRelDefinesByProperties.h"
#include "ifcpp/IFC4/include/IfcSpace.h"
#include "ifcpp/IFC4/include/IfcBeam.h"
#include "ifcpp/IFC4/include/IfcCurtainWall.h"
#include "ifcpp/IFC4/include/IfcWindow.h"
#include "ifcpp/IFC4/include/IfcBuildingStorey.h"
#include "ifcpp/IFC4/include/IfcSite.h"
#include "ifcpp/IFC4/include/IfcLengthMeasure.h"

#include "ifcpp/model/IfcPPModel.h"
#include "ifcpp/model/IfcPPException.h"
#include "ifcpp/reader/IfcStepReader.h"
#include "ifcpp/reader/IfcPlusPlusReader.h"
#include "ifcpp/writer/IfcStepWriter.h"
#include "ifcpp/model/UnitConverter.h"

#include "RepresentationConverter.h"
#include "PlacementConverter.h"
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

	//osg::Material* material = new osg::Material();
	//float shinyness = 35.f;
	//material = new osg::Material();
	//material->setAmbient( osg::Material::FRONT_AND_BACK, osg::Vec4f( 0.02f, 0.025f, 0.03f, 0.1 ) );
	//material->setDiffuse( osg::Material::FRONT_AND_BACK, osg::Vec4f( 0.8f, 0.82f, 0.84f, 0.1f ) );
	//material->setSpecular( osg::Material::FRONT_AND_BACK, osg::Vec4f( 0.02f, 0.025f, 0.03f, 0.03f ) );
	//material->setShininess( osg::Material::FRONT_AND_BACK, shinyness );
	//material->setColorMode( osg::Material::SPECULAR );
	//material->setEmission( osg::Material::FRONT_AND_BACK, osg::Vec4f( 0.05f, 0.08f, 0.1f, 0.1f ) );
	//material->setAlpha(osg::Material::FRONT_AND_BACK, 0.35f );

	m_glass_stateset = new osg::StateSet();
	m_glass_stateset->setMode( GL_BLEND, osg::StateAttribute::ON );
	m_glass_stateset->setRenderingHint( osg::StateSet::TRANSPARENT_BIN );

	m_group_result = new osg::Group();
}

ReaderWriterIFC::~ReaderWriterIFC()
{
}

void ReaderWriterIFC::reset()
{
	m_err.str(std::string());
	m_messages.str(std::string());

	m_product_shapes.clear();
	m_processed_products.clear();

	m_group_result->removeChildren( 0, m_group_result->getNumChildren() );
	m_vec_building_storeys.clear();
	m_recent_progress = 0.0;
}

void ReaderWriterIFC::setModel( shared_ptr<IfcPPModel> model )
{
	if( m_ifc_model )
	{
		m_ifc_model->clearIfcModel();
	}
	m_ifc_model = model;

	m_unit_converter = m_ifc_model->getUnitConverter();
	m_representation_converter = shared_ptr<RepresentationConverter>( new RepresentationConverter( m_unit_converter ) );
}

osgDB::ReaderWriter::ReadResult ReaderWriterIFC::readNode(const std::string& filename, const osgDB::ReaderWriter::Options*)
{
	reset();

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
//	m_geom_tree.clear();
	
	int millisecs_read_start = clock();
	m_step_reader->readStreamHeader( buffer, m_ifc_model );
	std::string file_schema_version = m_ifc_model->getFileSchema();
	std::map<int,shared_ptr<IfcPPEntity> > map_entities;
	
	m_err.clear();
	m_err.str( std::string() );

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

	int time_diff_read = clock() - millisecs_read_start;
	std::cout << "file parse time " << time_diff_read << " ms, num entities: " << map_entities.size() << std::endl;


	m_group_result->removeChildren( 0, m_group_result->getNumChildren() );
	//osg::ref_ptr<osg::Group> group_result = new osg::Group();
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
		m_representation_converter = shared_ptr<RepresentationConverter>( new RepresentationConverter( m_unit_converter ) );
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

	try
	{
		convertIfcModel();
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

	for( int i=0; i<m_vec_building_storeys.size(); ++i )
	{
		shared_ptr<ReaderWriterIFC::BuildingStoreyGroup>& storey_group = m_vec_building_storeys[i];

		osg::ref_ptr<osg::MatrixTransform> storey_transform = storey_group->storey_transform;

		m_group_result->addChild( storey_transform );

	}

	
//	group_result->addChild( m_geom_tree.products.get() );
	//group_result->addChild( m_geom_tree.terrain.get() );

	// osg ReadResult assumes that there is either a valid read object, or an error message.
	// however, often there are warnings or errors and still a valid read result
	// this needs to be done via ReaderWriterIFC::getWarnings and ReaderWriterIFC::getErrors
	osgDB::ReaderWriter::ReadResult::ReadStatus status = osgDB::ReaderWriter::ReadResult::FILE_LOADED;
	m_err.seekp(0, std::ios::end);
	std::stringstream::pos_type err_length = m_err.tellp();
	m_err.clear();
	if( err_length > 0 )
	{
		status = osgDB::ReaderWriter::ReadResult::ERROR_IN_READING_FILE;
	}

	return osgDB::ReaderWriter::ReadResult( m_group_result, status );
}


//#define SHOW_ENTITIES_OUTSIDE_SPATIAL_STRUCTURE
void ReaderWriterIFC::convertIfcModel()
{
	shared_ptr<IfcProject> project = m_ifc_model->getIfcProject();
	if( !project )
	{
		throw IfcPPException( "ReaderWriterIFC: no valid IfcProject in model." );
	}

	m_product_shapes.clear();
	//m_geom_tree.clear();
	std::vector<shared_ptr<IfcProduct> > vec_products;
	std::stringstream err;

	double length_to_meter_factor = m_ifc_model->getUnitConverter()->getLengthInMeterFactor();
	carve::EPSILON = 1.4901161193847656e-08*length_to_meter_factor;

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
		std::map<int,shared_ptr<ProductShape> > *map_products_ptr = &m_product_shapes;
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
				if( !product->m_Representation )
				{
					continue;
				}

				const int product_id = product->getId();
				shared_ptr<ProductShape> product_shape( new ProductShape() );
				try
				{
					convertIfcProduct( product, product_shape );
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
					thread_err << "error in ReaderWriterIFC::convertIfcRelContainedInSpatialStructure, product id " << product_id << std::endl;
				}

				//#pragma omp critical
				omp_set_lock(&writelock_map);
				{
					map_products_ptr->insert( std::make_pair( product_id, product_shape ) );
					m_processed_products[product_id] = product;
			
					for( int opening_i = 0; opening_i<product_shape->vec_openings.size(); ++opening_i )
					{
						shared_ptr<IfcProduct>& opening_product = product_shape->vec_openings[opening_i];
						m_processed_products[opening_product->getId()] = opening_product;
					}
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
		const int product_id = product->getId();
		shared_ptr<ProductShape> product_shape( new ProductShape() );
		product_shape->ifc_product = product;

		if( !product->m_Representation )
		{
			continue;
		}

		try
		{
			convertIfcProduct( product, product_shape );
		}
		catch( IfcPPException& e)
		{
			err << e.what();
		}		
		catch( carve::exception& e)
		{
			err << e.str();
		}
		catch( std::exception& e )
		{
			err << e.what();
		}
		catch( ... )
		{
			err << "error in ReaderWriterIFC::convertIfcRelContainedInSpatialStructure, product id " << product_id << std::endl;
		}

		m_product_shapes[product_id] = product_shape;
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

	// now resolve spatial structure
	//ModelGeomTree result_geom;
	try
	{
		//resolveSpatialStructure( project, result_geom );
		//const int num_products = m_product_shapes.size();
		//for( int i=0; i<num_products; ++i )
		std::map<int,shared_ptr<ProductShape> >::iterator it_product_shapes;
		for( it_product_shapes = m_product_shapes.begin(); it_product_shapes!=m_product_shapes.end(); ++it_product_shapes )
		{
			shared_ptr<ProductShape> product_shape = it_product_shapes->second;
			shared_ptr<IfcProduct> ifc_product = product_shape->ifc_product;

			shared_ptr<IfcBuildingStorey> building_storey = dynamic_pointer_cast<IfcBuildingStorey>(ifc_product);
			if( building_storey )
			{
				int building_storey_id = building_storey->getId();
				double elevation = 0.0;
				if( building_storey->m_Elevation )
				{
					elevation = building_storey->m_Elevation->m_value;
				}
				osg::ref_ptr<osg::MatrixTransform> transform_building_storey = new osg::MatrixTransform( osg::Matrix::translate( 0, 0, elevation ) );

				std::stringstream storey_name;
				storey_name << "#" << building_storey_id << " IfcBuildingStorey group";
				transform_building_storey->setName( storey_name.str().c_str() );

				m_group_result->addChild( transform_building_storey );

				std::vector<weak_ptr<IfcRelContainedInSpatialStructure> >& vec_rel_contained = building_storey->m_ContainsElements_inverse;
				std::vector<weak_ptr<IfcRelContainedInSpatialStructure> >::iterator it_rel_contained;

				// TODO: collect all products for one building storey and then create geometry for products in each storey in parallel

				for( it_rel_contained=vec_rel_contained.begin(); it_rel_contained!=vec_rel_contained.end(); ++it_rel_contained )
				{
					shared_ptr<IfcRelContainedInSpatialStructure> rel_spatial_structure( *it_rel_contained );
					//if( rel->getId() == rel_spatial_structure->getId() )
					//{
					//	continue;
					//}

					std::vector<shared_ptr<IfcProduct> >& vec_related_elements = rel_spatial_structure->m_RelatedElements;
					std::vector<shared_ptr<IfcProduct> >::iterator it_product;
					std::vector<weak_ptr<IfcRelAggregates> >::iterator it_rel_aggregates;
					std::map<int,shared_ptr<ProductShape> >::iterator it_product_map;
					std::stringstream err;

					for( it_product=vec_related_elements.begin(); it_product!=vec_related_elements.end(); ++it_product )
					{
						shared_ptr<IfcProduct> ifc_product = (*it_product);
						if( !ifc_product )
						{
							continue;
						}
						shared_ptr<IfcFeatureElementSubtraction> opening = dynamic_pointer_cast<IfcFeatureElementSubtraction>(ifc_product);
						if( opening )
						{
							continue;
						}

						const int product_id = ifc_product->getId();

						it_product_map = m_product_shapes.find(product_id);
						if( it_product_map != m_product_shapes.end() )
						{
							shared_ptr<ProductShape>& product_shape = it_product_map->second;
							product_shape->added_to_storey = true;
							osg::ref_ptr<osg::Group> product_group = product_shape->product_group;
							if( product_group.valid() )
							{
								transform_building_storey->addChild( product_group );
							}
						}
					}
				}

					//try
					//{
					//	convertIfcRelContainedInSpatialStructure( rel, result_geom, visited_rel );

				
			}

		}

		osg::ref_ptr<osg::Group> group_outside_spatial_structure = new osg::Group();
		group_outside_spatial_structure->setName( "IfcProduct outside spatial structure" );
		m_group_result->addChild( group_outside_spatial_structure );

		for( it_product_shapes = m_product_shapes.begin(); it_product_shapes!=m_product_shapes.end(); ++it_product_shapes )
		{
			shared_ptr<ProductShape> product_shape = it_product_shapes->second;
			shared_ptr<IfcProduct> ifc_product = product_shape->ifc_product;
			if( !product_shape->added_to_storey )
			{
				shared_ptr<IfcFeatureElementSubtraction> opening = dynamic_pointer_cast<IfcFeatureElementSubtraction>(ifc_product);
				if( opening )
				{
					continue;
				}

				osg::ref_ptr<osg::Group> product_group = product_shape->product_group;
				if( product_group.valid() )
				{
					group_outside_spatial_structure->addChild( product_group );
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
		err << "error in ReaderWriterIFC::convertIfcProject" << std::endl;
	}

	// now process products that are not in the spatial structure
//#ifdef SHOW_ENTITIES_OUTSIDE_SPATIAL_STRUCTURE
//	std::map<int,shared_ptr<IfcProduct> >::iterator it_product;
//	for( it_product=m_product_shapes.begin(); it_product!=m_product_shapes.end(); ++it_product )
//	{
//		shared_ptr<IfcProduct> product = it_product->second;
//
//		if( m_processed_products.find( product->getId() ) == m_processed_products.end() )
//		{
//			try
//			{
//				convertIfcProduct( product, result_geom );
//			}
//			catch( IfcPPException& e)
//			{
//				err << e.what();
//			}
//			catch(std::exception& e)
//			{
//				err << e.what();
//			}
//			catch( ... )
//			{
//				err << "ReaderWriterIFC::convertIfcProduct at product id " << product->getId() << " failed" << std::endl;
//			}
//		}
//	}
//#endif

//	m_geom_tree.products->addChild( result_geom.products );
	//m_geom_tree.storeys->addChild( result_geom.storeys );
	//m_geom_tree.spaces->addChild( result_geom.spaces );
	//m_geom_tree.terrain->addChild( result_geom.terrain );

	float z_shift = 0;
	if( m_representation_converter->getDebugGroupFirst()->getNumChildren() > 0 )
	{
		osg::MatrixTransform* mt = new osg::MatrixTransform( osg::Matrix::translate(0, 0, z_shift) );
		//m_geom_tree.products->addChild(mt);
		mt->addChild( m_representation_converter->getDebugGroupFirst() );
	}
	if( m_representation_converter->getDebugGroupSecond()->getNumChildren() > 0 )
	{
		osg::Group* group_second = m_representation_converter->getDebugGroupSecond();
		osg::MatrixTransform* mt = new osg::MatrixTransform( osg::Matrix::translate(0, 0, z_shift) );
		//m_geom_tree.products->addChild(mt);
		mt->addChild( group_second );

		//WireFrameModeOn( debug_group_copy );
		group_second->setStateSet(m_glass_stateset);
	}

	progressCallback( 1.0 );

	if( err.tellp() > 0 )
	{
		throw IfcPPException( err.str().c_str() );
	}
}

/*
void ReaderWriterIFC::resolveSpatialStructure( shared_ptr<IfcProject> project )//, ModelGeomTree& result_geom )
{
	if( !project )
	{
		return;
	}
	std::string err;
	double progress = 0.0;
	progressCallback( progress );

	// resolve aggregated elements
	std::vector<weak_ptr<IfcRelAggregates> >& vec_decomposed = project->m_IsDecomposedBy_inverse;
	std::vector<weak_ptr<IfcRelAggregates> >::iterator it_rel_dec;

	for( it_rel_dec=vec_decomposed.begin(); it_rel_dec!=vec_decomposed.end(); ++it_rel_dec )
	{
		shared_ptr<IfcRelAggregates> rel( *it_rel_dec );
		try
		{
			std::set<int> visited_rel_aggregates;
			convertIfcRelAggregates( rel, result_geom, visited_rel_aggregates );
		}
		catch( IfcPPException& e )
		{
			err.append( e.what() );
		}
	}
	if( err.size() > 0 )
	{
		throw IfcPPException( err );
	}
}
*/


#ifdef _DEBUG
double failed_geom_z = -20;
#endif
// @brief creates geometry objects from an IfcProduct object
void ReaderWriterIFC::convertIfcProduct( shared_ptr<IfcProduct> product, shared_ptr<ProductShape>& product_shape )
{
	// IfcProduct needs to have a representation
	if( !product->m_Representation )
	{
		return;
	}
	if( dynamic_pointer_cast<IfcFeatureElementSubtraction>(product) )
	{
		return;
	}

	// caution: everything in here runs in parallel, so every write access to member variables needs a write lock

	const int product_id = product->getId();
	std::stringstream err;
	osg::ref_ptr<osg::Group> product_group = new osg::Group();
	std::stringstream group_name;
	group_name << "#" << product_id << " IfcProduct group";
	product_group->setName( group_name.str().c_str() );

	// IfcProduct has an ObjectPlacement that can be local or global
	double length_factor = m_unit_converter->getLengthInMeterFactor();
	carve::math::Matrix product_placement_matrix( carve::math::Matrix::IDENT() );

	if( product->m_ObjectPlacement )
	{
		// IfcPlacement2Matrix follows related placements in case of local coordinate systems
		std::set<int> placement_already_applied;
		PlacementConverter::convertIfcObjectPlacement( product->m_ObjectPlacement, product_placement_matrix, length_factor, placement_already_applied );
	}

	// evaluate geometry
	ConverterOSG carve_converter;
	shared_ptr<RepresentationData> product_representation_data( new RepresentationData() );
	shared_ptr<IfcProductRepresentation> product_representation = product->m_Representation;
	std::vector<shared_ptr<IfcRepresentation> >& vec_representations = product_representation->m_Representations;
	std::vector<shared_ptr<IfcRepresentation> >::iterator it_representations;
	for( it_representations=vec_representations.begin(); it_representations!=vec_representations.end(); ++it_representations )
	{
		shared_ptr<IfcRepresentation> representation = (*it_representations);

		try
		{
			std::set<int> visited_representation;
			m_representation_converter->convertIfcRepresentation( representation, product_placement_matrix, product_representation_data, visited_representation );
		}
		catch( IfcPPException& e )
		{
			err << e.what();
		}
		catch( std::exception& e)
		{
			err << e.what();
		}
		catch(...)
		{
			err << "convertIfcProduct: convertIfcRepresentation failed at product id " << product_id << std::endl;
		}
	}

	std::vector<shared_ptr<ItemData> >& product_items = product_representation_data->vec_item_data;
	for( int i_item=0; i_item<product_items.size(); ++i_item )
	{
		shared_ptr<ItemData> item_data = product_items[i_item];
		osg::Group* item_group = new osg::Group();

		// create polyhedrons from closed shell data
		for( int i=0; i<item_data->closed_shell_data.size(); ++i )
		{
			shared_ptr<carve::input::PolyhedronData>& polyhedron_data = item_data->closed_shell_data[i];
			if( polyhedron_data->getVertexCount() < 3 )
			{
				continue;
			}
			carve::input::Options carve_options;
			item_data->polyhedrons.push_back( shared_ptr<carve::mesh::MeshSet<3> >( polyhedron_data->createMesh(carve_options) ) );

			// TODO: check to avoid duplicate polyhedrons
		}

		// create shape for open shells
		for( int i=0; i<item_data->open_shell_data.size(); ++i )
		{
			shared_ptr<carve::input::PolyhedronData>& open_shell_data = item_data->open_shell_data[i];
			if( open_shell_data->getVertexCount() < 3 )
			{
				continue;
			}

			carve::input::Options carve_options;
			shared_ptr<carve::poly::Polyhedron> poly( open_shell_data->create(carve_options) );
			osg::ref_ptr<osg::Geode> geode = new osg::Geode();
			carve_converter.drawPolyhedron( poly, geode );
			item_group->addChild(geode);
		}

		// TODO: handle open_or_closed_shells

		// now go through all polyhedrons of the product
		for( int i=0; i<item_data->polyhedrons.size(); ++i )
		{
			shared_ptr<carve::mesh::MeshSet<3> >& product_meshset = item_data->polyhedrons[i];
			bool product_polyhedron_ok = ConverterOSG::checkPolyHedron( product_meshset, err, product_id );
			if( !product_polyhedron_ok )
			{
				continue;
			}

			// check for openings
			shared_ptr<IfcElement> element = dynamic_pointer_cast<IfcElement>(product);
			if( element )
			{
				std::vector<weak_ptr<IfcRelVoidsElement> > vec_rel_voids( element->m_HasOpenings_inverse );

				if( vec_rel_voids.size() > 0 )
				{
					shared_ptr<carve::mesh::MeshSet<3> > polyhedron_openings;
					for( int i=0; i<vec_rel_voids.size(); ++i )
					{
						shared_ptr<IfcRelVoidsElement> rel_voids( vec_rel_voids[i] );
						shared_ptr<IfcFeatureElementSubtraction> opening = rel_voids->m_RelatedOpeningElement;
						if( !opening )
						{
							continue;
						}
						if( !opening->m_Representation )
						{
							continue;
						}

						// opening is a product, so remember it as processed
						int opening_id = opening->getId();
						//product_shape->vec_openings.push_back( opening );

						// opening can have its own relative placement
						shared_ptr<IfcObjectPlacement>	opening_placement = opening->m_ObjectPlacement;			//optional
						carve::math::Matrix opening_placement_matrix( carve::math::Matrix::IDENT() );
						if( opening_placement )
						{
							std::set<int> opening_placements_applied;
							PlacementConverter::convertIfcObjectPlacement( opening_placement, opening_placement_matrix, length_factor, opening_placements_applied );
						}

						carve::csg::CSG csg;
						std::vector<shared_ptr<IfcRepresentation> >& vec_opening_representations = opening->m_Representation->m_Representations;
						std::vector<shared_ptr<IfcRepresentation> >::iterator it_representations;
						for( it_representations=vec_opening_representations.begin(); it_representations!=vec_opening_representations.end(); ++it_representations )
						{
							shared_ptr<IfcRepresentation> opening_representation = (*it_representations);

							// TODO: bounding box test
							shared_ptr<RepresentationData> opening_representation_data( new RepresentationData() );
							try
							{
								// TODO: Representation caching, one element could be used for several openings
								std::set<int> visited_representation;
								m_representation_converter->convertIfcRepresentation( opening_representation, opening_placement_matrix, opening_representation_data, visited_representation );
							}
							catch( IfcPPException& e )
							{
								err << e.what();
							}
							catch( carve::exception& ce )
							{
								err << ce.str();
							}
							catch( std::exception& e )
							{
								err << e.what();
							}
							catch(...)
							{
								err << "convertIfcProduct: convertIfcRepresentation failed at opening id " << opening_id << std::endl;
							}

							std::vector<shared_ptr<ItemData> >& opening_representation_items = opening_representation_data->vec_item_data;
							for( int i_item=0; i_item<opening_representation_items.size(); ++i_item )
							{
								shared_ptr<ItemData> opening_item_data = opening_representation_items[i_item];

								std::vector<shared_ptr<carve::input::PolyhedronData> >::iterator it_opening_polyhedron_data = opening_item_data->closed_shell_data.begin();
								for( ; it_opening_polyhedron_data != opening_item_data->closed_shell_data.end(); ++it_opening_polyhedron_data )
								{
									shared_ptr<carve::input::PolyhedronData>& polyhedron_data = (*it_opening_polyhedron_data);
									if( polyhedron_data->getVertexCount() < 3 )
									{
										continue;
									}

									carve::input::Options carve_options;
									shared_ptr<carve::mesh::MeshSet<3> > opening_poly_single( polyhedron_data->createMesh(carve_options) );

									if( !polyhedron_openings )
									{
										polyhedron_openings = opening_poly_single;
									}
									else
									{
										try
										{
											if( polyhedron_openings->meshes.size() && opening_poly_single->meshes.size() > 0 )
											{
												polyhedron_openings =  shared_ptr<carve::mesh::MeshSet<3> >( csg.compute( polyhedron_openings.get(), opening_poly_single.get(), carve::csg::CSG::UNION, NULL, carve::csg::CSG::CLASSIFY_NORMAL) );
											}
										}
										catch( IfcPPException& e )
										{
											err << e.what();
										}
										catch( carve::exception& ce )
										{
											err << ce.str();
										}
										catch( std::exception& e )
										{
											err << e.what();
										}
										catch(...)
										{
											err << "convertIfcProduct: opening csg operation failed at product id " << product_id << std::endl;
										}
									}
								}
							}
						}
					}

					
					bool opening_polyhedron_ok = ConverterOSG::checkPolyHedron( polyhedron_openings, err, product_id );

					if( !opening_polyhedron_ok )
					{
						continue;
					}


					// do the subtraction
					carve::csg::CSG csg;
					try
					{
						if( product_meshset->meshes.size() && polyhedron_openings->meshes.size() > 0 )
						{
							shared_ptr<carve::mesh::MeshSet<3> > result( csg.compute( product_meshset.get(), polyhedron_openings.get(), carve::csg::CSG::A_MINUS_B, NULL, carve::csg::CSG::CLASSIFY_NORMAL) );
							bool result_polyhedron_ok = ConverterOSG::checkPolyHedron( result, err, product_id );
						
							if( result_polyhedron_ok )
							{
								// continue with current polyhedron
								product_meshset = result;
							}
							else
							{

#ifdef _DEBUG
							osg::ref_ptr<osg::Geode> geode = new osg::Geode();
							carve_converter.drawMeshSet( product_meshset, geode );
							osg::ref_ptr<osg::MatrixTransform> mt = new osg::MatrixTransform( osg::Matrix::translate(0, 0, failed_geom_z ) );
							mt->addChild(geode);
							product_group->addChild(mt);
							failed_geom_z -= 1;

							osg::ref_ptr<osg::Geode> geode_opnening = new osg::Geode();
							carve_converter.drawMeshSet( polyhedron_openings, geode_opnening );
							mt->addChild(geode_opnening);
#endif
							}
						}
					}
					catch( IfcPPException& e )
					{
						err << e.what();
					}
					catch( carve::exception& ce )
					{
						err << ce.str();
					}
					catch( std::exception& e )
					{
						err << e.what();
					}
					catch(...)
					{
						err << "convertIfcProduct: csg operation failed at product id " << product_id << std::endl;
#ifdef _DEBUG
						osg::ref_ptr<osg::Geode> geode = new osg::Geode();
						carve_converter.drawMeshSet( product_meshset, geode );
						osg::ref_ptr<osg::MatrixTransform> mt = new osg::MatrixTransform( osg::Matrix::translate(0, 0, failed_geom_z ) );
						mt->addChild(geode);
						product_group->addChild(mt);
						failed_geom_z -= 1;
#endif
					}
				}
			}

			osg::ref_ptr<osg::Geode> geode_result = new osg::Geode();
			carve_converter.drawMeshSet( product_meshset, geode_result );
			item_group->addChild(geode_result);
		}

		for( int polyline_i = 0; polyline_i < item_data->polyline_data.size(); ++polyline_i )
		{
			shared_ptr<carve::input::PolylineSetData>& polyline_data = item_data->polyline_data.at(polyline_i);
			osg::ref_ptr<osg::Geode> geode = new osg::Geode();
			carve_converter.drawPolyline( polyline_data, geode );
			item_group->addChild(geode);
		}

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

		if( item_group->getNumChildren() > 0 )
		{
			product_group->addChild(item_group);
		}
	}

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
					m_representation_converter->convertIfcPropertySet( property_set, product_group.get() );
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
							m_representation_converter->convertIfcPropertySet( property_set, product_group.get() );
						}
					}
				}
				
				continue;
			}

		}
	}

	for( int i=0; i<product_representation_data->statesets.size(); ++i )
	{
		osg::StateSet* next_product_stateset = product_representation_data->statesets[i];
		if( !next_product_stateset )
		{
			continue;
		}

		osg::StateSet* existing_item_stateset = product_group->getStateSet();


		if( existing_item_stateset )
		{
			osg::StateSet* merged_product_stateset = new osg::StateSet( *existing_item_stateset );
			merged_product_stateset->merge( *next_product_stateset );
			product_group->setStateSet( merged_product_stateset );
		}
		else
		{
			product_group->setStateSet( next_product_stateset );
		}
	}



	// enable transparency for certain objects
	if( dynamic_pointer_cast<IfcSpace>(product) )
	{
		//product_shape->space_group = product_group;
		product_group->setStateSet( m_glass_stateset );
	}
	else if( dynamic_pointer_cast<IfcCurtainWall>(product) || dynamic_pointer_cast<IfcWindow>(product) )
	{
		// TODO: make only glass part of window transparent
		product_group->setStateSet( m_glass_stateset );
		//product_shape->product_group = product_group;
	}
	// TODO: handle storeys separately
	else if( dynamic_pointer_cast<IfcSite>(product) )
	{
		std::stringstream group_name;
		group_name << "#" << product_id << " IfcSite-Representation-Terrain";
		product_group->setName( group_name.str().c_str() );
		//product_shape->terrain_group = product_group;
		//product_shape->product_group = product_group;
	}
	//else
	{
		if( product_group->getNumChildren() > 0 )
		{
			product_shape->product_group = product_group;
		}
	}
	
	if( err.tellp() > 0 )
	{
		throw IfcPPException( err.str().c_str() );
	}
}


/*
void ReaderWriterIFC::convertIfcRelContainedInSpatialStructure( shared_ptr<IfcRelContainedInSpatialStructure> rel_spatial_structure, ModelGeomTree& result_geom, std::set<int>& visited_rel )
{
	if( visited_rel.find( rel_spatial_structure->getId() ) != visited_rel.end() )
	{
		return;
	}
	visited_rel.insert( rel_spatial_structure->getId() );

	std::vector<shared_ptr<IfcProduct> >& vec_related_elements = rel_spatial_structure->m_RelatedElements;
	std::vector<shared_ptr<IfcProduct> >::iterator it_product;
	std::vector<weak_ptr<IfcRelAggregates> >::iterator it_rel_aggregates;
	std::map<int,shared_ptr<ProductShape> >::iterator it_product_map;
	std::stringstream err;

	for( it_product=vec_related_elements.begin(); it_product!=vec_related_elements.end(); ++it_product )
	{
		shared_ptr<IfcProduct> product = (*it_product);
		if( !product )
		{
			continue;
		}
		const int product_id = product->getId();

		it_product_map = m_product_shapes.find(product_id);
		if( it_product_map != m_product_shapes.end() )
		{
			shared_ptr<ProductShape>& product_group = it_product_map->second;

			if( product_group->space_group.valid() )
			{
				result_geom.spaces->addChild( product_group->space_group );
			}
			if( product_group->terrain_group.valid() )
			{
				result_geom.terrain->addChild( product_group->terrain_group );
			}
			if( product_group->product_group.valid() )
			{
				result_geom.products->addChild( product_group->product_group );
			}
		}

		if( dynamic_pointer_cast<IfcSpatialStructureElement>(product) )
		{
			shared_ptr<IfcSpatialStructureElement> spatial_structure_element = dynamic_pointer_cast<IfcSpatialStructureElement>(product);

			// check for Elevation attribute in case of a building storey
			shared_ptr<IfcBuildingStorey> building_storey = dynamic_pointer_cast<IfcBuildingStorey>(spatial_structure_element);
			if( building_storey )
			{
				if( building_storey->m_Elevation )
				{
					double elevation = building_storey->m_Elevation->m_value;
					osg::ref_ptr<osg::MatrixTransform> transform_building_storey = new osg::MatrixTransform( osg::Matrix::translate( 0, 0, elevation ) );
					

					// TODO: assign to result transform group
				}

				//int building_storey_id = building_storey->getId();
				//if( m_building_storeys.find(building_storey_id) == m_building_storeys.end() )
				//{
				//	m_building_storeys[building_storey_id] = shared_ptr<BuildingStoreyGroup>( new BuildingStoreyGroup() );
				//}
			}

			if( spatial_structure_element->m_ContainsElements_inverse.size() > 0 )
			{
				std::vector<weak_ptr<IfcRelContainedInSpatialStructure> >& vec_rel_contained = spatial_structure_element->m_ContainsElements_inverse;
				std::vector<weak_ptr<IfcRelContainedInSpatialStructure> >::iterator it_rel_contained;

				// TODO: collect all products for one building storey and then create geometry for products in each storey in parallel

				for( it_rel_contained=vec_rel_contained.begin(); it_rel_contained!=vec_rel_contained.end(); ++it_rel_contained )
				{
					shared_ptr<IfcRelContainedInSpatialStructure> rel( *it_rel_contained );
					if( rel->getId() == rel_spatial_structure->getId() )
					{
						continue;
					}

					try
					{
						convertIfcRelContainedInSpatialStructure( rel, result_geom, visited_rel );
					}
					catch( IfcPPException& e )
					{
						err << e.what();
					}
					catch( carve::exception& ce )
					{
						err << ce.str();
					}
					catch( std::exception& e )
					{
						err << e.what();
					}
					catch( ... )
					{
						err << "error in ReaderWriterIFC::convertIfcRelContainedInSpatialStructure, product id " << product->getId() << std::endl;
					}
				}
			}
		}

		// further child object related by IfcRelAggregates
		if( product->m_IsDecomposedBy_inverse.size() > 0 )
		{
			std::vector<weak_ptr<IfcRelAggregates> >& vec_rel_aggregates = product->m_IsDecomposedBy_inverse;
			for( it_rel_aggregates=vec_rel_aggregates.begin(); it_rel_aggregates!=vec_rel_aggregates.end(); ++it_rel_aggregates )
			{
				shared_ptr<IfcRelAggregates> rel_aggregates( *it_rel_aggregates );
				try
				{
					convertIfcRelAggregates( rel_aggregates, result_geom, visited_rel );
				}
				catch( IfcPPException& e )
				{
					err << e.what();
				}
				catch( carve::exception& ce )
				{
					err << ce.str();
				}
				catch( std::exception& e )
				{
					err << e.what();
				}
				catch( ... )
				{
					err << "error in ReaderWriterIFC::convertIfcRelContainedInSpatialStructure, product id " << product->getId() << std::endl;
				}
			}
		}
	}
	if( err.tellp() > 0 )
	{
		throw IfcPPException( err.str().c_str() );
	}
}
*/

/*
void ReaderWriterIFC::convertIfcRelAggregates( shared_ptr<IfcRelAggregates> rel_aggregates, ModelGeomTree& result_geom, std::set<int>& visited_rel )
{
	if( visited_rel.find( rel_aggregates->getId() ) != visited_rel.end() )
	{
		return;
	}
	visited_rel.insert( rel_aggregates->getId() );

	std::vector<shared_ptr<IfcObjectDefinition> >::iterator				it_rel_obj;
	std::vector<weak_ptr<IfcRelAggregates> >::iterator					it_rel_aggregates;
	std::vector<weak_ptr<IfcRelContainedInSpatialStructure> >::iterator it_rel_contained;
	std::vector<shared_ptr<IfcProduct> >::iterator						it_ifc_product;
	std::map<int,shared_ptr<ProductShape> >::iterator					it_product_map;
	std::stringstream err;

	std::vector<shared_ptr<IfcObjectDefinition> >& vec_related_objects = rel_aggregates->m_RelatedObjects;
	for( it_rel_obj=vec_related_objects.begin(); it_rel_obj!=vec_related_objects.end(); ++it_rel_obj )
	{
		shared_ptr<IfcObjectDefinition> obj_def = (*it_rel_obj);

		// check if object is an IfcProduct and if it has a representation
		shared_ptr<IfcProduct> product = dynamic_pointer_cast<IfcProduct>(obj_def);
		if( product )
		{
			const int product_id = product->getId();
			if( product->m_Representation )
			{
				it_product_map = m_product_shapes.find(product_id);
				if( it_product_map != m_product_shapes.end() )
				{
					shared_ptr<ProductShape>& product_group = it_product_map->second;

					if( product_group->space_group.valid() )
					{
						result_geom.spaces->addChild( product_group->space_group );
					}
					if( product_group->terrain_group.valid() )
					{
						result_geom.terrain->addChild( product_group->terrain_group );
					}
					if( product_group->product_group.valid() )
					{
						result_geom.products->addChild( product_group->product_group );
					}
				}
			}
		}

		// further child object related by IfcSpatialStructureElement
		shared_ptr<IfcSpatialStructureElement> spatial_structure_element = dynamic_pointer_cast<IfcSpatialStructureElement>(obj_def);
		if( spatial_structure_element )
		{
			if( spatial_structure_element->m_ContainsElements_inverse.size() > 0 )
			{
				std::vector<weak_ptr<IfcRelContainedInSpatialStructure> >& vec_rel_contained = spatial_structure_element->m_ContainsElements_inverse;

				for( it_rel_contained=vec_rel_contained.begin(); it_rel_contained!=vec_rel_contained.end(); ++it_rel_contained )
				{
					shared_ptr<IfcRelContainedInSpatialStructure> rel_contained( *it_rel_contained );
					try
					{
						convertIfcRelContainedInSpatialStructure( rel_contained, result_geom, visited_rel );
					}
					catch( IfcPPException& e )
					{
						err << e.what();
					}
					catch(std::exception& e)
					{
						err << e.what();
					}
					catch( ... )
					{
						err << "ReaderWriterIFC::convertIfcRelContainedInSpatialStructure failed" << std::endl;
					}
				}
			}
		}

		// further child object related by IfcRelAggregates
		if( obj_def->m_IsDecomposedBy_inverse.size() > 0 )
		{
			std::vector<weak_ptr<IfcRelAggregates> >& vec_rel_aggregates = obj_def->m_IsDecomposedBy_inverse;

			for( it_rel_aggregates=vec_rel_aggregates.begin(); it_rel_aggregates!=vec_rel_aggregates.end(); ++it_rel_aggregates )
			{
				shared_ptr<IfcRelAggregates> rel( *it_rel_aggregates );
				if( rel->getId() == rel_aggregates->getId() )
				{
					continue;
				}

				try
				{
					convertIfcRelAggregates( rel, result_geom, visited_rel );
				}
				catch( IfcPPException& e )
				{
					err << e.what();
				}
				catch(std::exception& e)
				{
					err << e.what();
				}
				catch( ... )
				{
					err << "ReaderWriterIFC::convertIfcRelAggregates failed" << std::endl;
				}
			}
		}
	}
	if( err.tellp() > 0 )
	{
		throw IfcPPException( err.str().c_str() );
	}
}
*/

void ReaderWriterIFC::slotProgressValueWrapper( void* ptr, double progress_value )
{
	ReaderWriterIFC* myself = (ReaderWriterIFC*)ptr;

	myself->progressCallback( progress_value );
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
