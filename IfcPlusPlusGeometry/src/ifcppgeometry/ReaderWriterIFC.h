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

#pragma once

#include <sstream>
#include <osgDB/ReaderWriter>

#include <osg/Group>
#include <osg/MatrixTransform>

#include "ifcpp/model/shared_ptr.h"
#include "ifcpp/model/StatusObservable.h"

#ifdef __GNUC__
#include "ifcpp/model/IfcPPModel.h"
#include "ifcpp/IFC4/include/AllIfcEntities.h"
#include "RepresentationConverter.h"
#else

class IfcPPModel;
class IfcPlusPlusReader;
class IfcStepWriter;
class UnitConverter;
class RepresentationConverter;
class IfcProject;
class IfcProduct;
class IfcRelContainedInSpatialStructure;
class IfcRelAggregates;
class IfcBuildingStorey;
#endif

class ReaderWriterIFC : public osgDB::ReaderWriter, public StatusObservable
{
public:

	//class ModelGeomTree
	//{
	//public:
	//	ModelGeomTree()
	//	{
	//		
	//		products = new osg::Group();
	//		spaces = new osg::Group();
	//		terrain = new osg::Group();
	//	}
	//	void clear()
	//	{
	//		//storeys->removeChildren(0,storeys->getNumChildren());
	//		products->removeChildren(0,products->getNumChildren());
	//		spaces->removeChildren(0,spaces->getNumChildren());
	//		terrain->removeChildren(0,terrain->getNumChildren());
	//	}
	//	
	//	//osg::ref_ptr<osg::Group> products;
	//	//osg::ref_ptr<osg::Group> spaces;
	//	//osg::ref_ptr<osg::Group> terrain;

	//	std::vector<shared_ptr<BuildingStoreyGroup> > vec_building_storeys;
	//};

	class ProductShape
	{
	public:
		ProductShape() { added_to_storey = false; }

		shared_ptr<IfcProduct> ifc_product;
		osg::ref_ptr<osg::Group> product_group;
		osg::ref_ptr<osg::Group> space_group;
		//osg::ref_ptr<osg::Group> terrain_group;
		std::vector<shared_ptr<IfcProduct> > vec_openings;
		bool added_to_storey;
	};
	class BuildingStoreyGroup
	{
	public:
		BuildingStoreyGroup()
		{
			storey_transform = new osg::MatrixTransform();
		}
		~BuildingStoreyGroup(){}
		shared_ptr<IfcBuildingStorey> ifc_building_storey;
		std::vector<shared_ptr<ProductShape> > vec_products;
		osg::ref_ptr<osg::MatrixTransform> storey_transform;
	};

	void convertIfcModel();
	//void resolveSpatialStructure(	shared_ptr<IfcProject> project );//, ModelGeomTree& result_geom );
	void convertIfcProduct(	shared_ptr<IfcProduct> product, shared_ptr<ProductShape>& product_shape );
//	void convertIfcRelAggregates( shared_ptr<IfcRelAggregates> rel_aggregates, std::set<int>& visited_rel );
	//void convertIfcRelContainedInSpatialStructure( shared_ptr<IfcRelContainedInSpatialStructure> relc, ModelGeomTree& result_geom, std::set<int>& visited_rel );

	ReaderWriterIFC();
	~ReaderWriterIFC();	
	virtual const char* className() { return "ReaderWriterIFC"; }
	virtual osgDB::ReaderWriter::ReadResult readNode(const std::string& filename, const osgDB::ReaderWriter::Options*);

	void reset();
	std::vector<shared_ptr<BuildingStoreyGroup> >& getBuildingStoreys() { return m_vec_building_storeys; }
	//ModelGeomTree& getModelGeometryTree() { return m_geom_tree; }
	void setModel( shared_ptr<IfcPPModel> model );
	shared_ptr<IfcPPModel> getIfcPPModel() { return m_ifc_model; }
	shared_ptr<IfcPlusPlusReader> getIfcPPReader() { return m_step_reader; }
	shared_ptr<IfcStepWriter> getIfcPPWriter() { return m_step_writer; }
	std::stringstream& getErrors() { return m_err; }
	std::stringstream& getMessages() { return m_messages; }

	static void slotProgressValueWrapper( void* obj_ptr, double value );
	static void slotMessageWrapper( void* obj_ptr, const std::string& str );
	static void slotErrorWrapper( void* obj_ptr, const std::string& str );

protected:
	shared_ptr<IfcPPModel>		m_ifc_model;
	shared_ptr<IfcPlusPlusReader>		m_step_reader;
	shared_ptr<IfcStepWriter>		m_step_writer;
	std::stringstream				m_err;
	std::stringstream				m_messages;
	//ModelGeomTree					m_geom_tree;

	shared_ptr<UnitConverter>			m_unit_converter;
	shared_ptr<RepresentationConverter> m_representation_converter;
	osg::ref_ptr<osg::StateSet>			m_glass_stateset;

	std::map<int,shared_ptr<ProductShape> > m_product_shapes;
	std::map<int,shared_ptr<IfcProduct> > m_processed_products;
	//std::map<int, shared_ptr<BuildingStoreyGroup> > m_building_storeys;

	osg::ref_ptr<osg::Group> m_group_result;
	std::vector<shared_ptr<BuildingStoreyGroup> > m_vec_building_storeys;
	double m_recent_progress;
};
