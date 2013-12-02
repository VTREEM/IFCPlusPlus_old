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

#include <osg/Switch>
#include <osg/MatrixTransform>
#include <osgViewer/Viewer>

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
class RepresentationData;
class IfcProject;
class IfcProduct;
class IfcRelContainedInSpatialStructure;
class IfcRelAggregates;
class IfcBuildingStorey;
#endif

class ReaderWriterIFC : public osgDB::ReaderWriter, public StatusObservable
{
public:
	class ProductShape
	{
	public:
		ProductShape() { added_to_storey = false; }

		shared_ptr<IfcProduct> ifc_product;
		osg::ref_ptr<osg::Switch> product_switch;
		osg::ref_ptr<osg::Group> space_group;
		std::vector<shared_ptr<IfcProduct> > vec_openings;
		shared_ptr<RepresentationData> representation_data;
		bool added_to_storey;
	};

	void createGeometry();
	void convertIfcProduct(	shared_ptr<IfcProduct> product, shared_ptr<ProductShape>& product_shape );
	void resolveProjectStructure( shared_ptr<IfcPPObject> obj, osg::Group* parent_group );

	ReaderWriterIFC();
	~ReaderWriterIFC();	
	virtual const char* className() { return "ReaderWriterIFC"; }
	virtual osgDB::ReaderWriter::ReadResult readNode(const std::string& filename, const osgDB::ReaderWriter::Options*);

	void reset();
	void setModel( shared_ptr<IfcPPModel> model );
	shared_ptr<IfcPPModel> getIfcPPModel() { return m_ifc_model; }
	shared_ptr<IfcPlusPlusReader> getIfcPPReader() { return m_step_reader; }
	shared_ptr<IfcStepWriter> getIfcPPWriter() { return m_step_writer; }
	shared_ptr<RepresentationConverter> getRepresentationConverter() { return m_representation_converter; }
	std::map<int,shared_ptr<ProductShape> >& getProductShapeData() { return m_product_shapes; }
	std::stringstream& getErrors() { return m_err; }
	std::stringstream& getMessages() { return m_messages; }
	void setNumVerticesPerCircle( int num_circles );
	void resetNumVerticesPerCircle();

	static void slotProgressValueWrapper( void* obj_ptr, double value );
	static void slotProgressTextWrapper( void* obj_ptr, const std::string& str );
	static void slotMessageWrapper( void* obj_ptr, const std::string& str );
	static void slotErrorWrapper( void* obj_ptr, const std::string& str );

#ifdef _DEBUG
	osgViewer::Viewer* m_debug_viewer;
#endif

protected:
	shared_ptr<IfcPPModel>				m_ifc_model;
	shared_ptr<IfcPlusPlusReader>		m_step_reader;
	shared_ptr<IfcStepWriter>			m_step_writer;
	std::stringstream					m_err;
	std::stringstream					m_messages;
	bool								m_keep_geom_input_data;
	int									m_num_vertices_per_circle;

	shared_ptr<UnitConverter>			m_unit_converter;
	shared_ptr<RepresentationConverter> m_representation_converter;
	osg::ref_ptr<osg::StateSet>			m_glass_stateset;

	std::map<int,shared_ptr<ProductShape> > m_product_shapes;
	std::map<int,shared_ptr<IfcProduct> > m_processed_products;

	std::map<int, shared_ptr<IfcPPObject> > m_map_visited;
	osg::ref_ptr<osg::Group> m_group_result;
	double m_recent_progress;
};
