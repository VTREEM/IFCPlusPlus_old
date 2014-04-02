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

#include <vector>
#include <ifcpp/model/shared_ptr.h>
#include <ifcpp/IFC4/include/IfcProduct.h>
#include <ifcpp/IFC4/include/IfcRepresentation.h>
#include "IncludeCarveHeaders.h"

#include <osg/ref_ptr>
#include <osg/StateSet>
#include <osg/Switch>

//\brief Class to hold input data of one IFC geometric representation item.
class ItemData
{
public:
	std::vector<shared_ptr<carve::input::PolyhedronData> >	closed_polyhedrons;
	std::vector<shared_ptr<carve::input::PolyhedronData> >	open_polyhedrons;
	std::vector<shared_ptr<carve::input::PolyhedronData> >	open_or_closed_polyhedrons;
	std::vector<shared_ptr<carve::input::PolylineSetData> > polylines;
	std::vector<shared_ptr<carve::mesh::MeshSet<3> > >		meshsets;
	//std::vector<shared_ptr<carve::mesh::MeshSet<3> > >	meshsets_open;
	std::vector<osg::ref_ptr<osg::StateSet> >				statesets;
	void createMeshSetsFromClosedPolyhedrons();
	void applyPosition( carve::math::Matrix& mat );
};

struct PlacementData
{
	std::set<int> placement_already_applied;
	carve::math::Matrix pos;
};

class ShapeInputData
{
public:
	ShapeInputData() { added_to_storey = false; }
	~ShapeInputData() {}

	void addInputData( shared_ptr<ShapeInputData>& other );

	shared_ptr<IfcProduct> ifc_product;
	shared_ptr<IfcRepresentation> representation;
	shared_ptr<IfcObjectPlacement> object_placement;
	osg::ref_ptr<osg::Switch> product_switch;
	osg::ref_ptr<osg::Group> space_group;
	//std::vector<shared_ptr<IfcProduct> > vec_openings;
	
	std::vector<shared_ptr<ItemData> >			vec_item_data;
	std::vector<osg::ref_ptr<osg::StateSet> >	vec_statesets;
	bool added_to_storey;
};
