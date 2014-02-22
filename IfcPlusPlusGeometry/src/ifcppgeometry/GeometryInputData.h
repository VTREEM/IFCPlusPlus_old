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

//! @author Fabian Gerold
//! @date 2013-12-02

#pragma once

#include <vector>
#include <ifcpp/model/shared_ptr.h>
#include <ifcpp/IFC4/include/IfcProduct.h>
#include <carve/input.hpp>

#include <osg/ref_ptr>
#include <osg/StateSet>
#include <osg/Switch>

class ItemData
{
public:
	std::vector<shared_ptr<carve::input::PolyhedronData> >	item_closed_mesh_data;
	std::vector<shared_ptr<carve::input::PolyhedronData> >	item_open_mesh_data;
	std::vector<shared_ptr<carve::input::PolyhedronData> >	item_open_or_closed_mesh_data;
	std::vector<shared_ptr<carve::input::PolylineSetData> > item_polyline_data;
	std::vector<shared_ptr<carve::mesh::MeshSet<3> > >		item_meshsets;
	std::vector<osg::ref_ptr<osg::StateSet> >				item_statesets;
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

	shared_ptr<IfcProduct> ifc_product;
	osg::ref_ptr<osg::Switch> product_switch;
	osg::ref_ptr<osg::Group> space_group;
	std::vector<shared_ptr<IfcProduct> > vec_openings;

	std::vector<shared_ptr<ItemData> >			vec_item_data;
	std::vector<osg::ref_ptr<osg::StateSet> >	vec_statesets;
	bool added_to_storey;
};
