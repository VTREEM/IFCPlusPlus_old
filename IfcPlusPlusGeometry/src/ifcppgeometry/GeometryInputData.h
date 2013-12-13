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
#include <carve/input.hpp>

#include <osg/ref_ptr>
#include <osg/StateSet>

class ItemData
{
public:
	std::vector<shared_ptr<carve::input::PolyhedronData> >	closed_mesh_data;
	std::vector<shared_ptr<carve::input::PolyhedronData> >	open_mesh_data;
	std::vector<shared_ptr<carve::input::PolyhedronData> >	open_or_closed_mesh_data;
	std::vector<shared_ptr<carve::input::PolylineSetData> > polyline_data;
	std::vector<shared_ptr<carve::mesh::MeshSet<3> > >		meshsets;
	std::vector<osg::ref_ptr<osg::StateSet> >				statesets;
};

class RepresentationData
{
public:
	std::vector<shared_ptr<ItemData> >			vec_item_data;
	std::vector<osg::ref_ptr<osg::StateSet> >	statesets;
};

struct PlacementData
{
	std::set<int> placement_already_applied;
	carve::math::Matrix pos;
};
