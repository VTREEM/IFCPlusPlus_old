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

#include "GeometrySettings.h"

GeometrySettings::GeometrySettings()
{
	m_num_vertices_per_circle = 20;
	m_min_num_vertices_per_arc = 6;

	m_min_colinearity = 0.01;
	m_min_delta_v = 0.01;
	m_min_normal_angle = 0.01;
	m_use_mesh_simplifier_after_csg = false;
	m_use_mesh_simplifier_before_csg = false;
	m_use_mesh_simplifier_before_draw = false;
	m_set_process_output_face = true;
	m_classify_type = carve::csg::CSG::CLASSIFY_NORMAL;
}

GeometrySettings::~GeometrySettings()
{
}
