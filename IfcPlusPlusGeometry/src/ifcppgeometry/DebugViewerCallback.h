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

#ifdef  _DEBUG

#include <osg/Vec4>
#include "carve/mesh.hpp"
#include "carve/input.hpp"
#include "ifcpp/model/shared_ptr.h"
#include "GeometryInputData.h"

void setRenderMeshsetCallBack( void* obj_ptr, void (*func)(void*, const shared_ptr<carve::mesh::MeshSet<3> >& meshset, const osg::Vec4f& color, const bool wireframe) );
void setRenderPolylineCallBack( void* obj_ptr, void (*func)(void*, const shared_ptr<carve::input::PolylineSetData >& poly_line, const osg::Vec4f& color ) );
void renderMeshsetInDebugViewer(const shared_ptr<carve::mesh::MeshSet<3> >& meshset, const osg::Vec4f& color, const bool wireframe);
void renderMeshsetInDebugViewer( const shared_ptr<ShapeInputData>& product_shape, const osg::Vec4f& color, const bool wireframe );
void renderPolylineInDebugViewer( shared_ptr<carve::input::PolylineSetData >& poly_line, osg::Vec4f& color );

#endif

