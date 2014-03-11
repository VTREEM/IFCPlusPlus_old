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

#include "DebugViewerCallback.h"

#ifdef  _DEBUG

void* obj_call_on_render_meshset = NULL;
void (*func_call_on_render_meshset)(void*, const shared_ptr<carve::mesh::MeshSet<3> >& meshset, const shared_ptr<carve::input::PolyhedronData>& poly, const osg::Vec4f& color, const bool wireframe) = NULL;
void (*func_call_on_render_polyline)(void*, const shared_ptr<carve::input::PolylineSetData >& poly_line, const osg::Vec4f& color) = NULL;

void setRenderMeshsetCallBack( void* obj_ptr, void (*func)(void*, const shared_ptr<carve::mesh::MeshSet<3> >& meshset, const shared_ptr<carve::input::PolyhedronData>& poly, const osg::Vec4f& color, const bool wireframe ) )
{
	obj_call_on_render_meshset = obj_ptr;
	func_call_on_render_meshset = func;
}

void setRenderPolylineCallBack( void* obj_ptr, void (*func)(void*, const shared_ptr<carve::input::PolylineSetData >& poly_line, const osg::Vec4f& color ) )
{
	obj_call_on_render_meshset = obj_ptr;
	func_call_on_render_polyline = func;
}

void renderMeshsetInDebugViewer( const shared_ptr<ShapeInputData>& input_data, const osg::Vec4f& color, const bool wireframe )
{
	for( int i=0; i<input_data->vec_item_data.size(); ++i )
	{
		const shared_ptr<ItemData>&	item_data = input_data->vec_item_data[i];
		//item_data->createMeshSetsFromClosedPolyhedrons();

		std::vector<shared_ptr<carve::input::PolyhedronData> >& vec_poly_data = item_data->item_closed_mesh_data;
		for( int j=0; j<vec_poly_data.size(); ++j )
		{
			shared_ptr<carve::input::PolyhedronData> poly_data = vec_poly_data[j];
			shared_ptr<carve::mesh::MeshSet<3> > meshset( poly_data->createMesh(carve::input::opts()) );
			renderMeshsetInDebugViewer( meshset, poly_data, color, wireframe );
		}

		for( int j=0; j<item_data->item_meshsets.size(); ++j )
		{
			shared_ptr<carve::input::PolyhedronData> poly_data;
			renderMeshsetInDebugViewer( item_data->item_meshsets[j], poly_data, color, wireframe );
		}
	}
}

void renderMeshsetInDebugViewer( const shared_ptr<carve::mesh::MeshSet<3> >& meshset, shared_ptr<carve::input::PolyhedronData>& poly, const osg::Vec4f& color, const bool wireframe )
{
	if( func_call_on_render_meshset )
	{
		if( obj_call_on_render_meshset )
		{
			func_call_on_render_meshset( obj_call_on_render_meshset, meshset, poly, color, wireframe );
		}
	}
}

void renderMeshsetInDebugViewer( const shared_ptr<carve::mesh::MeshSet<3> >& meshset, const osg::Vec4f& color, const bool wireframe )
{
	if( func_call_on_render_meshset )
	{
		if( obj_call_on_render_meshset )
		{
			shared_ptr<carve::input::PolyhedronData> poly;
			func_call_on_render_meshset( obj_call_on_render_meshset, meshset, poly, color, wireframe );
		}
	}
}

void renderPolylineInDebugViewer( shared_ptr<carve::input::PolylineSetData >& poly_line, osg::Vec4f& color )
{
	if( func_call_on_render_polyline )
	{
		if( obj_call_on_render_meshset )
		{
			func_call_on_render_polyline( obj_call_on_render_meshset, poly_line, color );
		}
	}
}

#endif
