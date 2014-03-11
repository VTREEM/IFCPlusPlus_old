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

#ifdef _DEBUG

#include <QtCore/qglobal.h>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QLabel>

#include <osg/Group>
#include <osg/Vec4>

#include "carve/mesh.hpp"
#include "carve/input.hpp"
#include "ifcpp/model/shared_ptr.h"

class ViewerWidget;
class ViewController;

class DebugViewer : public QMainWindow
{
	Q_OBJECT

public:
	DebugViewer();
	~DebugViewer();

	static void renderMeshsetWrapper(void* obj_ptr, const shared_ptr<carve::mesh::MeshSet<3> >& meshset, const shared_ptr<carve::input::PolyhedronData>& poly, const osg::Vec4f& color, const bool wireframe);
	static void renderPolylineWrapper(void* obj_ptr, const shared_ptr<carve::input::PolylineSetData >& poly_line, const osg::Vec4f& color);
	
	void renderMeshset( const shared_ptr<carve::mesh::MeshSet<3> >& meshset, const shared_ptr<carve::input::PolyhedronData>& poly, const osg::Vec4f& color, const bool wireframe );
	void renderPolyline( const shared_ptr<carve::input::PolylineSetData >& poly_line, const osg::Vec4f& color );

	ViewerWidget*			m_viewer_widget;
	ViewController*			m_view_controller;
	bool m_cull_front;
	bool m_cull_back;

protected:
	void closeEvent( QCloseEvent *event );

public slots:
	void slotBtnZoomBoundingsClicked();
	void slotBtnWireframeClicked();
	void slotCullFrontFaces( int state );
	void slotCullBackFaces( int state );
};
#endif
