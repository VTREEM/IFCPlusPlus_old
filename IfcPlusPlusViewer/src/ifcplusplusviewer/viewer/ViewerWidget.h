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

#include <osgViewer/Viewer>
namespace osgQt{ class  GraphicsWindowQt; }


#include <QtCore/qglobal.h>
#if (QT_VERSION < QT_VERSION_CHECK(5, 0, 0))
	#include <QtGui/qwidget.h>
	#include <QtCore/QTimer>
#else
	#include <QtWidgets/qwidget.h>
	#include <QtCore/QTimer>
#endif

class CameraMan3D;

//! @brief class to combine OSG Viewer and Qt widget
//! @author Fabian Gerold, Bauhaus University Weimar
//! @date 2011-07-02

class ViewerWidget : public QWidget, public osgViewer::Viewer
{
	Q_OBJECT

public:
	enum ViewerProjection
	{
		PROJECTION_PARALLEL,
		PROJECTION_PERSPECTIVE
	};
	enum ViewerMode
	{
		VIEWER_MODE_SHADED,
		VIEWER_MODE_WIREFRAME,
		VIEWER_MODE_HIDDEN_LINE
	};
	ViewerWidget( QWidget * parent = 0 );
	~ViewerWidget();

	QSize minimumSizeHint() const;
	QSize sizeHint() const;

	osgQt::GraphicsWindowQt* getGraphicsWindow() const { return m_gw; }
	void setRootNode( osg::Group* node );
	void setModelNode( osg::Group* node );
	void setHudNode( osg::Group* node );
	void setProjection( ViewerProjection p );
	void zoomToBoundingSphere( const osg::BoundingSphere& bs );
	void setViewerMode( ViewerMode mode );

	
	void getFocus() { setFocus(); }
	void addViewerWidget( QWidget* widget, int x, int y, int w, int h );
	osg::Camera* getCamera() { return _camera; }
	CameraMan3D* getCameraManager();

	void stopTimer();
	void startTimer();
	
signals:
	void signalViewerWidgetResized( int w, int h );
	void signalKeyPressEvent( QKeyEvent* e );
	void signalKeyReleaseEvent( QKeyEvent* e );

protected:
	QTimer					  m_timer;
	CameraMan3D*			    m_camera_manager;
	osgQt::GraphicsWindowQt*	m_gw;
	osg::ref_ptr<osg::Group>	m_rootnode;
	osg::ref_ptr<osg::Group>	m_model_node;
	osg::ref_ptr<osg::Camera>       m_hud_camera;
	ViewerProjection			m_projection;
	double					  m_near_plane;
	double					  m_far_plane;
	ViewerMode				      m_viewer_mode;

	void init();
	virtual void resizeGL( int width, int height );
	virtual void mousePressEvent( QMouseEvent* e );
	virtual void mouseReleaseEvent( QMouseEvent* e );
	virtual void mouseMoveEvent( QMouseEvent* e );
	virtual void wheelEvent( QWheelEvent* e );
	virtual void keyPressEvent( QKeyEvent* e );
	virtual void keyReleaseEvent( QKeyEvent* e );
	virtual void mouseDoubleClickEvent ( QMouseEvent * e );
	virtual void paintEvent( QPaintEvent* e );
};