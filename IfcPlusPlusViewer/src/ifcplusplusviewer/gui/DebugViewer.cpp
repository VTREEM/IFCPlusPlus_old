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

#ifdef _DEBUG

#include <osg/PolygonMode>
#include <osg/Material>

#include <QtCore/qglobal.h>

#include <QtWidgets/qaction.h>
#include <QtWidgets/qtoolbar.h>
#include <QtCore/QSettings>
#include <QtCore/QFile>

#include "ViewController.h"
#include "viewer/ViewerWidget.h"
#include "viewer/Orbit3DManipulator.h"
#include "ifcppgeometry/GeomUtils.h"
#include "ifcppgeometry/DebugViewerCallback.h"
#include "ifcppgeometry/ConverterOSG.h"

#include "DebugViewer.h"

DebugViewer::DebugViewer() : QMainWindow()
{
	setWindowTitle("IfcPlusPlus DebugViewer");
	setWindowIcon( QIcon( ":img/IfcPlusPlusViewerWindowIcon.png" ) );
	
	// global style sheet definitions
	QFile file( ":styles.css" );
	file.open( QFile::ReadOnly );
	QString styleSheet = QLatin1String( file.readAll() );
	setStyleSheet( styleSheet );

	// viewer
	m_view_controller = new ViewController();
	m_viewer_widget = new ViewerWidget();
	m_viewer_widget->setParent( this );
	Orbit3DManipulator* camera_manip = new Orbit3DManipulator( NULL );
	m_viewer_widget->getMainView()->setCameraManipulator( camera_manip );
	m_viewer_widget->setRootNode( m_view_controller->getRootNode() );


	setRenderMeshsetCallBack( this, &DebugViewer::renderMeshsetWrapper );
	setRenderPolylineCallBack( this, &DebugViewer::renderPolylineWrapper );

	// gui
	QAction* zoom_bounds_btn = new QAction(QIcon(":img/zoomBoundings.png"), "&Zoom to boundings", this );
	zoom_bounds_btn->setShortcut(tr("Ctrl+Z"));
	zoom_bounds_btn->setStatusTip("Zoom to boundings");
	connect(zoom_bounds_btn, SIGNAL(triggered()), this, SLOT(slotBtnZoomBoundingsClicked()));

	QAction* wireframe = new QAction(QIcon(":img/TabViewWireframe.png"), "&Wireframe [w]", this );
	wireframe->setCheckable( true );
	//wireframe->setShortcut(tr("w"));
	wireframe->setStatusTip("Wireframe [w]");
	connect(wireframe, SIGNAL(triggered()), this, SLOT(slotBtnWireframeClicked()));

	QToolBar * m_file_toolbar = new QToolBar();
	m_file_toolbar->setObjectName("FileToolbar");
	m_file_toolbar->addAction(zoom_bounds_btn);
	m_file_toolbar->addAction(wireframe);
	addToolBar( Qt::LeftToolBarArea, m_file_toolbar );

	// central widget
	setCentralWidget( m_viewer_widget );

	// restore geometry
	QSettings settings(QSettings::UserScope, QLatin1String("IfcPlusPlus"));
	QStringList keys = settings.allKeys();
	if( keys.contains( "DebugViewerGeometry" ) )
	{
		restoreGeometry(settings.value("DebugViewerGeometry").toByteArray());
	}
	else
	{
		showMaximized();
	}
	if( keys.contains( "DebugViewerState" ) )
	{
		restoreState(settings.value("DebugViewerState").toByteArray());
	}
}

DebugViewer::~DebugViewer()
{

}

void DebugViewer::closeEvent( QCloseEvent *event )
{
	QSettings settings(QSettings::UserScope, QLatin1String("IfcPlusPlus"));
	settings.setValue("DebugViewerGeometry", saveGeometry());
	settings.setValue("DebugViewerState", saveState());
	QMainWindow::closeEvent( event );
}


void DebugViewer::slotBtnZoomBoundingsClicked()
{
	osg::BoundingSphere bs = m_view_controller->getRootNode()->computeBound();
	
	osgViewer::View* main_view = m_viewer_widget->getMainView();
	if( main_view )
	{
		osgGA::CameraManipulator* camera_manip = main_view->getCameraManipulator();
		Orbit3DManipulator* orbit_manip = dynamic_cast<Orbit3DManipulator*>( camera_manip );
		if( orbit_manip )
		{
			orbit_manip->zoomToBoundingSphere( bs );
		}
	}
}

void DebugViewer::slotBtnWireframeClicked()
{
	QAction* toggle_btn = (QAction*)sender();

	if( toggle_btn->isChecked() )
	{
		//m_system->getViewController()->setViewerMode( ViewController::VIEWER_MODE_WIREFRAME );
		//m_viewer_widget->setViewerMode( ViewerWidget::VIEWER_MODE_WIREFRAME );
	}
	else
	{
		//m_system->getViewController()->setViewerMode( ViewController::VIEWER_MODE_SHADED );
		//m_viewer_widget->setViewerMode( ViewerWidget::VIEWER_MODE_SHADED );
	}
}

void DebugViewer::renderMeshsetWrapper( void* ptr, const shared_ptr<carve::mesh::MeshSet<3> >& meshset, const osg::Vec4f& color, const bool wireframe )
{
	DebugViewer* myself = (DebugViewer*)ptr;
	if( myself )
	{
		myself->renderMeshset( meshset, color, wireframe );
	}
}

void DebugViewer::renderPolylineWrapper(void* ptr, const shared_ptr<carve::input::PolylineSetData >& poly_line, const osg::Vec4f& color)
{
	DebugViewer* myself = (DebugViewer*)ptr;
	if( myself )
	{
		myself->renderPolyline( poly_line, color );
	}
}

void DebugViewer::renderMeshset( const shared_ptr<carve::mesh::MeshSet<3> >& meshset, const osg::Vec4f& color, const bool wireframe )
{
	if( m_view_controller->getRootNode() )
	{
		osg::ref_ptr<osg::Geode> geode = new osg::Geode();
		if( wireframe )
		{
			osg::ref_ptr<osg::PolygonMode> polygon_mode = new osg::PolygonMode();
			polygon_mode->setMode(  osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE );
			geode->getOrCreateStateSet()->setAttribute( polygon_mode );
		}
		osg::Material* material = new osg::Material();//(osg::Material *) geode->getStateSet()->getAttribute(osg::StateAttribute::MATERIAL); 
		material->setColorMode(osg::Material::EMISSION); 
		material->setEmission(osg::Material::FRONT_AND_BACK, color ); 
		geode->getOrCreateStateSet()->setAttributeAndModes(material, osg::StateAttribute::OVERRIDE); 

		ConverterOSG carve_converter;
		carve_converter.drawMeshSet( meshset, geode );
		m_view_controller->getRootNode()->addChild(geode);

		m_viewer_widget->getViewer().frame();
	}
}

void DebugViewer::renderPolyline( const shared_ptr<carve::input::PolylineSetData >& poly_line, const osg::Vec4f& color )
{
	if( m_view_controller->getRootNode() )
	{
		osg::ref_ptr<osg::Geode> geode = new osg::Geode();
			
		osg::Material* material = new osg::Material();//(osg::Material *) geode->getStateSet()->getAttribute(osg::StateAttribute::MATERIAL); 
		material->setColorMode(osg::Material::EMISSION); 
		material->setEmission(osg::Material::FRONT_AND_BACK, color ); 
		geode->getOrCreateStateSet()->setAttributeAndModes(material, osg::StateAttribute::OVERRIDE); 

		ConverterOSG carve_converter;
		carve_converter.drawPolyline( poly_line, geode );
		m_view_controller->getRootNode()->addChild(geode);
		m_viewer_widget->getViewer().frame();
	}
}
#endif
