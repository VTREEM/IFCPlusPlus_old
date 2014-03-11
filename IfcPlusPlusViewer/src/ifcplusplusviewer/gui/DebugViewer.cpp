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

#include <QtWidgets/QAction>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QCheckBox>
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
	setWindowTitle("IFC++ DebugViewer");
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

	QSettings settings(QSettings::UserScope, QLatin1String("IfcPlusPlus"));
	QStringList keys = settings.allKeys();
	
	m_cull_front = false;
	m_cull_back = false;
	if( keys.contains( "DebugViewerCullFrontFaces" ) )
	{
		m_cull_front = settings.value("DebugViewerCullFrontFaces").toBool();
	}
	if( keys.contains( "DebugViewerCullBackFaces" ) )
	{
		m_cull_back = settings.value("DebugViewerCullBackFaces").toBool();
	}
	GeomUtils::cullFrontBack( m_cull_front, m_cull_back, m_view_controller->getRootNode()->getOrCreateStateSet() );

	// cull face buttons
	QCheckBox* cull_front_faces = new QCheckBox( "Cull front faces" );
	if( m_cull_front )
	{
		cull_front_faces->setChecked( true );
	}
	connect( cull_front_faces, SIGNAL( stateChanged( int ) ), this, SLOT( slotCullFrontFaces( int ) ) );

	QCheckBox* cull_back_faces = new QCheckBox( "Cull back faces" );
	if( m_cull_back )
	{
		cull_back_faces->setChecked( true );
	}
	connect( cull_back_faces, SIGNAL( stateChanged( int ) ), this, SLOT( slotCullBackFaces( int ) ) );

	QToolBar * m_file_toolbar = new QToolBar();
	m_file_toolbar->setObjectName("FileToolbar");
	m_file_toolbar->addAction(zoom_bounds_btn);
	m_file_toolbar->addAction(wireframe);
	m_file_toolbar->addWidget(cull_front_faces);
	m_file_toolbar->addWidget(cull_back_faces);
	addToolBar( Qt::LeftToolBarArea, m_file_toolbar );

	// central widget
	setCentralWidget( m_viewer_widget );

	// restore geometry
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
		m_view_controller->setViewerMode( ViewController::VIEWER_MODE_WIREFRAME );
		//m_viewer_widget->setViewerMode( ViewerWidget::VIEWER_MODE_WIREFRAME );
	}
	else
	{
		m_view_controller->setViewerMode( ViewController::VIEWER_MODE_SHADED );
		//m_viewer_widget->setViewerMode( ViewerWidget::VIEWER_MODE_SHADED );
	}
}

void DebugViewer::renderMeshsetWrapper( void* ptr, const shared_ptr<carve::mesh::MeshSet<3> >& meshset, const shared_ptr<carve::input::PolyhedronData>& poly, const osg::Vec4f& color, const bool wireframe )
{
	DebugViewer* myself = (DebugViewer*)ptr;
	if( myself )
	{
		myself->renderMeshset( meshset, poly, color, wireframe );
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

void DebugViewer::renderMeshset( const shared_ptr<carve::mesh::MeshSet<3> >& meshset, const shared_ptr<carve::input::PolyhedronData>& poly, 
								const osg::Vec4f& color, const bool wireframe )
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

	ConverterOSG::drawMeshSet( meshset, geode );
	m_view_controller->getModelNode()->addChild(geode);

	osg::ref_ptr<osg::Geode> geode_vertex_numbers = new osg::Geode();
	m_view_controller->getRootNode()->addChild(geode_vertex_numbers);

	if( poly )
	{
		ConverterOSG::drawVertexNumbers( poly, color, geode_vertex_numbers );
	}
	else
	{
		shared_ptr<carve::poly::Polyhedron> poly_from_mesh( carve::polyhedronFromMesh(meshset.get(), -1) );
		shared_ptr<carve::input::PolyhedronData> poly_data( new carve::input::PolyhedronData() );
		for( int i=0; i<poly_from_mesh->vertices.size(); ++i )
		{
			poly_data->addVertex( poly_from_mesh->vertices[i].v );
		}
		for( int i=0; i<poly_from_mesh->faces.size(); ++i )
		{
			carve::poly::Face<3>& f = poly_from_mesh->faces[i];
			if( f.nVertices() == 3 )
			{
				poly_data->addFace( poly_from_mesh->vertexToIndex( f.vertex(0) ), poly_from_mesh->vertexToIndex( f.vertex(1) ), poly_from_mesh->vertexToIndex( f.vertex(2) ) );
			}
			else if( f.nVertices() == 4 )
			{
				poly_data->addFace( poly_from_mesh->vertexToIndex( f.vertex(0) ), poly_from_mesh->vertexToIndex( f.vertex(1) ), poly_from_mesh->vertexToIndex( f.vertex(2) ), poly_from_mesh->vertexToIndex( f.vertex(3) ) );
			}
		}

		ConverterOSG::drawVertexNumbers( poly_data, color, geode_vertex_numbers );
	}
	

	m_viewer_widget->getViewer().frame();
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

		ConverterOSG::drawPolyline( poly_line, geode );
		m_view_controller->getRootNode()->addChild(geode);
		m_viewer_widget->getViewer().frame();
	}
}


void DebugViewer::slotCullFrontFaces( int state )
{
	if( state == Qt::Checked )
	{
		m_cull_front = true;
	}
	else
	{
		m_cull_front = false;
	}
	QSettings settings(QSettings::UserScope, QLatin1String("IfcPlusPlus"));
	settings.setValue("DebugViewerCullFrontFaces", m_cull_front );

	GeomUtils::cullFrontBack( m_cull_front, m_cull_back, m_view_controller->getRootNode()->getOrCreateStateSet() );
}

void DebugViewer::slotCullBackFaces( int state )
{
	if( state == Qt::Checked )
	{
		m_cull_back = true;
	}
	else
	{
		m_cull_back = false;
	}
	QSettings settings(QSettings::UserScope, QLatin1String("IfcPlusPlus"));
	settings.setValue("DebugViewerCullBackFaces", m_cull_back );

	GeomUtils::cullFrontBack( m_cull_front, m_cull_back, m_view_controller->getRootNode()->getOrCreateStateSet() );
}

#endif
