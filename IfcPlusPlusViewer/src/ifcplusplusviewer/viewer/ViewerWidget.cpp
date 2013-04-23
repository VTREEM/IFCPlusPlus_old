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


//#include <QtOpenGL/QGLWidget>

#include <stdlib.h>
#include <osgGA/GUIEventAdapter>
#include <osgGA/OrbitManipulator>

#include <osgViewer/ViewerEventHandlers>
#include <osgViewer/Renderer>
#include <osg/AnimationPath>
#include <osg/LightModel>

#include <QtCore/qglobal.h>
#if (QT_VERSION < QT_VERSION_CHECK(5, 0, 0))
	#include <QtGui/qboxlayout.h>
	#include <QtGui/qlabel.h>
	#include <QtGui/qevent.h>
#else
	#include <QtWidgets/qboxlayout.h>
	#include <QtWidgets/qlabel.h>
	#include <QtGui/QKeyEvent>
#endif


// D:\lib\Qt\5.0.2\qtbase\src\gui\opengl\qopengl.h(71)://typedef GLfloat GLdouble;

#include <osgQt/GraphicsWindowQt>

#include "ifcppgeometry/Utility.h"
#include "CameraMan3D.h"
#include "ViewerWidget.h"

//! @author Fabian Gerold, Bauhaus University Weimar
//! @date 2011-07-02

ViewerWidget::ViewerWidget( QWidget* parent ) : QWidget(parent)
{
	setMouseTracking( true );
	//setThreadingModel( osgViewer::Viewer::SingleThreaded );

	m_near_plane = 0.1;
	m_far_plane = 5000;
	int x = 0;
	int y = 0;
	int w = width();
	int h = height();

	osg::DisplaySettings* ds = osg::DisplaySettings::instance().get();
	osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
	traits->windowName = "";
	traits->windowDecoration = false;
	traits->x = x;
	traits->y = y;
	traits->width = w;
	traits->height = h;
	traits->doubleBuffer = true;
	traits->alpha = ds->getMinimumNumAlphaBits();
	traits->stencil = ds->getMinimumNumStencilBits();
	traits->sampleBuffers = ds->getMultiSamples();
	traits->samples = ds->getNumMultiSamples();

	//( osg::GraphicsContext::Traits* traits, QWidget* parent, 
	// osg::GraphicsContext::Traits* traits, QWidget* parent = NULL, const QGLWidget* shareWidget = NULL, Qt::WindowFlags f = 0 );
	QGLWidget* shareWidget = NULL;
	Qt::WindowFlags flags = 0;
	m_gw = new osgQt::GraphicsWindowQt(traits.get(), parent, shareWidget, flags );
	m_gw->getGLWidget()->setForwardKeyEvents( true );

	//osg::Light* light = getLight();
	//light->setAmbient(osg::Vec4(0.3f,0.3f,0.3f,1.0f));

	_camera->setGraphicsContext( m_gw );
	_camera->setClearColor( osg::Vec4(0.96f,	0.96f,  0.96f,  1.0f) );
	_camera->setCullingMode( osg::CullSettings::NO_CULLING );

	m_camera_manager = new CameraMan3D( _camera );
	setCameraManipulator( m_camera_manager );

	// headup display for mouse snapping symbols etc.
	bool setup_hud = false;
	m_hud_camera = NULL;
	if( setup_hud )
	{
		m_hud_camera = new osg::Camera();
		m_hud_camera->setName( "HUD_Camera" );
		m_hud_camera->setReferenceFrame( osg::Transform::ABSOLUTE_RF );
		m_hud_camera->setViewport(0,0,50,100);
		m_hud_camera->setProjectionMatrixAsOrtho2D( 0, w, 0, h );

		m_hud_camera->setViewMatrix( osg::Matrix::identity() );
		m_hud_camera->setRenderOrder( osg::Camera::POST_RENDER );
		m_hud_camera->setClearMask( GL_DEPTH_BUFFER_BIT );
	}

	m_viewer_mode = VIEWER_MODE_SHADED;
	setProjection( PROJECTION_PARALLEL );
	setContentsMargins(0, 0, 0, 0);
	QVBoxLayout* vbox = new QVBoxLayout();
	vbox->setContentsMargins(1, 1, 1, 1);
	vbox->addWidget( m_gw->getGLWidget() );
	setLayout( vbox );

	connect(&m_timer, SIGNAL(timeout()), this, SLOT(update()));
}
ViewerWidget::~ViewerWidget()
{
	m_timer.stop();
}

void ViewerWidget::stopTimer()
{
	m_timer.stop();
}
void ViewerWidget::startTimer()
{
	m_timer.start(10);
}
void ViewerWidget::paintEvent( QPaintEvent* )
{
	frame();
}

CameraMan3D* ViewerWidget::getCameraManager()
{
	return m_camera_manager;
}

void ViewerWidget::setRootNode( osg::Group* root )
{
	setSceneData( root );
	if( m_hud_camera )
	{
		root->addChild( m_hud_camera );
	}
	m_rootnode = root;
}

void ViewerWidget::setModelNode( osg::Group* root )
{
	m_model_node = root;
}

void ViewerWidget::setHudNode( osg::Group* node )
{
	if( m_hud_camera )
	{
		m_hud_camera->addChild( node );
	}
}

void ViewerWidget::setViewerMode( ViewerMode mode )
{
	if( mode != m_viewer_mode )
	{
		if( m_viewer_mode == VIEWER_MODE_WIREFRAME )
		{
			WireFrameModeOff( m_model_node.get() );
		}
		else if( m_viewer_mode == VIEWER_MODE_HIDDEN_LINE )
		{
			HiddenLineModeOff( m_model_node.get() );
		}

		m_viewer_mode = mode;
		if( m_viewer_mode == VIEWER_MODE_WIREFRAME )
		{
			WireFrameModeOn( m_model_node.get() );
		}
		else if( m_viewer_mode == VIEWER_MODE_HIDDEN_LINE )
		{
			HiddenLineModeOn( m_model_node.get() );
		}
	}
}

void ViewerWidget::setProjection( ViewerProjection p )
{
	m_projection = p;
	int w = width();
	int h = height();
	double ratio = double(h)/double(w);

	_camera->setViewport( new osg::Viewport(0, 0, w, h) );
	
	if( m_projection == PROJECTION_PARALLEL )
	{
		// for some reason, osg auto computed near/far planes cause unintended clipping...
		_camera->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);
		_camera->setProjectionMatrixAsOrtho(-100, 100, -100.0*ratio, 100.0*ratio, m_near_plane-1000, m_near_plane/0.0005+100 );
	}
	else if( m_projection == PROJECTION_PERSPECTIVE )
	{
		double near_plane = 0.1;
		_camera->setComputeNearFarMode(osg::CullSettings::COMPUTE_NEAR_FAR_USING_BOUNDING_VOLUMES);
		_camera->setProjectionMatrixAsPerspective( 90.0, 1.0/ratio, near_plane, m_far_plane );
	}
	if( m_hud_camera )
	{
		m_hud_camera->setProjectionMatrixAsOrtho2D(0, w, 0, h);
	}
}

QSize ViewerWidget::minimumSizeHint() const
{
    return QSize( 100, 100 );
}

QSize ViewerWidget::sizeHint() const
{
	return QSize( 800, 600 );
}

void ViewerWidget::resizeGL( int width, int height )
{
	m_gw->getEventQueue()->windowResize(0, 0, width, height );
	m_gw->resized(0,0,width,height);
	setProjection( m_projection );
	emit( signalViewerWidgetResized( width, height ) );
}

// keyboard events
void ViewerWidget::keyPressEvent( QKeyEvent* ke )
{
	if( ke->text().compare("w") == 0 )
	{
		if( m_viewer_mode == VIEWER_MODE_WIREFRAME )
		{
			setViewerMode( VIEWER_MODE_SHADED );
		}
		else
		{
			setViewerMode( VIEWER_MODE_WIREFRAME );
		}
	}
	else if( ke->text().compare("h") == 0 )
	{
		if( m_viewer_mode == VIEWER_MODE_HIDDEN_LINE )
		{
			setViewerMode( VIEWER_MODE_SHADED );
		}
		else
		{
			setViewerMode( VIEWER_MODE_HIDDEN_LINE );
		}
	}
}

void ViewerWidget::keyReleaseEvent( QKeyEvent*  )
{
}

void ViewerWidget::mousePressEvent( QMouseEvent*  )
{
}

void ViewerWidget::mouseReleaseEvent( QMouseEvent*  )
{
}

void ViewerWidget::mouseDoubleClickEvent( QMouseEvent *  )
{
}

void ViewerWidget::mouseMoveEvent( QMouseEvent* )
{
}

void ViewerWidget::wheelEvent( QWheelEvent*  )
{
}

// add a widget to be displayed in the ViewerWidget area
// ViewerWidget must be parent of the widget 
void ViewerWidget::addViewerWidget( QWidget* widget, int x, int y, int w, int h )
{
	//m_widget_set.insert( widget );
	widget->setParent( this );
	widget->setGeometry( x, y, w, h );
	widget->show();
}

void ViewerWidget::zoomToBoundingSphere( const osg::BoundingSphere& bs )
{
	frame();
	int w = width();
	int h = height();

	osg::Matrixd matrix_model_screen;
	matrix_model_screen.postMult(_camera->getViewMatrix());
	matrix_model_screen.postMult(_camera->getProjectionMatrix());
	if( _camera->getViewport() )
	{
		matrix_model_screen.postMult(_camera->getViewport()->computeWindowMatrix());
	}

	osg::Matrix matrix_screen_model;
	matrix_screen_model = matrix_model_screen.inverse(matrix_model_screen);

	// move
	{
		osg::Vec3d bs_center( bs.center() );
		// project point p1 to screen coordinates
		bs_center = matrix_model_screen.preMult(bs_center);

		double dx = w*0.5 - bs_center.x();
		double dy = h*0.5 - bs_center.y();
		double dx_normalized = dx/double(w)*2.0;
		double dy_normalized = dy/double(h)*2.0;
		m_camera_manager->pan( dx_normalized, dy_normalized );

		//osg::AnimationPath* path = new osg::AnimationPath();
		//osg::Vec3d position;
		//osg::Quat rotation;
		//int num_steps = 10;
		//double time=0.0;
		//double time_delta = 5.0/(double)num_steps;
		//for(int i=0;i<num_steps;++i)
		//{
		//      path->insert(time,osg::AnimationPath::ControlPoint(position,rotation));
		//      position._v[0] += dx_normalized/(double)num_steps;
		//      position._v[1] += dy_normalized/(double)num_steps;
		//      time += time_delta;
		//}
		//osg::AnimationPathCallback* apc = new osg::AnimationPathCallback( path );
		//m_camera->setUpdateCallback(apc);
	}

	// scale
	{
		osg::Vec3f p1( 0.0, 0.0, 0.0);
		osg::Vec3d p2;
		if( h > w )
		{
			p2.set( w*0.5f, 0.0, 0.0);
		}
		else
		{
			p2.set( h*0.5f, 0.0, 0.0);
		}

		p1 = matrix_screen_model.preMult(p1);
		p2 = matrix_screen_model.preMult(p2);

		double d_model = (p2 - p1).length();
		double scale_factor = 1.0;
		double bs_radius = bs.radius();
		if( bs_radius > 0.0 )
		{
			scale_factor = d_model/bs_radius;
		
			//scale_factor *= 0.85f;
			if( m_projection == PROJECTION_PARALLEL )
			{
				osg::Matrix scale;
				scale.makeScale( scale_factor, scale_factor, 1 );
				osg::Matrix previous = _camera->getProjectionMatrix();
				_camera->setProjectionMatrix( previous*scale );
			}

			if( bs_radius > 0.1*m_far_plane )
			{
				m_far_plane = bs_radius*10.0;
				setProjection( m_projection );
			}
		}
	}
}
