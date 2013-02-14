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

#include <iostream>
#include <osg/Quat>
#include <osg/Notify>
#include <osg/BoundsChecking>
#include <osg/BoundingSphere>

#include "CameraMan3D.h"

CameraMan3D::CameraMan3D( osg::Camera*  cam )
{
	m_cam = cam;
	m_modelScale = 0.01f;
	m_minimumZoomScale = 0.05f;
	m_thrown_pan = false;
	m_thrown_rotate = false;
	m_orbit_mode_enabled = true;
	m_orbit_mode_override_disabled = false;
	m_distance = 130.0f;
	m_fading_move = 1.0;
	
	m_z_axis_down = false;
	init();

	setYawPitch( m_yaw, m_pitch );
	setAutoComputeHomePosition( false );
}

CameraMan3D::~CameraMan3D()
{
}

void CameraMan3D::init()
{
	if( m_z_axis_down )
	{
		m_yaw =   osg::inDegrees( 45.f ); // rotation about z-axis
		m_pitch = osg::inDegrees( 250.f );      // rotation about y-axis
		_homeEye.set( 1.0, 1.0, -1.41*tan( osg::inDegrees( 20.f ) ) );
		_homeUp.set( 0.0, 0.0, -1.0 );
	}
	else
	{
		m_yaw =   osg::inDegrees( 135.f ); // rotation about z-axis
		m_pitch = osg::inDegrees( 70.f );       // rotation about y-axis, angle from positive z axis
		_homeEye.set( 1.0, 1.0, 1.41*tan( osg::inDegrees( 20.f ) ) );
	}
}

void CameraMan3D::setOrbitModeEnabled( bool orbit )
{
	m_orbit_mode_enabled = orbit;
}

void CameraMan3D::setOverrideOrbitModeDisabled()
{
	m_orbit_mode_override_disabled = true;
}
void CameraMan3D::restoreOverrideOrbitMode()
{
	m_orbit_mode_override_disabled = false;
}

void CameraMan3D::setNode(osg::Node* node)
{
	m_node = node;
	if( m_node.get() )
	{
		const osg::BoundingSphere& boundingSphere=m_node->getBound();
		m_modelScale = boundingSphere._radius;
	}
	if( getAutoComputeHomePosition() )
	{
		computeHomePosition();
	}
}


const osg::Node* CameraMan3D::getNode() const
{
	return m_node.get();
}


osg::Node* CameraMan3D::getNode()
{
	return m_node.get();
}


void CameraMan3D::home(double /*currentTime*/)
{
	if( getAutoComputeHomePosition() )
	{
		computeHomePosition();
	}
	computePosition(_homeEye, _homeCenter, _homeUp);
	m_thrown_pan = false;
	m_thrown_rotate = false;
}

void CameraMan3D::home(const osgGA::GUIEventAdapter& ea ,osgGA::GUIActionAdapter& aa )
{
	home(ea.getTime());
	aa.requestRedraw();
	aa.requestContinuousUpdate(false);
}


void CameraMan3D::init(const osgGA::GUIEventAdapter& ,osgGA::GUIActionAdapter& aa )
{
	flushMouseEventStack();
	aa.requestRedraw();
}

bool CameraMan3D::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	if( ea.getHandled() )
	{
		return false;
	}

	const osgGA::GUIEventAdapter::EventType event_type = ea.getEventType();
	switch( event_type )
	{
		case(osgGA::GUIEventAdapter::PUSH):
		{
			m_thrown_rotate = false;
			m_thrown_pan = false;
			m_fading_move = 1.0;

			flushMouseEventStack();
			rememberMouseEvent(ea);
			if( calcMovement( event_type ))
			{
				aa.requestRedraw();
			}
			aa.requestContinuousUpdate(false);

			return true;
		}

		case(osgGA::GUIEventAdapter::RELEASE):
		{
			m_thrown_rotate = false;
			m_thrown_pan = false;
			m_fading_move = 1.0;

			if( ea.getButtonMask()==0 )
			{
				double timeSinceLastRecordEvent = m_ga_t0.valid() ? (ea.getTime() - m_ga_t0->getTime()) : DBL_MAX;
				if( timeSinceLastRecordEvent>0.02) flushMouseEventStack();

				if( isMouseMoving())
				{
					if( calcMovement( event_type ) )
					{
						aa.requestRedraw();
						aa.requestContinuousUpdate(true);
					}
				}
				else
				{
					flushMouseEventStack();
					rememberMouseEvent(ea);
					if( calcMovement( event_type ) )
					{
						aa.requestRedraw();
					}
					aa.requestContinuousUpdate(false);
				}
			}
			else
			{
				flushMouseEventStack();
				rememberMouseEvent(ea);
				if( calcMovement( event_type ) )
				{
					aa.requestRedraw();
				}
				aa.requestContinuousUpdate(false);
			}
			return true;
		}

		case(osgGA::GUIEventAdapter::DRAG):
		{
			m_thrown_rotate = false;
			m_thrown_pan = false;
			m_fading_move = 1.0;

			rememberMouseEvent(ea);
			if( calcMovement( event_type ) )
			{
				aa.requestRedraw();
			}
			aa.requestContinuousUpdate(false);
			
			return true;
		}

		case(osgGA::GUIEventAdapter::SCROLL):
		{
			if( ea.getScrollingMotion() == osgGA::GUIEventAdapter::SCROLL_DOWN )
			{
				zoom( 0.2, ea.getXnormalized(), ea.getYnormalized() );
			}
			else if( ea.getScrollingMotion() == osgGA::GUIEventAdapter::SCROLL_UP )
			{
				zoom( -0.2, ea.getXnormalized(), ea.getYnormalized() );
			}
				
			return false;
		}
		case(osgGA::GUIEventAdapter::MOVE):
		{
			return false;
		}

		case(osgGA::GUIEventAdapter::KEYDOWN):
			if( ea.getKey() == osgGA::GUIEventAdapter::KEY_Escape )
			{
				return true;
			}
			return false;
			
		case( osgGA::GUIEventAdapter::KEYUP ):
			if( ea.getKey() == osgGA::GUIEventAdapter::KEY_Escape )
			{
				return true;
			}
			return false;
		
		case( osgGA::GUIEventAdapter::FRAME ):
			if( m_thrown_rotate || m_thrown_pan )
			{
				if( calcMovement( event_type ) )
				{
					aa.requestRedraw();
				}
			}
			return false;
		default:
			return false;
	}
	return false;
}


bool CameraMan3D::isMouseMoving()
{
	if( m_ga_t0.get()==NULL || m_ga_t1.get()==NULL )
	{
		return false;
	}

	static const double velocity = 0.1f;

	double dx = m_ga_t0->getXnormalized()-m_ga_t1->getXnormalized();
	double dy = m_ga_t0->getYnormalized()-m_ga_t1->getYnormalized();
	double len = sqrtf(dx*dx+dy*dy);
	double dt = m_ga_t0->getTime()-m_ga_t1->getTime();

	return (len>dt*velocity);
}


void CameraMan3D::flushMouseEventStack()
{
	m_ga_t1 = NULL;
	m_ga_t0 = NULL;
}


void CameraMan3D::rememberMouseEvent( const osgGA::GUIEventAdapter& ea )
{
	m_ga_t1 = m_ga_t0;
	m_ga_t0 = &ea;
}

void CameraMan3D::setByMatrix( const osg::Matrixd& matrix )
{
	m_center = osg::Vec3(0.0f,0.0f,-m_distance)*matrix;
	m_rotation = matrix.getRotate();
}

osg::Matrixd CameraMan3D::getMatrix() const
{
	return osg::Matrixd::translate( 0.0,0.0,m_distance )*osg::Matrixd::rotate( m_rotation )*osg::Matrixd::translate( m_center );
}

osg::Matrixd CameraMan3D::getInverseMatrix() const
{
	return osg::Matrixd::translate( -m_center )*osg::Matrixd::rotate( m_rotation.inverse() )*osg::Matrixd::translate( 0.0,0.0,-m_distance );
}

void CameraMan3D::computePosition( const osg::Vec3& eye,const osg::Vec3& center,const osg::Vec3& up )
{
	osg::Vec3 lv( center - eye );

	osg::Vec3 f( lv );
	f.normalize();
	osg::Vec3 s( f^up );
	s.normalize();
	osg::Vec3 u( s^f );
	u.normalize();
	
	osg::Matrix rotation_matrix(s[0],	u[0],   -f[0],  0.0f,
								s[1],    u[1],   -f[1],  0.0f,
								s[2],    u[2],   -f[2],  0.0f,
								0.0f,    0.0f,   0.0f,    1.0f);

	m_center = center;
	//m_distance = lv.length();
	m_rotation = rotation_matrix.getRotate().inverse();
}

bool CameraMan3D::calcMovement( osgGA::GUIEventAdapter::EventType event_type )
{
	// return if less then two events have been added.
	if( m_ga_t0.get()==NULL || m_ga_t1.get()==NULL )
	{
		return false;
	}

	const double dx = ( m_ga_t0->getXnormalized()-m_ga_t1->getXnormalized() )*m_fading_move;
	const double dy = ( m_ga_t0->getYnormalized()-m_ga_t1->getYnormalized() )*m_fading_move;
	if( event_type == osgGA::GUIEventAdapter::FRAME )
	{
		if( m_thrown_rotate )
		{
			m_fading_move *= 0.99;
		}
		else if( m_thrown_pan )
		{
			m_fading_move *= 0.96;
		}
	}

	double distance = sqrtf(dx*dx + dy*dy);

	// return if movement is too fast, indicating an error in event values or change in screen.
	if( distance>0.5 )
	{
		return false;
	}
	else if( distance<0.00005 )
	{
		// we dont need to requestRedraw when movement is too small
		return false;
	}

	// return if there is no movement.
	if( distance==0.0f)
	{
		return false;
	}

	unsigned int buttonMask = m_ga_t1->getButtonMask();
		
	if( buttonMask==osgGA::GUIEventAdapter::MIDDLE_MOUSE_BUTTON || buttonMask==(osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON|osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON))
	{
		// pan model
		osg::Matrix m1;
		m1.makeTranslate( dx, dy, 0 );

		osg::Matrix cm = m_cam->getProjectionMatrix();
		m_cam->setProjectionMatrix( cm*m1 );

		// throw
		if( event_type == osgGA::GUIEventAdapter::RELEASE )
		{
			m_thrown_pan = true;
		}
		return true;
	}
	else if( buttonMask == osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON )
	{
		if( !m_orbit_mode_enabled )
		{
			return false;
		}
		if( m_orbit_mode_override_disabled )
		{
			return false;
		}
		
		m_yaw += dx*-5.0;
		m_pitch += dy*2.5;

		setYawPitch( m_yaw, m_pitch );

		if( event_type == osgGA::GUIEventAdapter::RELEASE )
		{
			m_thrown_rotate = true;
		}
		
		return true;
	}
	else if( buttonMask==osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON )
	{
		// zoom model.
		zoom( -dy, m_ga_t1->getXnormalized(), 0.5 );
		return true;
	}
	return false;
}

void CameraMan3D::setZAxisDown( bool down )
{
	m_z_axis_down = down;
	init();
	setYawPitch( m_yaw, m_pitch );
}

void CameraMan3D::setRotation( const osg::Quat& rotation )
{
	m_rotation = rotation;
}

void CameraMan3D::setYawPitch( const double yaw, const double pitch )
{
	m_yaw = yaw;
	m_pitch = pitch;
	double pitch_clamp[2] = { 0.0, osg::inDegrees( 180.f ) };
	osg::Vec3d camera_up( 0, 0, -1.0 );
	if( m_z_axis_down )
	{
		camera_up.set( 0.0, 0.0, 1.0 );
		pitch_clamp[0] = osg::inDegrees( 180.f );
		pitch_clamp[1] = osg::inDegrees( 360.f );
	}

	if( m_pitch < pitch_clamp[0] )
	{
		m_pitch = pitch_clamp[0];
	}
	if( m_pitch > pitch_clamp[1] )
	{
		m_pitch = pitch_clamp[1];
	}
		
	m_rotation = osg::Quat( m_yaw, camera_up, m_pitch, osg::Vec3d( -1, 0, 0 ), 0.0, osg::Vec3d( 0, 1, 0 ) ).inverse();
}

void CameraMan3D::zoom( double ds, double x, double y )
{
	// scale
	osg::Matrix scale;
	scale.makeScale( 1-ds, 1-ds, 1 );
	
	// move
	osg::Matrix move;       
	move.makeTranslate( x*(ds), y*(ds), 0 );

	// apply both
	osg::Matrix previous = m_cam->getProjectionMatrix();
	m_cam->setProjectionMatrix( previous*scale*move );
}

void CameraMan3D::pan( double dx, double dy )
{
	// pan model
	osg::Matrix previous = m_cam->getProjectionMatrix();
	m_cam->setProjectionMatrix( previous*osg::Matrix::translate(dx, dy, 0) );
}
