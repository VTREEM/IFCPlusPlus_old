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

#include <osgGA/CameraManipulator>
#include <osg/Quat>

class CameraMan3D : public osgGA::CameraManipulator
{

public:
	CameraMan3D( osg::Camera* cam );

	virtual const char* classname() const { return "CameraMan3D"; }

	void init();
	void setOrbitModeEnabled( bool orbit );
	bool isOrbitModeEnabled() {  return m_orbit_mode_enabled; }
	void setOverrideOrbitModeDisabled();
	void restoreOverrideOrbitMode();

	virtual void setByMatrix(const osg::Matrixd& matrix);
	virtual void setByInverseMatrix(const osg::Matrixd& matrix) { setByMatrix(osg::Matrixd::inverse(matrix)); }
	virtual osg::Matrixd getMatrix() const;
	virtual osg::Matrixd getInverseMatrix() const;

	virtual void setNode(osg::Node*);
	virtual const osg::Node* getNode() const;
	virtual osg::Node* getNode();
	
	virtual void home(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us);
	virtual void home(double);
	
	virtual void init(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter& us);
	virtual bool handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter& us);

	void setCenter(const osg::Vec3d& center) { m_center = center; }
	const osg::Vec3d& getCenter() const { return m_center; }
	void setRotation( const osg::Quat& rotation );
	void setYawPitch( const double yaw, const double pitch );
	void setZAxisDown( bool down );

	const osg::Quat& getRotation() const { return m_rotation; }

	void setDistance(double distance)
	{
		m_distance = distance;
	}

	double getDistance() const { return m_distance; }

	void zoom( double ds, double x, double y );
	void pan( double dx, double dy );

protected:
	virtual ~CameraMan3D();

	void flushMouseEventStack();
	void rememberMouseEvent(const osgGA::GUIEventAdapter& ea);
	void computePosition(const osg::Vec3& eye,const osg::Vec3& lv,const osg::Vec3& up);
	bool calcMovement( osgGA::GUIEventAdapter::EventType event_type );
	bool isMouseMoving();

	osg::ref_ptr<const osgGA::GUIEventAdapter> m_ga_t1;
	osg::ref_ptr<const osgGA::GUIEventAdapter> m_ga_t0;
	
	osg::ref_ptr<osg::Node>	m_node;

	double			m_distance;
	double			m_modelScale;
	double			m_minimumZoomScale;
	bool			m_thrown_rotate;
	bool			m_thrown_pan;
	bool			m_orbit_mode_enabled;
	bool			m_orbit_mode_override_disabled;
	osg::Camera*	m_cam;
	osg::Vec3d		m_center;
	osg::Vec3d		m_tb_center;
	osg::Quat		m_rotation;
	double			m_yaw;
	double			m_pitch;
	
	bool			m_z_axis_down;
	double			m_fading_move;
};
