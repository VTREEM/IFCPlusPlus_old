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

//! @author Fabian Gerold
//! @date 2011-07-18

#pragma once

#include <carve/matrix.hpp>
#include <carve/geom2d.hpp>
#include <carve/geom3d.hpp>
#include <carve/input.hpp>
#include <ifcpp/model/shared_ptr.h>

#include <osg/ref_ptr>
#include <osg/Array>
#include <osg/Geode>
#include <osg/Group>
#include <osgViewer/Viewer>

class UnitConverter;
class IfcPolyline;
class IfcCurve;
class IfcCartesianPoint;
class IfcTrimmingSelect;

class CurveConverter
{
public:
	CurveConverter( shared_ptr<UnitConverter> unit_converter, int num_vertices_per_circle );
	~CurveConverter();

	void convertIfcCurve( const shared_ptr<IfcCurve>& ifc_curve, std::vector<carve::geom::vector<3> >& loops, std::vector<carve::geom::vector<3> >& segment_start_points ) const;
	void convertIfcCurve( const shared_ptr<IfcCurve>& ifc_curve, std::vector<carve::geom::vector<3> >& loops, std::vector<carve::geom::vector<3> >& segment_start_points,
		std::vector<shared_ptr<IfcTrimmingSelect> >& trim1_vec, std::vector<shared_ptr<IfcTrimmingSelect> >& trim2_vec, bool sense_agreement ) const;
	void convertIfcPolyline( const shared_ptr<IfcPolyline>& poly_line,	std::vector<carve::geom::vector<3> >& loops ) const;

	void convertIfcCartesianPoint(			const shared_ptr<IfcCartesianPoint>& ifc_point,				carve::geom::vector<3>& point ) const;
	static void convertIfcCartesianPoint(	const shared_ptr<IfcCartesianPoint>& ifc_point,				carve::geom::vector<3>& point, double length_factor );
	void convertIfcCartesianPointVector(	const std::vector<shared_ptr<IfcCartesianPoint> >& points,	std::vector<carve::geom::vector<3> >& vertices ) const;
	void convertIfcCartesianPointVectorSkipDuplicates( const std::vector<shared_ptr<IfcCartesianPoint> >& ifc_points, std::vector<carve::geom::vector<3> >& loop );

	// osg
	//static void convertIfcCartesianPoint( const shared_ptr<IfcCartesianPoint>& ifc_point,	osg::Vec3d& point, double length_factor );
	//void convertIfcCartesianPointVector(    const std::vector<shared_ptr<IfcCartesianPoint> >& points,      osg::Vec3dArray* vertices );
	//static void convertIfcCartesianPointVector(    const std::vector<shared_ptr<IfcCartesianPoint> >& points, osg::Vec3dArray* vertices, double length_factor );
		
	static double getAngleOnCircle( const carve::geom::vector<3>& circle_center, double circle_radius, const carve::geom::vector<3>& trim_point );
	static double getAngleOnCircle( const osg::Vec3d& circle_center, double circle_radius, const osg::Vec3d& trim_point );
	
	shared_ptr<UnitConverter>		m_unit_converter;
	int								m_num_vertices_per_circle;
};
