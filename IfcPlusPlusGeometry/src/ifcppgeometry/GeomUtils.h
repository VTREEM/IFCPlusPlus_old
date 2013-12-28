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

#include <vector>
#include <osg/Array>
#include <osg/Geode>
#include <osg/Group>
#include <osgViewer/Viewer>
#include <ifcpp/model/shared_ptr.h>

namespace carve
{
	namespace geom		{ 	template<unsigned ndim> struct vector;	}
	namespace mesh		{	template<unsigned ndim> class MeshSet;	}
	namespace math		{	struct Quaternion;	struct Matrix;	}
	namespace input		{	struct PolyhedronData;	struct PolylineSetData;	}
	namespace poly		{	class Polyhedron;	}
}

enum ProjectionPlane
{
	UNDEFINED,
	XY_PLANE,
	YZ_PLANE,
	XZ_PLANE
};

osg::ref_ptr<osg::Geode> createCoordinateAxes();
osg::ref_ptr<osg::Group> createCoordinateAxesArrows();
osg::ref_ptr<osg::Geode> createCoordinateGrid();
osg::ref_ptr<osg::Geode> createQuarterCircles();
void WireFrameModeOn( osg::StateSet* state );
void WireFrameModeOn( osg::Node* srisdNode );
void WireFrameModeOn( osg::Drawable* drawable );
void WireFrameModeOff( osg::StateSet* state );
void WireFrameModeOff( osg::Node* srisdNode );
void WireFrameModeOff( osg::Drawable* drawable );
void HiddenLineModeOn( osg::Group* node );
void HiddenLineModeOff( osg::Group* node );
void cullFrontBack( bool front, bool back, osg::StateSet* stateset );

osg::Vec3d computePolygonNormal( const osg::Vec3dArray* polygon );
osg::Vec3f computePolygonNormal( const osg::Vec3Array* polygon );
carve::geom::vector<3> computePolygonCentroid( const std::vector<carve::geom::vector<3> >& polygon );
carve::geom::vector<3> computePolygonNormal( const std::vector<carve::geom::vector<3> >& polygon );
carve::geom::vector<3> computePolygon2DNormal( const std::vector<carve::geom::vector<2> >& polygon );

bool LineSegmentToLineIntersection(carve::geom::vector<2>& v1, carve::geom::vector<2>& v2, carve::geom::vector<2>& v3, carve::geom::vector<2>& v4, std::vector<carve::geom::vector<2> >& result );
bool LineSegmentToLineSegmentIntersection(carve::geom::vector<2>& v1, carve::geom::vector<2>& v2, carve::geom::vector<2>& v3, carve::geom::vector<2>& v4, std::vector<carve::geom::vector<2> >& result );

void appendPointsToCurve( const std::vector<carve::geom::vector<3> >& points_vec, std::vector<carve::geom::vector<3> >& target_vec );
void appendPointsToCurve( const std::vector<carve::geom::vector<2> >& points_vec, std::vector<carve::geom::vector<3> >& target_vec );
void computeInverse( const carve::math::Matrix& matrix_a, carve::math::Matrix& inv );
void closestPointOnLine( carve::geom::vector<3>& closest, const carve::geom::vector<3>& point, const carve::geom::vector<3>& line_origin, const carve::geom::vector<3>& line_direction );
void closestPointOnLine( osg::Vec3d& closest, const osg::Vec3d& point, const osg::Vec3d& line_origin, const osg::Vec3d& line_direction );
bool isPointOnLineSegment( double& lambda, const osg::Vec3d& point, const osg::Vec3d& line_origin, const osg::Vec3d& line_direction );
void extrude3D( const std::vector<std::vector<carve::geom::vector<3> > >& paths, const carve::geom::vector<3> dir, shared_ptr<carve::input::PolyhedronData>& poly_data, std::stringstream& err );
void extrude(	const std::vector<std::vector<carve::geom::vector<2> > >& paths, const carve::geom::vector<3> dir, shared_ptr<carve::input::PolyhedronData>& poly_data, std::stringstream& err );
void makeLookAt(const carve::geom::vector<3>& eye,const carve::geom::vector<3>& center,const carve::geom::vector<3>& up, carve::math::Matrix& m );
bool bisectingPlane( const carve::geom::vector<3>& v1, const carve::geom::vector<3>& v2, const carve::geom::vector<3>& v3, carve::geom::vector<3>& normal );
void convertPlane2Matrix( const carve::geom::vector<3>& plane_normal, const carve::geom::vector<3>& plane_position, 
						 const carve::geom::vector<3>& local_z, carve::math::Matrix& resulting_matrix );

void renderMeshsetInDebugViewer( osgViewer::View* view, shared_ptr<carve::mesh::MeshSet<3> >& meshset, osg::Vec4f& color, bool wireframe );
void renderPolylineInDebugViewer( osgViewer::View* view, shared_ptr<carve::input::PolylineSetData >& poly_line, osg::Vec4f& color );
