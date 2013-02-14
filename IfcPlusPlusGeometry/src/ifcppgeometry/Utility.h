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

namespace carve {
	namespace input { struct PolyhedronData; }
	namespace math { struct Matrix; }
	namespace geom { template<unsigned int ndim> struct vector; }
}

osg::ref_ptr<osg::Geode> createCoordinateAxes();
osg::ref_ptr<osg::Group> createCoordinateAxesArrows();
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
carve::geom::vector<3> computePolygonNormal( const std::vector<carve::geom::vector<3> >& polygon );
carve::geom::vector<3> computePolygon2DNormal( const std::vector<carve::geom::vector<2> >& polygon );

bool LineSegmentToLineIntersection(carve::geom::vector<2>& v1, carve::geom::vector<2>& v2, carve::geom::vector<2>& v3, carve::geom::vector<2>& v4, std::vector<carve::geom::vector<2> >& result );
bool LineSegmentToLineSegmentIntersection(carve::geom::vector<2>& v1, carve::geom::vector<2>& v2, carve::geom::vector<2>& v3, carve::geom::vector<2>& v4, std::vector<carve::geom::vector<2> >& result );

void computeInverse( const carve::math::Matrix& matrix_a, carve::math::Matrix& inv );
void closestPointOnLine( carve::geom::vector<3>& closest, const carve::geom::vector<3>& point, const carve::geom::vector<3>& line_origin, const carve::geom::vector<3>& line_direction );
void closestPointOnLine( osg::Vec3d& closest, const osg::Vec3d& point, const osg::Vec3d& line_origin, const osg::Vec3d& line_direction );
bool isPointOnLineSegment( double& lambda, const osg::Vec3d& point, const osg::Vec3d& line_origin, const osg::Vec3d& line_direction );
void extrude( const std::vector<std::vector<carve::geom::vector<3> > >& paths, const carve::geom::vector<3> dir, carve::input::PolyhedronData& poly_data );
void zoomToBoundingSphere( osgViewer::Viewer* viewer, const osg::BoundingSphere& bs, double ratio_w = 1.0 );
