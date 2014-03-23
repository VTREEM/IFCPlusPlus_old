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

#include <set>
#include <ifcpp/model/shared_ptr.h>
#include "IncludeCarveHeaders.h"

class IfcObjectPlacement;
class IfcPlacement;
class IfcCartesianTransformationOperator;
class IfcAxis2Placement2D;
class IfcAxis2Placement3D;
class UnitConverter;

class PlacementConverter
{
public:
	static void convertIfcPlacement(			const shared_ptr<IfcPlacement> placement,					carve::math::Matrix& matrix,	double length_factor );
	static void convertIfcObjectPlacement(		const shared_ptr<IfcObjectPlacement> object_placement,		carve::math::Matrix& matrix,	double length_factor, std::set<int>& already_applied );
	static void getWorldCoordinateSystem(		const shared_ptr<IfcRepresentationContext>& context,		carve::math::Matrix& matrix,	double length_factor, std::set<int>& already_applied );
	static void convertTransformationOperator(	const shared_ptr<IfcCartesianTransformationOperator> tr_op,	carve::math::Matrix& matrix,	double length_factor );
	static void convertIfcAxis2Placement2D(		const shared_ptr<IfcAxis2Placement2D> axis2placement2d,		carve::math::Matrix& matrix,	double length_factor );
	static void convertIfcAxis2Placement3D(		const shared_ptr<IfcAxis2Placement3D> axis2placement3d,		carve::math::Matrix& matrix,	double length_factor );
	static void getPlane(						const shared_ptr<IfcAxis2Placement3D> axis2placement3d,		carve::geom::plane<3>& plane,	carve::geom::vector<3>& translate, double length_factor );
	static void convertMatrix( const carve::math::Matrix& matrix, shared_ptr<IfcAxis2Placement3D>& axis2placement3d, double length_factor, int& entity_id, std::vector<shared_ptr<IfcPPEntity> >& vec_entities );

};
