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

#include <osg/Array>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Point>

#include <ifcpp/IFC2X4/include/IfcRationalBSplineSurfaceWithKnots.h>
#include <ifcpp/IFC2X4/include/IfcParameterValue.h>
#include <ifcpp/IFC2X4/include/IfcLengthMeasure.h>
#include <ifcpp/IFC2X4/include/IfcCartesianPoint.h>
#include <ifcpp/model/shared_ptr.h>
#include <ifcpp/model/UnitConverter.h>

#include "RepresentationConverter.h"

//  int											m_UDegree;
//  int											m_VDegree;
//  std::vector<std::vector<shared_ptr<IfcCartesianPoint> > >	m_ControlPointsList;
//  shared_ptr<IfcBSplineSurfaceForm>			m_SurfaceForm;
//  bool											m_UClosed;
//  bool											m_VClosed;
//  bool											m_SelfIntersect;

// IfcBSplineSurfaceWithKnots -----------------------------------------------------------
// attributes:
//std::vector<int >							m_UMultiplicities;
//std::vector<int >							m_VMultiplicities;
//std::vector<shared_ptr<IfcParameterValue> >	m_UKnots;
//std::vector<shared_ptr<IfcParameterValue> >	m_VKnots;
//shared_ptr<IfcKnotType>						m_KnotSpec;

void convertIfcCartesianPointVector2D( std::vector<std::vector<shared_ptr<IfcCartesianPoint> > >& points, double length_factor, osg::Vec3Array* vertices )
{
	std::vector<std::vector<shared_ptr<IfcCartesianPoint> > >::iterator it_cp_outer;
	for( it_cp_outer=points.begin(); it_cp_outer!=points.end(); ++it_cp_outer )
	{
		std::vector<shared_ptr<IfcCartesianPoint> >& points_inner = (*it_cp_outer);
		std::vector<shared_ptr<IfcCartesianPoint> >::iterator it_cp;
		for( it_cp=points_inner.begin(); it_cp!=points_inner.end(); ++it_cp )
		{
			shared_ptr<IfcCartesianPoint> cp = (*it_cp);

			if( !cp )
			{
				continue;
			}

			std::vector<shared_ptr<IfcLengthMeasure> >& coords = cp->m_Coordinates;
			if( coords.size() > 2 )
			{
				vertices->push_back( osg::Vec3( coords[0]->m_value*length_factor, coords[1]->m_value*length_factor, coords[2]->m_value*length_factor ) );
			}
			else if( coords.size() > 1 )
			{
				vertices->push_back( osg::Vec3( coords[0]->m_value*length_factor, coords[1]->m_value*length_factor, 0.0 ) );
			}
		}
	}
}


void RepresentationConverter::convertIfcBSplineSurface( const shared_ptr<IfcRationalBSplineSurfaceWithKnots>& ifc_surface, const carve::math::Matrix& pos, shared_ptr<carve::input::PolylineSetData>& polyline_data )
{
	std::vector<shared_ptr<IfcParameterValue> >& ifc_u_knots = ifc_surface->m_UKnots;
	std::vector<shared_ptr<IfcParameterValue> >& ifc_v_knots = ifc_surface->m_VKnots;
	std::vector<std::vector<shared_ptr<IfcCartesianPoint> > >& ifc_control_points = ifc_surface->m_ControlPointsList;
	std::vector<std::vector<double> > vec_weights = ifc_surface->m_WeightsData;

	osg::ref_ptr<osg::Vec3Array> control_point_array = new osg::Vec3Array();
	double length_factor = m_unit_converter->getLengthInMeterFactor();
	convertIfcCartesianPointVector2D( ifc_control_points, length_factor, control_point_array );
	int degree_u = ifc_surface->m_UDegree;
	int degree_v = ifc_surface->m_VDegree;
	unsigned int numPathU=10;
	unsigned int numPathV=10;

	const unsigned int eta = ifc_control_points.size();
	if( eta < 2 )
	{
		return;
	}
	const unsigned int zeta = ifc_control_points[0].size();

	const int num_points_per_section = eta*zeta;

	
}
