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

#include <osg/Group>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/ShapeDrawable>
#include <osg/LineWidth>
#include <osg/Point>
#include <osgUtil/Tessellator>
#include <osgUtil/Optimizer>

#include "carve/input.hpp"
#include "ifcpp/IFC2X4/include/IfcSectionedSpine.h"
#include "ifcpp/IFC2X4/include/IfcReferencedSectionedSpine.h"
#include "ifcpp/IFC2X4/include/IfcReferenceCurve.h"
#include "ifcpp/IFC2X4/include/IfcReferenceCurve3D.h"
#include "ifcpp/IFC2X4/include/IfcCompositeCurve.h"
#include "ifcpp/IFC2X4/include/IfcRationalBSplineCurveWithKnots.h"
#include "ifcpp/IFC2X4/include/IfcProfileDef.h"
#include "ifcpp/IFC2X4/include/IfcReferencePlacement.h"

#include "ifcpp/IFC2X4/include/IfcCartesianPoint.h"
#include "ifcpp/IFC2X4/include/IfcReferenceCurvePlacement.h"
#include "ifcpp/IFC2X4/include/IfcArbitraryClosedProfileDef.h"
#include "ifcpp/IFC2X4/include/IfcPolyline.h"
#include "ifcpp/IFC2X4/include/IfcParameterValue.h"
#include "ifcpp/IFC2X4/include/IfcCartesianPoint.h"
#include "ifcpp/IFC2X4/include/IfcLengthMeasure.h"
#include "ifcpp/IFC2X4/include/IfcDirection.h"
#include "ifcpp/IFC2X4/include/IfcArbitraryProfileDefWithVoids.h"
#include "ifcpp/IFC2X4/include/IfcRationalBSplineSurfaceWithKnots.h"

#include "ifcpp/model/IfcPPModel.h"
#include "ifcpp/model/UnitConverter.h"
#include "ifcpp/model/IfcPPException.h"
#include "ifcpp/model/shared_ptr.h"
#include "ProfileConverter.h"
#include "RepresentationConverter.h"

void RepresentationConverter::convertIfcSectionedSpine( const shared_ptr<IfcSectionedSpine>& spine, const carve::math::Matrix& pos, shared_ptr<ItemData> item_data )
{
	const shared_ptr<IfcCompositeCurve> spine_curve = spine->m_SpineCurve;
	if( !spine_curve )
	{
		return;
	}
	const std::vector<shared_ptr<IfcProfileDef> >& vec_cross_sections = spine->m_CrossSections;
	const std::vector<shared_ptr<IfcAxis2Placement3D> >& vec_cross_section_positions = spine->m_CrossSectionPositions;

	std::vector<shared_ptr<IfcProfileDef> >::iterator it_cross_sections;

	unsigned int num_cross_sections = vec_cross_sections.size();
	if( vec_cross_section_positions.size() < num_cross_sections )
	{
		num_cross_sections = vec_cross_section_positions.size();
	}

	std::vector<shared_ptr<IfcCompositeCurveSegment> > segements = spine_curve->m_Segments;
	int num_segments = segements.size();
	if( vec_cross_section_positions.size() < num_segments+1 )
	{
		num_segments = vec_cross_section_positions.size()-1;
	}

	std::vector<carve::geom::vector<3> > curve_polygon;
	std::vector<carve::geom::vector<3> > segment_start_points;
	convertIfcCurve( spine_curve, curve_polygon, segment_start_points );

}


void RepresentationConverter::convertIfcReferencedSectionedSpine( const shared_ptr<IfcReferencedSectionedSpine>& spine, const carve::math::Matrix& pos, shared_ptr<ItemData> item_data )
{
	std::stringstream err;

	shared_ptr<IfcReferenceCurve> spine_curve = spine->m_SpineCurve;
	std::vector<shared_ptr<IfcProfileDef> >& vec_cross_sections_unordered = spine->m_CrossSections;
	std::vector<shared_ptr<IfcReferencePlacement> >& vec_cross_section_positions_unordered = spine->m_CrossSectionPositions;

	// copy cross sections
	std::vector<shared_ptr<IfcProfileDef> >::iterator it_cross_sections;
	std::vector<shared_ptr<IfcProfileDef> > vec_cross_sections = vec_cross_sections_unordered;

	// copy placements
	std::vector<shared_ptr<IfcReferencePlacement> >::iterator it_placements;
	std::vector<shared_ptr<IfcReferenceCurvePlacement> > vec_cross_section_positions;
	for( it_placements=vec_cross_section_positions_unordered.begin(); it_placements!=vec_cross_section_positions_unordered.end(); ++it_placements )
	{
		shared_ptr<IfcReferencePlacement> reference_placement = (*it_placements);

		shared_ptr<IfcReferenceCurvePlacement> reference_curve_placement = dynamic_pointer_cast<IfcReferenceCurvePlacement>(reference_placement);
		if( reference_curve_placement )
		{
			vec_cross_section_positions.push_back( reference_curve_placement );
		}
	}

	unsigned int num_cross_sections = vec_cross_sections.size();
	if( vec_cross_section_positions.size() < num_cross_sections )
	{
		num_cross_sections = vec_cross_section_positions.size();
	}

	shared_ptr<carve::input::PolyhedronData> polyhedron_data( new carve::input::PolyhedronData() );

	// sort placements according to abscissa
	std::vector<shared_ptr<IfcReferenceCurvePlacement> >::iterator it_curve_placements;
	std::vector<shared_ptr<IfcReferenceCurvePlacement> >::iterator it_curve_placements_inner;

	for( unsigned int i=0; i<num_cross_sections; ++i )
	{
		shared_ptr<IfcReferenceCurvePlacement> reference_curve_placement = vec_cross_section_positions[i];
		double abscissa = reference_curve_placement->m_CurvilinearAbscissa->m_value;

		for( unsigned int j=i+1; j<num_cross_sections; ++j )
		{
			shared_ptr<IfcReferenceCurvePlacement> other = vec_cross_section_positions[j];
			double abscissa_other = other->m_CurvilinearAbscissa->m_value;

			if( abscissa_other < abscissa )
			{
				// reordering necessary
				shared_ptr<IfcReferenceCurvePlacement> copy = vec_cross_section_positions[i];
				vec_cross_section_positions[i] = vec_cross_section_positions[j];
				vec_cross_section_positions[j] = copy;

				shared_ptr<IfcProfileDef> copy_profile = vec_cross_sections[i];
				vec_cross_sections[i] = vec_cross_sections[j];
				vec_cross_sections[j] = copy_profile;
				abscissa = abscissa_other;
			}
		}
	}

	if( dynamic_pointer_cast<IfcReferenceCurve3D>(spine_curve) )
	{
		shared_ptr<IfcReferenceCurve3D> spine_curve_3d = dynamic_pointer_cast<IfcReferenceCurve3D>(spine_curve);
		shared_ptr<IfcCurve> reference_curve = spine_curve_3d->m_Curve3D;

	}
}


bool RepresentationConverter::bisectingPlane( osg::Vec3d& n, const osg::Vec3d& v1, const osg::Vec3d& v2, const osg::Vec3d& v3)
{
	bool valid = false;
	osg::Vec3d v21 = v2 - v1;
	osg::Vec3d v32 = v3 - v2;
	double len21 = v21.length();
	double len32 = v32.length();

	if( len21 <= GEOM_TOLERANCE * len32)
	{
		if( len32 == 0.0)
		{
			// all three points lie ontop of one-another
			n.set( 0.0, 0.0, 0.0 );
			valid = false;
		}
		else
		{
			// return a normalized copy of v32 as bisector
			len32 = 1.0 / len32;
			n = v32*len32;
			valid = true;
		}

	}
	else
	{
		valid = true;
		if( len32 <= GEOM_TOLERANCE * len21)
		{
			// return v21 as bisector
			v21.normalize();
			n = v21;
		}
		else
		{
			v21.normalize();
			v32.normalize();

			double dot = v32*v21;

			// if dot == 1 or -1, then points are colinear
			if( (dot >= (1.0-GEOM_TOLERANCE)) || (dot <= (-1.0+GEOM_TOLERANCE)))
			{
				n = -v21;
			}
			else
			{
				n = (v32 + v21)*dot - v32 - v21;
				n.normalize();
			}
		}
	}
	return valid;
}

