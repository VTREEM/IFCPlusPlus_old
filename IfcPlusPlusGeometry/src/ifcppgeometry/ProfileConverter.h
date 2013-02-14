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

#include "ifcpp/model/shared_ptr.h"

#include <carve/geom3d.hpp>

class UnitConverter;
class IfcProfileDef;
class IfcArbitraryClosedProfileDef;
class IfcArbitraryOpenProfileDef;
class IfcCompositeProfileDef;
class IfcDerivedProfileDef;
class IfcParameterizedProfileDef;
class IfcNurbsProfile;

class ProfileConverter
{
public:
	ProfileConverter( shared_ptr<UnitConverter> unit_converter );
	~ProfileConverter();

	void setProfile( shared_ptr<IfcProfileDef> profile_def );
	const std::vector<std::vector<carve::geom::vector<3> > >& getCoordinates() { return m_paths; }
	static void addArc( std::vector<carve::geom::vector<3> >& coords, double radius, double start_angle, double opening_angle, double xM, double yM, int segments = 12 );
	static void addFullArc( std::vector<carve::geom::vector<3> >& coords, double radius, double start_angle, double opening_angle, double xM, double yM, int segments = 12 );
	void mirrorCopyPath( std::vector<carve::geom::vector<3> >& coords, bool horizontally, bool vertically ) const;
	void mirrorCopyPathBack( std::vector<carve::geom::vector<3> >& coords, bool horizontally, bool vertically ) const;

private:
	shared_ptr<UnitConverter> m_unit_converter;
	std::vector<std::vector<carve::geom::vector<3> > > m_paths;
	
	void convertIfcArbitraryClosedProfileDef(	const shared_ptr<IfcArbitraryClosedProfileDef>& profile_def,	std::vector<std::vector<carve::geom::vector<3> > >& paths ) const;
	void convertIfcArbitraryOpenProfileDef(		const shared_ptr<IfcArbitraryOpenProfileDef>& profile_def,		std::vector<std::vector<carve::geom::vector<3> > >& paths ) const;
	void convertIfcCompositeProfileDef(			const shared_ptr<IfcCompositeProfileDef>& profile_def,			std::vector<std::vector<carve::geom::vector<3> > >& paths ) const;
	void convertIfcDerivedProfileDef(			const shared_ptr<IfcDerivedProfileDef>& profile_def,			std::vector<std::vector<carve::geom::vector<3> > >& paths ) const;
	void convertIfcParameterizedProfileDef(		const shared_ptr<IfcParameterizedProfileDef>& profile_def,		std::vector<std::vector<carve::geom::vector<3> > >& paths ) const;
	void convertIfcParameterizedProfileDefWithPosition(		const shared_ptr<IfcParameterizedProfileDef>& profile_def,		std::vector<std::vector<carve::geom::vector<3> > >& paths ) const;
	void deleteLastPoint( std::vector<carve::geom::vector<3> >& polygon ) const;
	void addAvoidingDuplicates( const std::vector<carve::geom::vector<3> >& polygon, std::vector<std::vector<carve::geom::vector<3> > >& paths ) const;
};
