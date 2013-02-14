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

#include <osg/StateSet>
#include "ifcpp/model/shared_ptr.h"

class IfcStyledItem;
class IfcPresentationStyle;
class IfcPresentationStyleSelect;
class IfcCurveStyle;
class IfcSurfaceStyle;

class StylesConverter
{
public:
	StylesConverter();
	~StylesConverter();
	
	osg::StateSet* convertIfcStyledItem( weak_ptr<IfcStyledItem> item );
	osg::StateSet* convertIfcComplexPropertyColor( shared_ptr<IfcComplexProperty> complex_property );
	osg::StateSet* convertIfcSurfaceStyle( shared_ptr<IfcSurfaceStyle> surface_style );
	osg::StateSet* convertIfcPresentationStyle( shared_ptr<IfcPresentationStyle> presentation_style );
	osg::StateSet* convertIfcPresentationStyleSelect( shared_ptr<IfcPresentationStyleSelect> presentation_style );
	osg::StateSet* convertIfcCurveStyle( shared_ptr<IfcCurveStyle> curve_style );
	
	std::map<int,osg::ref_ptr<osg::StateSet> > m_map_ifc_styles;
};

