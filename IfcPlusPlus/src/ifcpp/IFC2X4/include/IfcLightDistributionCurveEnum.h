/* -*-c++-*- IfcPlusPlus - www.ifcplusplus.com - Copyright (C) 2011 Fabian Gerold
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
#include <map>
#include <sstream>
#include <string>
#include "ifcpp/model/shared_ptr.h"
#include "ifcpp/model/IfcPPObject.h"

// TYPE IfcLightDistributionCurveEnum = ENUMERATION OF	(TYPE_A	,TYPE_B	,TYPE_C	,NOTDEFINED);
class IfcLightDistributionCurveEnum : public IfcPPAbstractEnum, public IfcPPType
{
public:
	enum IfcLightDistributionCurveEnumEnum
	{
		ENUM_TYPE_A,
		ENUM_TYPE_B,
		ENUM_TYPE_C,
		ENUM_NOTDEFINED
	};

	IfcLightDistributionCurveEnum();
	IfcLightDistributionCurveEnum( IfcLightDistributionCurveEnumEnum e ) { m_enum = e; }
	~IfcLightDistributionCurveEnum();
	virtual const char* classname() const { return "IfcLightDistributionCurveEnum"; }
	virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
	static shared_ptr<IfcLightDistributionCurveEnum> readStepData( std::string& arg );
	IfcLightDistributionCurveEnumEnum m_enum;
};
