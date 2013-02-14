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

// TYPE IfcBridgeSegmentType = ENUMERATION OF (TYPICAL_SEGMENT,PIER_SEGMENT,PIECE,LIFT,ELEMENT,JAMB,PYLON_HEAD,SPAN,CANTILEVER,FINITE_ELEMENT_REFERENCE);
class IfcBridgeSegmentType : public IfcPPAbstractEnum, public IfcPPType
{
public:
	enum IfcBridgeSegmentTypeEnum
	{
		ENUM_TYPICAL_SEGMENT,
		ENUM_PIER_SEGMENT,
		ENUM_PIECE,
		ENUM_LIFT,
		ENUM_ELEMENT,
		ENUM_JAMB,
		ENUM_PYLON_HEAD,
		ENUM_SPAN,
		ENUM_CANTILEVER,
		ENUM_FINITE_ELEMENT_REFERENCE
	};

	IfcBridgeSegmentType();
	IfcBridgeSegmentType( IfcBridgeSegmentTypeEnum e ) { m_enum = e; }
	~IfcBridgeSegmentType();
	virtual const char* classname() const { return "IfcBridgeSegmentType"; }
	virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
	static shared_ptr<IfcBridgeSegmentType> readStepData( std::string& arg );
	IfcBridgeSegmentTypeEnum m_enum;
};

