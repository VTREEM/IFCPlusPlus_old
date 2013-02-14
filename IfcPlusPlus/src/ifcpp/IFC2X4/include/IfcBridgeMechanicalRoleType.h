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

// TYPE IfcBridgeMechanicalRoleType = ENUMERATION OF (    LONGITUDINAL,    TRANSVERSAL,    COMPLETE,    NONE,    UNDEFINED);
class IfcBridgeMechanicalRoleType : public IfcPPAbstractEnum, public IfcPPType
{
public:
	enum IfcBridgeMechanicalRoleTypeEnum
	{
		ENUM_LONGITUDINAL,
		ENUM_TRANSVERSAL,
		ENUM_COMPLETE,
		ENUM_NONE,
		ENUM_UNDEFINED
	};

	IfcBridgeMechanicalRoleType();
	IfcBridgeMechanicalRoleType( IfcBridgeMechanicalRoleTypeEnum e ) { m_enum = e; }
	~IfcBridgeMechanicalRoleType();
	virtual const char* classname() const { return "IfcBridgeMechanicalRoleType"; }
	virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
	static shared_ptr<IfcBridgeMechanicalRoleType> readStepData( std::string& arg );
	IfcBridgeMechanicalRoleTypeEnum m_enum;
};

