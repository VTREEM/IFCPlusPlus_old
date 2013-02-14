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

// TYPE IfcBridgeStructureType = ENUMERATION OF (BOX_GIRDER_BRIDGE,ARCHED_BRIDGE,SUSPENSION_BRIDGE,CABLE_STAYED_BRIDGE,GIRDER_BRIDGE,SLAB_BRIDGE,SLAB_BRIDGE_WITH_BROAD_CANTILEVER,BOW_STRING_BRIDGE,LADDER_BRIDGE,FRAMEWORK_BRIDGE,GISCLARD_BRIDGE,PORTAL_BRIDGE);
class IfcBridgeStructureType : public IfcPPAbstractEnum, public IfcPPType
{
public:
	enum IfcBridgeStructureTypeEnum
	{
		ENUM_BOX_GIRDER_BRIDGE,
		ENUM_ARCHED_BRIDGE,
		ENUM_SUSPENSION_BRIDGE,
		ENUM_CABLE_STAYED_BRIDGE,
		ENUM_GIRDER_BRIDGE,
		ENUM_SLAB_BRIDGE,
		ENUM_SLAB_BRIDGE_WITH_BROAD_CANTILEVER,
		ENUM_BOW_STRING_BRIDGE,
		ENUM_LADDER_BRIDGE,
		ENUM_FRAMEWORK_BRIDGE,
		ENUM_GISCLARD_BRIDGE,
		ENUM_PORTAL_BRIDGE
	};

	IfcBridgeStructureType();
	IfcBridgeStructureType( IfcBridgeStructureTypeEnum e ) { m_enum = e; }
	~IfcBridgeStructureType();
	virtual const char* classname() const { return "IfcBridgeStructureType"; }
	virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
	static shared_ptr<IfcBridgeStructureType> readStepData( std::string& arg );
	IfcBridgeStructureTypeEnum m_enum;
};

