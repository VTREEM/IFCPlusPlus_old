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

#include <sstream>
#include <vector>
#include <map>
#include "ifcpp/model/shared_ptr.h"

#ifdef __GNUC__
#include "ifcpp/IfcPPTypeEnums.h"
#include "ifcpp/IfcPPEntityEnums.h"
#else
enum IfcPPTypeEnum;
enum IfcPPEntityEnum;
#endif

class IfcPPObject
{
public:
	virtual const char* classname() const { return "IfcPPObject"; }
};


// ENTITY
class IfcPPEntity : virtual public IfcPPObject
{
protected:
	int m_id;

public:
	IfcPPEntity();
	IfcPPEntity( int id );
	virtual ~IfcPPEntity();
	virtual const char* classname() const { return "IfcPPEntity"; }
	virtual void getStepLine( std::stringstream& stream ) const;
	virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
	virtual void readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<IfcPPEntity> >& map );
	virtual void getAttributes( std::vector<std::pair<std::string, shared_ptr<IfcPPObject> > >& vec_attributes );
	virtual void getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<IfcPPObject> > >& map_attributes );
	virtual void setInverseCounterparts( shared_ptr<IfcPPEntity> ptr_self );
	virtual void unlinkSelf();
	virtual const int getId() const { return m_id; }
	void setId( int id );
	std::string m_entity_argument_str;
	IfcPPEntityEnum m_entity_enum;
};
