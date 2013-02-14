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

enum IfcPPEntityEnum;

class IfcPPObject
{
public:
	virtual const char* classname() const { return "IfcPPObject"; }
	virtual void getStepData( std::stringstream& ) {};
};

class IfcPPEntity : public IfcPPObject
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
	virtual void readStepData( std::vector<std::string>& args, const std::map<int,shared_ptr<IfcPPEntity> >& map );
	virtual void setInverseCounterparts( shared_ptr<IfcPPEntity> ptr_self );
	virtual void unlinkSelf();
	virtual const int getId() const { return m_id; }
	void setId( int id );
	std::string m_arguments;
	IfcPPEntityEnum m_entity_enum;
};

//// pure abstract class to derive IFC TYPEs from
class IfcPPType : public IfcPPObject
{
public:	
	virtual const char* classname() const { return "IfcPPType"; }
	virtual void getStepData( std::stringstream& ) {};
};


//// pure abstract class to derive IFC TYPEs from
class IfcPPAbstractSelect
{
public:
	virtual const char* classname() const { return "IfcPPAbstractSelect"; }
	virtual void getStepData( std::stringstream& ) {};
};

class IfcPPAbstractEnum
{
public:
	virtual const char* classname() const { return "IfcPPAbstractEnum"; }
	virtual void getStepData( std::stringstream& ) {};
};
