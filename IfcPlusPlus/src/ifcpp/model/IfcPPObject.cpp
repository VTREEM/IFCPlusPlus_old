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

#include "IfcPPException.h"
#include "ifcpp/writer/WriterUtil.h"
#include "IfcPPObject.h"


void IfcPPBool::readArgument( const std::string& attribute_value )
{
	if( _stricmp( attribute_value.c_str(), ".F." ) == 0 )
	{
		m_value = false;
	}
	else if( _stricmp( attribute_value.c_str(), ".T." ) == 0 )
	{
		m_value = true;;
	}
}

void IfcPPLogical::readArgument( const std::string& attribute_value )
{
	if( _stricmp( attribute_value.c_str(), ".F." ) == 0 )
	{
		m_value = LOGICAL_FALSE;
	}
	else if( _stricmp( attribute_value.c_str(), ".T." ) == 0 )
	{
		m_value = LOGICAL_TRUE;
	}
	else if( _stricmp( attribute_value.c_str(), ".U." ) == 0 )
	{
		m_value = LOGICAL_UNKNOWN;;
	}
}

void IfcPPInt::readArgument( const std::string& attribute_value )
{
	m_value = atoi( attribute_value.c_str() );
}

void IfcPPReal::readArgument( const std::string& attribute_value )
{
	m_value = atof( attribute_value.c_str() );
}

void IfcPPString::readArgument( const std::string& attribute_value )
{
	m_value = attribute_value.c_str();
}

// ENTITY
IfcPPEntity::IfcPPEntity() : m_id(-1)
{
}

IfcPPEntity::IfcPPEntity( int id ) : m_id(id)
{
}

IfcPPEntity::~IfcPPEntity()
{
}

void IfcPPEntity::setId( int id )
{
	m_id = id;
}

void IfcPPEntity::readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<IfcPPEntity> >& map )
{
	throw IfcPPException("IfcPPEntity::readStepArguments(), this method should be overwritten");
}
void IfcPPEntity::getStepLine( std::stringstream& stream ) const
{
	throw IfcPPException("IfcPPEntity::getStepLine(), this method should be overwritten");
}

void IfcPPEntity::getStepParameter( std::stringstream& stream, bool ) const
{
	throw IfcPPException("IfcPPEntity::getStepParameter(), this method should be overwritten");
}

void IfcPPEntity::setInverseCounterparts( shared_ptr<IfcPPEntity> ptr_self )
{
	throw IfcPPException("IfcPPEntity::setInverseCounterparts(), this method should be overwritten");
}

void IfcPPEntity::unlinkSelf()
{
	throw IfcPPException("IfcPPEntity::unlinkSelf(), this method should be overwritten");
}

void IfcPPEntity::getAttributes( std::vector<std::pair<std::string, shared_ptr<IfcPPObject> > >& vec_attributes )
{
}

void IfcPPEntity::getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<IfcPPObject> > >& vec_attributes )
{
}
