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

#include <sstream>
#include <limits>
#include <map>
#include "ifcpp/reader/ReaderUtil.h"
#include "ifcpp/writer/WriterUtil.h"
#include "ifcpp/model/shared_ptr.h"
#include "ifcpp/model/IfcPPException.h"
#include "include/IfcSimpleValue.h"
#include "include/IfcLogical.h"

// TYPE IfcLogical 
IfcLogical::IfcLogical() {}
IfcLogical::IfcLogical( bool value ) { m_value = value; }
IfcLogical::~IfcLogical() {}
void IfcLogical::getStepParameter( std::stringstream& stream, bool is_select_type ) const
{
	if( is_select_type ) { stream << "IFCLOGICAL("; }
	//supertype as attribute: bool m_value
	if( m_value == false )
	{
		stream << ".F.";
	}
	else if( m_value == true )
	{
		stream << ".T.";
	}
	if( is_select_type ) { stream << ")"; }
}
shared_ptr<IfcLogical> IfcLogical::createObjectFromStepData( const std::string& arg )
{
	// read TYPE
	if( arg.compare( "$" ) == 0 ) { return shared_ptr<IfcLogical>(); }
	auto type_object = std::make_shared<IfcLogical>();
	//supertype as attribute: bool m_value
	if( _stricmp( arg.c_str(), ".F." ) == 0 )
	{
		type_object->m_value = false;
	}
	else if( _stricmp( arg.c_str(), ".T." ) == 0 )
	{
		type_object->m_value = true;
	}
	return type_object;
}
