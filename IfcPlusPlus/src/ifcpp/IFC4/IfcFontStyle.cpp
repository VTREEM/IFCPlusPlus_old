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
#include "include/IfcFontStyle.h"

// TYPE IfcFontStyle 
IfcFontStyle::IfcFontStyle() {}
IfcFontStyle::IfcFontStyle( std::string value ) { m_value = value; }
IfcFontStyle::~IfcFontStyle() {}
void IfcFontStyle::getStepParameter( std::stringstream& stream, bool is_select_type ) const
{
	if( is_select_type ) { stream << "IFCFONTSTYLE("; }
	//supertype as attribute: std::string m_value
	stream << "'" << encodeStepString( m_value ) << "'";
	if( is_select_type ) { stream << ")"; }
}
shared_ptr<IfcFontStyle> IfcFontStyle::createObjectFromStepData( const std::string& arg )
{
	// read TYPE
	if( arg.compare( "$" ) == 0 ) { return shared_ptr<IfcFontStyle>(); }
	auto type_object = std::make_shared<IfcFontStyle>();
	//supertype as attribute: std::string m_value
	if( arg.at(0) == '\'' && arg.at(arg.size()-1) == '\'' )
	{
		type_object->m_value = arg.substr(1,arg.length()-2);
	}
	else
	{
		type_object->m_value = arg;
	}
	return type_object;
}
