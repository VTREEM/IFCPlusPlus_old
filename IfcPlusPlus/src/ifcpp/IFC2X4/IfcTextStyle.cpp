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

#include "ifcpp/model/IfcPPException.h"
#include "ifcpp/reader/ReaderUtil.h"
#include "ifcpp/writer/WriterUtil.h"
#include "ifcpp/IfcPPEntityEnums.h"
#include "include/IfcLabel.h"
#include "include/IfcTextFontSelect.h"
#include "include/IfcTextStyle.h"
#include "include/IfcTextStyleForDefinedFont.h"
#include "include/IfcTextStyleTextModel.h"

// ENTITY IfcTextStyle 
IfcTextStyle::IfcTextStyle() { m_entity_enum = IFCTEXTSTYLE; }
IfcTextStyle::IfcTextStyle( int id ) { m_id = id; m_entity_enum = IFCTEXTSTYLE; }
IfcTextStyle::~IfcTextStyle() {}

// method setEntity takes over all attributes from another instance of the class
void IfcTextStyle::setEntity( shared_ptr<IfcPPEntity> other_entity )
{
	shared_ptr<IfcTextStyle> other = dynamic_pointer_cast<IfcTextStyle>(other_entity);
	if( !other) { return; }
	m_Name = other->m_Name;
	m_ModelOrDraughting = other->m_ModelOrDraughting;
	m_TextCharacterAppearance = other->m_TextCharacterAppearance;
	m_TextStyle = other->m_TextStyle;
	m_TextFontStyle = other->m_TextFontStyle;
}
void IfcTextStyle::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_id << "=IFCTEXTSTYLE" << "(";
	if( m_Name ) { m_Name->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_ModelOrDraughting == false ) { stream << ".F."; }
	else if( m_ModelOrDraughting == true ) { stream << ".T."; }
	stream << ",";
	if( m_TextCharacterAppearance ) { stream << "#" << m_TextCharacterAppearance->getId(); } else { stream << "$"; }
	stream << ",";
	if( m_TextStyle ) { stream << "#" << m_TextStyle->getId(); } else { stream << "$"; }
	stream << ",";
	if( m_TextFontStyle ) { m_TextFontStyle->getStepParameter( stream, true ); } else { stream << "$"; }
	stream << ");";
}
void IfcTextStyle::getStepParameter( std::stringstream& stream, bool ) const { stream << "#" << m_id; }
void IfcTextStyle::readStepData( std::vector<std::string>& args, const std::map<int,shared_ptr<IfcPPEntity> >& map )
{
	const int num_args = (int)args.size();
	if( num_args<5 ){ std::stringstream strserr; strserr << "Wrong parameter count for entity IfcTextStyle, expecting 5, having " << num_args << ". Object id: " << getId() << std::endl; throw IfcPPException( strserr.str().c_str() ); }
	#ifdef _DEBUG
	if( num_args>5 ){ std::cout << "Wrong parameter count for entity IfcTextStyle, expecting 5, having " << num_args << ". Object id: " << getId() << std::endl; }
	#endif
	m_Name = IfcLabel::readStepData( args[0] );
	if( _stricmp( args[1].c_str(), ".F." ) == 0 ) { m_ModelOrDraughting = false; }
	else if( _stricmp( args[1].c_str(), ".T." ) == 0 ) { m_ModelOrDraughting = true; }
	readEntityReference( args[2], m_TextCharacterAppearance, map );
	readEntityReference( args[3], m_TextStyle, map );
	m_TextFontStyle = IfcTextFontSelect::readStepData( args[4], map );
}
void IfcTextStyle::setInverseCounterparts( shared_ptr<IfcPPEntity> ptr_self_entity )
{
	IfcPresentationStyle::setInverseCounterparts( ptr_self_entity );
}
void IfcTextStyle::unlinkSelf()
{
	IfcPresentationStyle::unlinkSelf();
}
