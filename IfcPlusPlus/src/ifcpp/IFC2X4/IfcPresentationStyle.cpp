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
#include "include/IfcPresentationStyle.h"

// ENTITY IfcPresentationStyle 
IfcPresentationStyle::IfcPresentationStyle() { m_entity_enum = IFCPRESENTATIONSTYLE; }
IfcPresentationStyle::IfcPresentationStyle( int id ) { m_id = id; m_entity_enum = IFCPRESENTATIONSTYLE; }
IfcPresentationStyle::~IfcPresentationStyle() {}

// method setEntity takes over all attributes from another instance of the class
void IfcPresentationStyle::setEntity( shared_ptr<IfcPPEntity> other_entity )
{
	shared_ptr<IfcPresentationStyle> other = dynamic_pointer_cast<IfcPresentationStyle>(other_entity);
	if( !other) { return; }
	m_Name = other->m_Name;
	m_ModelOrDraughting = other->m_ModelOrDraughting;
}
void IfcPresentationStyle::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_id << "=IFCPRESENTATIONSTYLE" << "(";
	if( m_Name ) { m_Name->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_ModelOrDraughting == false ) { stream << ".F."; }
	else if( m_ModelOrDraughting == true ) { stream << ".T."; }
	stream << ");";
}
void IfcPresentationStyle::getStepParameter( std::stringstream& stream, bool ) const { stream << "#" << m_id; }
void IfcPresentationStyle::readStepData( std::vector<std::string>& args, const std::map<int,shared_ptr<IfcPPEntity> >& map )
{
	const int num_args = (int)args.size();
	if( num_args<2 ){ std::stringstream strserr; strserr << "Wrong parameter count for entity IfcPresentationStyle, expecting 2, having " << num_args << ". Object id: " << getId() << std::endl; throw IfcPPException( strserr.str().c_str() ); }
	#ifdef _DEBUG
	if( num_args>2 ){ std::cout << "Wrong parameter count for entity IfcPresentationStyle, expecting 2, having " << num_args << ". Object id: " << getId() << std::endl; }
	#endif
	m_Name = IfcLabel::readStepData( args[0] );
	if( _stricmp( args[1].c_str(), ".F." ) == 0 ) { m_ModelOrDraughting = false; }
	else if( _stricmp( args[1].c_str(), ".T." ) == 0 ) { m_ModelOrDraughting = true; }
}
void IfcPresentationStyle::setInverseCounterparts( shared_ptr<IfcPPEntity> )
{
}
void IfcPresentationStyle::unlinkSelf()
{
}