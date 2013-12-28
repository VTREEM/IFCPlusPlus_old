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
#include "include/IfcDraughtingPreDefinedCurveFont.h"
#include "include/IfcLabel.h"

// ENTITY IfcDraughtingPreDefinedCurveFont 
IfcDraughtingPreDefinedCurveFont::IfcDraughtingPreDefinedCurveFont() { m_entity_enum = IFCDRAUGHTINGPREDEFINEDCURVEFONT; }
IfcDraughtingPreDefinedCurveFont::IfcDraughtingPreDefinedCurveFont( int id ) { m_id = id; m_entity_enum = IFCDRAUGHTINGPREDEFINEDCURVEFONT; }
IfcDraughtingPreDefinedCurveFont::~IfcDraughtingPreDefinedCurveFont() {}

// method setEntity takes over all attributes from another instance of the class
void IfcDraughtingPreDefinedCurveFont::setEntity( shared_ptr<IfcPPEntity> other_entity )
{
	shared_ptr<IfcDraughtingPreDefinedCurveFont> other = dynamic_pointer_cast<IfcDraughtingPreDefinedCurveFont>(other_entity);
	if( !other) { return; }
	m_Name = other->m_Name;
}
void IfcDraughtingPreDefinedCurveFont::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_id << "=IFCDRAUGHTINGPREDEFINEDCURVEFONT" << "(";
	if( m_Name ) { m_Name->getStepParameter( stream ); } else { stream << "$"; }
	stream << ");";
}
void IfcDraughtingPreDefinedCurveFont::getStepParameter( std::stringstream& stream, bool ) const { stream << "#" << m_id; }
void IfcDraughtingPreDefinedCurveFont::readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<IfcPPEntity> >& map )
{
	const int num_args = (int)args.size();
	if( num_args<1 ){ std::stringstream strserr; strserr << "Wrong parameter count for entity IfcDraughtingPreDefinedCurveFont, expecting 1, having " << num_args << ". Object id: " << getId() << std::endl; throw IfcPPException( strserr.str().c_str() ); }
	#ifdef _DEBUG
	if( num_args>1 ){ std::cout << "Wrong parameter count for entity IfcDraughtingPreDefinedCurveFont, expecting 1, having " << num_args << ". Object id: " << getId() << std::endl; }
	#endif
	m_Name = IfcLabel::createObjectFromStepData( args[0] );
}
void IfcDraughtingPreDefinedCurveFont::setInverseCounterparts( shared_ptr<IfcPPEntity> ptr_self_entity )
{
	IfcPreDefinedCurveFont::setInverseCounterparts( ptr_self_entity );
}
void IfcDraughtingPreDefinedCurveFont::unlinkSelf()
{
	IfcPreDefinedCurveFont::unlinkSelf();
}
