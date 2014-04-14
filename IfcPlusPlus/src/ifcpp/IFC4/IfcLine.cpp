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
#include "ifcpp/model/IfcPPAttributeObject.h"
#include "ifcpp/reader/ReaderUtil.h"
#include "ifcpp/writer/WriterUtil.h"
#include "ifcpp/IfcPPEntityEnums.h"
#include "include/IfcCartesianPoint.h"
#include "include/IfcLine.h"
#include "include/IfcPresentationLayerAssignment.h"
#include "include/IfcStyledItem.h"
#include "include/IfcVector.h"

// ENTITY IfcLine 
IfcLine::IfcLine() {}
IfcLine::IfcLine( int id ) { m_id = id; }
IfcLine::~IfcLine() {}

// method setEntity takes over all attributes from another instance of the class
void IfcLine::setEntity( shared_ptr<IfcPPEntity> other_entity )
{
	shared_ptr<IfcLine> other = dynamic_pointer_cast<IfcLine>(other_entity);
	if( !other) { return; }
	m_Pnt = other->m_Pnt;
	m_Dir = other->m_Dir;
}
void IfcLine::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_id << "=IFCLINE" << "(";
	if( m_Pnt ) { stream << "#" << m_Pnt->getId(); } else { stream << "$"; }
	stream << ",";
	if( m_Dir ) { stream << "#" << m_Dir->getId(); } else { stream << "$"; }
	stream << ");";
}
void IfcLine::getStepParameter( std::stringstream& stream, bool ) const { stream << "#" << m_id; }
void IfcLine::readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<IfcPPEntity> >& map )
{
	const int num_args = (int)args.size();
	if( num_args<2 ){ std::stringstream strserr; strserr << "Wrong parameter count for entity IfcLine, expecting 2, having " << num_args << ". Object id: " << getId() << std::endl; throw IfcPPException( strserr.str().c_str() ); }
	#ifdef _DEBUG
	if( num_args>2 ){ std::cout << "Wrong parameter count for entity IfcLine, expecting 2, having " << num_args << ". Object id: " << getId() << std::endl; }
	#endif
	readEntityReference( args[0], m_Pnt, map );
	readEntityReference( args[1], m_Dir, map );
}
void IfcLine::getAttributes( std::vector<std::pair<std::string, shared_ptr<IfcPPObject> > >& vec_attributes )
{
	IfcCurve::getAttributes( vec_attributes );
	vec_attributes.push_back( std::make_pair( "Pnt", m_Pnt ) );
	vec_attributes.push_back( std::make_pair( "Dir", m_Dir ) );
}
void IfcLine::getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<IfcPPObject> > >& vec_attributes )
{
}
void IfcLine::setInverseCounterparts( shared_ptr<IfcPPEntity> ptr_self_entity )
{
	IfcCurve::setInverseCounterparts( ptr_self_entity );
}
void IfcLine::unlinkSelf()
{
	IfcCurve::unlinkSelf();
}
