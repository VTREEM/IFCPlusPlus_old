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
#include "include/IfcCartesianPoint.h"
#include "include/IfcPlacement.h"
#include "include/IfcPresentationLayerAssignment.h"
#include "include/IfcStyledItem.h"

// ENTITY IfcPlacement 
IfcPlacement::IfcPlacement() {}
IfcPlacement::IfcPlacement( int id ) { m_id = id; }
IfcPlacement::~IfcPlacement() {}

// method setEntity takes over all attributes from another instance of the class
void IfcPlacement::setEntity( shared_ptr<IfcPPEntity> other_entity )
{
	shared_ptr<IfcPlacement> other = dynamic_pointer_cast<IfcPlacement>(other_entity);
	if( !other) { return; }
	m_Location = other->m_Location;
}
void IfcPlacement::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_id << "=IFCPLACEMENT" << "(";
	if( m_Location ) { stream << "#" << m_Location->getId(); } else { stream << "$"; }
	stream << ");";
}
void IfcPlacement::getStepParameter( std::stringstream& stream, bool ) const { stream << "#" << m_id; }
void IfcPlacement::readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<IfcPPEntity> >& map )
{
	const int num_args = (int)args.size();
	if( num_args<1 ){ std::stringstream strserr; strserr << "Wrong parameter count for entity IfcPlacement, expecting 1, having " << num_args << ". Object id: " << getId() << std::endl; throw IfcPPException( strserr.str().c_str() ); }
	#ifdef _DEBUG
	if( num_args>1 ){ std::cout << "Wrong parameter count for entity IfcPlacement, expecting 1, having " << num_args << ". Object id: " << getId() << std::endl; }
	#endif
	readEntityReference( args[0], m_Location, map );
}
void IfcPlacement::setInverseCounterparts( shared_ptr<IfcPPEntity> ptr_self_entity )
{
	IfcGeometricRepresentationItem::setInverseCounterparts( ptr_self_entity );
}
void IfcPlacement::unlinkSelf()
{
	IfcGeometricRepresentationItem::unlinkSelf();
}
