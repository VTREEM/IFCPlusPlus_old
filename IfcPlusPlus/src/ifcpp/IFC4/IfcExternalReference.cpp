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
#include "include/IfcExternalReference.h"
#include "include/IfcExternalReferenceRelationship.h"
#include "include/IfcIdentifier.h"
#include "include/IfcLabel.h"
#include "include/IfcURIReference.h"

// ENTITY IfcExternalReference 
IfcExternalReference::IfcExternalReference() {}
IfcExternalReference::IfcExternalReference( int id ) { m_id = id; }
IfcExternalReference::~IfcExternalReference() {}

// method setEntity takes over all attributes from another instance of the class
void IfcExternalReference::setEntity( shared_ptr<IfcPPEntity> other_entity )
{
	shared_ptr<IfcExternalReference> other = dynamic_pointer_cast<IfcExternalReference>(other_entity);
	if( !other) { return; }
	m_Location = other->m_Location;
	m_Identification = other->m_Identification;
	m_Name = other->m_Name;
}
void IfcExternalReference::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_id << "=IFCEXTERNALREFERENCE" << "(";
	if( m_Location ) { m_Location->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_Identification ) { m_Identification->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_Name ) { m_Name->getStepParameter( stream ); } else { stream << "$"; }
	stream << ");";
}
void IfcExternalReference::getStepParameter( std::stringstream& stream, bool ) const { stream << "#" << m_id; }
void IfcExternalReference::readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<IfcPPEntity> >& map )
{
	const int num_args = (int)args.size();
	if( num_args<3 ){ std::stringstream strserr; strserr << "Wrong parameter count for entity IfcExternalReference, expecting 3, having " << num_args << ". Object id: " << getId() << std::endl; throw IfcPPException( strserr.str().c_str() ); }
	#ifdef _DEBUG
	if( num_args>3 ){ std::cout << "Wrong parameter count for entity IfcExternalReference, expecting 3, having " << num_args << ". Object id: " << getId() << std::endl; }
	#endif
	m_Location = IfcURIReference::createObjectFromStepData( args[0] );
	m_Identification = IfcIdentifier::createObjectFromStepData( args[1] );
	m_Name = IfcLabel::createObjectFromStepData( args[2] );
}
void IfcExternalReference::setInverseCounterparts( shared_ptr<IfcPPEntity> )
{
}
void IfcExternalReference::unlinkSelf()
{
}
