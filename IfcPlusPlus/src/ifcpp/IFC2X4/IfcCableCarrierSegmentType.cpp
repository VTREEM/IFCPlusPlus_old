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
#include "include/IfcCableCarrierSegmentType.h"
#include "include/IfcCableCarrierSegmentTypeEnum.h"
#include "include/IfcGloballyUniqueId.h"
#include "include/IfcIdentifier.h"
#include "include/IfcLabel.h"
#include "include/IfcOwnerHistory.h"
#include "include/IfcPropertySetDefinition.h"
#include "include/IfcRelAggregates.h"
#include "include/IfcRelAssigns.h"
#include "include/IfcRelAssignsToProduct.h"
#include "include/IfcRelAssociates.h"
#include "include/IfcRelDeclares.h"
#include "include/IfcRelDefinesByType.h"
#include "include/IfcRelNests.h"
#include "include/IfcRepresentationMap.h"
#include "include/IfcText.h"

// ENTITY IfcCableCarrierSegmentType 
IfcCableCarrierSegmentType::IfcCableCarrierSegmentType() { m_entity_enum = IFCCABLECARRIERSEGMENTTYPE; }
IfcCableCarrierSegmentType::IfcCableCarrierSegmentType( int id ) { m_id = id; m_entity_enum = IFCCABLECARRIERSEGMENTTYPE; }
IfcCableCarrierSegmentType::~IfcCableCarrierSegmentType() {}

// method setEntity takes over all attributes from another instance of the class
void IfcCableCarrierSegmentType::setEntity( shared_ptr<IfcPPEntity> other_entity )
{
	shared_ptr<IfcCableCarrierSegmentType> other = dynamic_pointer_cast<IfcCableCarrierSegmentType>(other_entity);
	if( !other) { return; }
	m_GlobalId = other->m_GlobalId;
	m_OwnerHistory = other->m_OwnerHistory;
	m_Name = other->m_Name;
	m_Description = other->m_Description;
	m_ApplicableOccurrence = other->m_ApplicableOccurrence;
	m_HasPropertySets = other->m_HasPropertySets;
	m_RepresentationMaps = other->m_RepresentationMaps;
	m_Tag = other->m_Tag;
	m_ElementType = other->m_ElementType;
	m_PredefinedType = other->m_PredefinedType;
}
void IfcCableCarrierSegmentType::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_id << "=IFCCABLECARRIERSEGMENTTYPE" << "(";
	if( m_GlobalId ) { m_GlobalId->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_OwnerHistory ) { stream << "#" << m_OwnerHistory->getId(); } else { stream << "$"; }
	stream << ",";
	if( m_Name ) { m_Name->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_Description ) { m_Description->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_ApplicableOccurrence ) { m_ApplicableOccurrence->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	writeEntityList( stream, m_HasPropertySets );
	stream << ",";
	writeEntityList( stream, m_RepresentationMaps );
	stream << ",";
	if( m_Tag ) { m_Tag->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_ElementType ) { m_ElementType->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_PredefinedType ) { m_PredefinedType->getStepParameter( stream ); } else { stream << "$"; }
	stream << ");";
}
void IfcCableCarrierSegmentType::getStepParameter( std::stringstream& stream, bool ) const { stream << "#" << m_id; }
void IfcCableCarrierSegmentType::readStepData( std::vector<std::string>& args, const std::map<int,shared_ptr<IfcPPEntity> >& map )
{
	const int num_args = (int)args.size();
	if( num_args<10 ){ std::stringstream strserr; strserr << "Wrong parameter count for entity IfcCableCarrierSegmentType, expecting 10, having " << num_args << ". Object id: " << getId() << std::endl; throw IfcPPException( strserr.str().c_str() ); }
	#ifdef _DEBUG
	if( num_args>10 ){ std::cout << "Wrong parameter count for entity IfcCableCarrierSegmentType, expecting 10, having " << num_args << ". Object id: " << getId() << std::endl; }
	#endif
	m_GlobalId = IfcGloballyUniqueId::readStepData( args[0] );
	readEntityReference( args[1], m_OwnerHistory, map );
	m_Name = IfcLabel::readStepData( args[2] );
	m_Description = IfcText::readStepData( args[3] );
	m_ApplicableOccurrence = IfcIdentifier::readStepData( args[4] );
	readEntityReferenceList( args[5], m_HasPropertySets, map );
	readEntityReferenceList( args[6], m_RepresentationMaps, map );
	m_Tag = IfcLabel::readStepData( args[7] );
	m_ElementType = IfcLabel::readStepData( args[8] );
	m_PredefinedType = IfcCableCarrierSegmentTypeEnum::readStepData( args[9] );
}
void IfcCableCarrierSegmentType::setInverseCounterparts( shared_ptr<IfcPPEntity> ptr_self_entity )
{
	IfcFlowSegmentType::setInverseCounterparts( ptr_self_entity );
}
void IfcCableCarrierSegmentType::unlinkSelf()
{
	IfcFlowSegmentType::unlinkSelf();
}
