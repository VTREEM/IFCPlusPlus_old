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
#include "include/IfcActorSelect.h"
#include "include/IfcConstraint.h"
#include "include/IfcConstraintEnum.h"
#include "include/IfcDateTime.h"
#include "include/IfcExternalReferenceRelationship.h"
#include "include/IfcLabel.h"
#include "include/IfcResourceConstraintRelationship.h"
#include "include/IfcText.h"

// ENTITY IfcConstraint 
IfcConstraint::IfcConstraint() { m_entity_enum = IFCCONSTRAINT; }
IfcConstraint::IfcConstraint( int id ) { m_id = id; m_entity_enum = IFCCONSTRAINT; }
IfcConstraint::~IfcConstraint() {}

// method setEntity takes over all attributes from another instance of the class
void IfcConstraint::setEntity( shared_ptr<IfcPPEntity> other_entity )
{
	shared_ptr<IfcConstraint> other = dynamic_pointer_cast<IfcConstraint>(other_entity);
	if( !other) { return; }
	m_Name = other->m_Name;
	m_Description = other->m_Description;
	m_ConstraintGrade = other->m_ConstraintGrade;
	m_ConstraintSource = other->m_ConstraintSource;
	m_CreatingActor = other->m_CreatingActor;
	m_CreationTime = other->m_CreationTime;
	m_UserDefinedGrade = other->m_UserDefinedGrade;
}
void IfcConstraint::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_id << "=IFCCONSTRAINT" << "(";
	if( m_Name ) { m_Name->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_Description ) { m_Description->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_ConstraintGrade ) { m_ConstraintGrade->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_ConstraintSource ) { m_ConstraintSource->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_CreatingActor ) { m_CreatingActor->getStepParameter( stream, true ); } else { stream << "$"; }
	stream << ",";
	if( m_CreationTime ) { m_CreationTime->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_UserDefinedGrade ) { m_UserDefinedGrade->getStepParameter( stream ); } else { stream << "$"; }
	stream << ");";
}
void IfcConstraint::getStepParameter( std::stringstream& stream, bool ) const { stream << "#" << m_id; }
void IfcConstraint::readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<IfcPPEntity> >& map )
{
	const int num_args = (int)args.size();
	if( num_args<7 ){ std::stringstream strserr; strserr << "Wrong parameter count for entity IfcConstraint, expecting 7, having " << num_args << ". Object id: " << getId() << std::endl; throw IfcPPException( strserr.str().c_str() ); }
	#ifdef _DEBUG
	if( num_args>7 ){ std::cout << "Wrong parameter count for entity IfcConstraint, expecting 7, having " << num_args << ". Object id: " << getId() << std::endl; }
	#endif
	m_Name = IfcLabel::createObjectFromStepData( args[0] );
	m_Description = IfcText::createObjectFromStepData( args[1] );
	m_ConstraintGrade = IfcConstraintEnum::createObjectFromStepData( args[2] );
	m_ConstraintSource = IfcLabel::createObjectFromStepData( args[3] );
	m_CreatingActor = IfcActorSelect::createObjectFromStepData( args[4], map );
	m_CreationTime = IfcDateTime::createObjectFromStepData( args[5] );
	m_UserDefinedGrade = IfcLabel::createObjectFromStepData( args[6] );
}
void IfcConstraint::setInverseCounterparts( shared_ptr<IfcPPEntity> )
{
}
void IfcConstraint::unlinkSelf()
{
}
