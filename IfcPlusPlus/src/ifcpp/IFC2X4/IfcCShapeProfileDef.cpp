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
#include "include/IfcAxis2Placement2D.h"
#include "include/IfcCShapeProfileDef.h"
#include "include/IfcExternalReferenceRelationship.h"
#include "include/IfcLabel.h"
#include "include/IfcNonNegativeLengthMeasure.h"
#include "include/IfcPositiveLengthMeasure.h"
#include "include/IfcProfileProperties.h"
#include "include/IfcProfileTypeEnum.h"

// ENTITY IfcCShapeProfileDef 
IfcCShapeProfileDef::IfcCShapeProfileDef() { m_entity_enum = IFCCSHAPEPROFILEDEF; }
IfcCShapeProfileDef::IfcCShapeProfileDef( int id ) { m_id = id; m_entity_enum = IFCCSHAPEPROFILEDEF; }
IfcCShapeProfileDef::~IfcCShapeProfileDef() {}

// method setEntity takes over all attributes from another instance of the class
void IfcCShapeProfileDef::setEntity( shared_ptr<IfcPPEntity> other_entity )
{
	shared_ptr<IfcCShapeProfileDef> other = dynamic_pointer_cast<IfcCShapeProfileDef>(other_entity);
	if( !other) { return; }
	m_ProfileType = other->m_ProfileType;
	m_ProfileName = other->m_ProfileName;
	m_Position = other->m_Position;
	m_Depth = other->m_Depth;
	m_Width = other->m_Width;
	m_WallThickness = other->m_WallThickness;
	m_Girth = other->m_Girth;
	m_InternalFilletRadius = other->m_InternalFilletRadius;
}
void IfcCShapeProfileDef::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_id << "=IFCCSHAPEPROFILEDEF" << "(";
	if( m_ProfileType ) { m_ProfileType->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_ProfileName ) { m_ProfileName->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_Position ) { stream << "#" << m_Position->getId(); } else { stream << "$"; }
	stream << ",";
	if( m_Depth ) { m_Depth->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_Width ) { m_Width->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_WallThickness ) { m_WallThickness->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_Girth ) { m_Girth->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_InternalFilletRadius ) { m_InternalFilletRadius->getStepParameter( stream ); } else { stream << "$"; }
	stream << ");";
}
void IfcCShapeProfileDef::getStepParameter( std::stringstream& stream, bool ) const { stream << "#" << m_id; }
void IfcCShapeProfileDef::readStepData( std::vector<std::string>& args, const std::map<int,shared_ptr<IfcPPEntity> >& map )
{
	const int num_args = (int)args.size();
	if( num_args<8 ){ std::stringstream strserr; strserr << "Wrong parameter count for entity IfcCShapeProfileDef, expecting 8, having " << num_args << ". Object id: " << getId() << std::endl; throw IfcPPException( strserr.str().c_str() ); }
	#ifdef _DEBUG
	if( num_args>8 ){ std::cout << "Wrong parameter count for entity IfcCShapeProfileDef, expecting 8, having " << num_args << ". Object id: " << getId() << std::endl; }
	#endif
	m_ProfileType = IfcProfileTypeEnum::readStepData( args[0] );
	m_ProfileName = IfcLabel::readStepData( args[1] );
	readEntityReference( args[2], m_Position, map );
	m_Depth = IfcPositiveLengthMeasure::readStepData( args[3] );
	m_Width = IfcPositiveLengthMeasure::readStepData( args[4] );
	m_WallThickness = IfcPositiveLengthMeasure::readStepData( args[5] );
	m_Girth = IfcPositiveLengthMeasure::readStepData( args[6] );
	m_InternalFilletRadius = IfcNonNegativeLengthMeasure::readStepData( args[7] );
}
void IfcCShapeProfileDef::setInverseCounterparts( shared_ptr<IfcPPEntity> ptr_self_entity )
{
	IfcParameterizedProfileDef::setInverseCounterparts( ptr_self_entity );
}
void IfcCShapeProfileDef::unlinkSelf()
{
	IfcParameterizedProfileDef::unlinkSelf();
}
