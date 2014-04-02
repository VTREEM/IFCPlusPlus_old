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
#include "include/IfcCartesianTransformationOperator3DnonUniform.h"
#include "include/IfcDirection.h"
#include "include/IfcPresentationLayerAssignment.h"
#include "include/IfcStyledItem.h"

// ENTITY IfcCartesianTransformationOperator3DnonUniform 
IfcCartesianTransformationOperator3DnonUniform::IfcCartesianTransformationOperator3DnonUniform() {}
IfcCartesianTransformationOperator3DnonUniform::IfcCartesianTransformationOperator3DnonUniform( int id ) { m_id = id; }
IfcCartesianTransformationOperator3DnonUniform::~IfcCartesianTransformationOperator3DnonUniform() {}

// method setEntity takes over all attributes from another instance of the class
void IfcCartesianTransformationOperator3DnonUniform::setEntity( shared_ptr<IfcPPEntity> other_entity )
{
	shared_ptr<IfcCartesianTransformationOperator3DnonUniform> other = dynamic_pointer_cast<IfcCartesianTransformationOperator3DnonUniform>(other_entity);
	if( !other) { return; }
	m_Axis1 = other->m_Axis1;
	m_Axis2 = other->m_Axis2;
	m_LocalOrigin = other->m_LocalOrigin;
	m_Scale = other->m_Scale;
	m_Axis3 = other->m_Axis3;
	m_Scale2 = other->m_Scale2;
	m_Scale3 = other->m_Scale3;
}
void IfcCartesianTransformationOperator3DnonUniform::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_id << "=IFCCARTESIANTRANSFORMATIONOPERATOR3DNONUNIFORM" << "(";
	if( m_Axis1 ) { stream << "#" << m_Axis1->getId(); } else { stream << "$"; }
	stream << ",";
	if( m_Axis2 ) { stream << "#" << m_Axis2->getId(); } else { stream << "$"; }
	stream << ",";
	if( m_LocalOrigin ) { stream << "#" << m_LocalOrigin->getId(); } else { stream << "$"; }
	stream << ",";
	if( m_Scale == m_Scale ){ stream << m_Scale; }
	else { stream << "$"; }
	stream << ",";
	if( m_Axis3 ) { stream << "#" << m_Axis3->getId(); } else { stream << "$"; }
	stream << ",";
	if( m_Scale2 == m_Scale2 ){ stream << m_Scale2; }
	else { stream << "$"; }
	stream << ",";
	if( m_Scale3 == m_Scale3 ){ stream << m_Scale3; }
	else { stream << "$"; }
	stream << ");";
}
void IfcCartesianTransformationOperator3DnonUniform::getStepParameter( std::stringstream& stream, bool ) const { stream << "#" << m_id; }
void IfcCartesianTransformationOperator3DnonUniform::readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<IfcPPEntity> >& map )
{
	const int num_args = (int)args.size();
	if( num_args<7 ){ std::stringstream strserr; strserr << "Wrong parameter count for entity IfcCartesianTransformationOperator3DnonUniform, expecting 7, having " << num_args << ". Object id: " << getId() << std::endl; throw IfcPPException( strserr.str().c_str() ); }
	#ifdef _DEBUG
	if( num_args>7 ){ std::cout << "Wrong parameter count for entity IfcCartesianTransformationOperator3DnonUniform, expecting 7, having " << num_args << ". Object id: " << getId() << std::endl; }
	#endif
	readEntityReference( args[0], m_Axis1, map );
	readEntityReference( args[1], m_Axis2, map );
	readEntityReference( args[2], m_LocalOrigin, map );
	readRealValue( args[3], m_Scale );
	readEntityReference( args[4], m_Axis3, map );
	readRealValue( args[5], m_Scale2 );
	readRealValue( args[6], m_Scale3 );
}
void IfcCartesianTransformationOperator3DnonUniform::setInverseCounterparts( shared_ptr<IfcPPEntity> ptr_self_entity )
{
	IfcCartesianTransformationOperator3D::setInverseCounterparts( ptr_self_entity );
}
void IfcCartesianTransformationOperator3DnonUniform::unlinkSelf()
{
	IfcCartesianTransformationOperator3D::unlinkSelf();
}
