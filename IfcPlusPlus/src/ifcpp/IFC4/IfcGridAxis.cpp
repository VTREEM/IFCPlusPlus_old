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
#include "include/IfcBoolean.h"
#include "include/IfcCurve.h"
#include "include/IfcGrid.h"
#include "include/IfcGridAxis.h"
#include "include/IfcLabel.h"
#include "include/IfcVirtualGridIntersection.h"

// ENTITY IfcGridAxis 
IfcGridAxis::IfcGridAxis() {}
IfcGridAxis::IfcGridAxis( int id ) { m_id = id; }
IfcGridAxis::~IfcGridAxis() {}

// method setEntity takes over all attributes from another instance of the class
void IfcGridAxis::setEntity( shared_ptr<IfcPPEntity> other_entity )
{
	shared_ptr<IfcGridAxis> other = dynamic_pointer_cast<IfcGridAxis>(other_entity);
	if( !other) { return; }
	m_AxisTag = other->m_AxisTag;
	m_AxisCurve = other->m_AxisCurve;
	m_SameSense = other->m_SameSense;
}
void IfcGridAxis::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_id << "=IFCGRIDAXIS" << "(";
	if( m_AxisTag ) { m_AxisTag->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_AxisCurve ) { stream << "#" << m_AxisCurve->getId(); } else { stream << "$"; }
	stream << ",";
	if( m_SameSense ) { m_SameSense->getStepParameter( stream ); } else { stream << "$"; }
	stream << ");";
}
void IfcGridAxis::getStepParameter( std::stringstream& stream, bool ) const { stream << "#" << m_id; }
void IfcGridAxis::readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<IfcPPEntity> >& map )
{
	const int num_args = (int)args.size();
	if( num_args<3 ){ std::stringstream strserr; strserr << "Wrong parameter count for entity IfcGridAxis, expecting 3, having " << num_args << ". Object id: " << getId() << std::endl; throw IfcPPException( strserr.str().c_str() ); }
	#ifdef _DEBUG
	if( num_args>3 ){ std::cout << "Wrong parameter count for entity IfcGridAxis, expecting 3, having " << num_args << ". Object id: " << getId() << std::endl; }
	#endif
	m_AxisTag = IfcLabel::createObjectFromStepData( args[0] );
	readEntityReference( args[1], m_AxisCurve, map );
	m_SameSense = IfcBoolean::createObjectFromStepData( args[2] );
}
void IfcGridAxis::setInverseCounterparts( shared_ptr<IfcPPEntity> )
{
}
void IfcGridAxis::unlinkSelf()
{
}
