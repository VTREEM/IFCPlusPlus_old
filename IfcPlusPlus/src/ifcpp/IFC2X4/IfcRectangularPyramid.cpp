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
#include "include/IfcAxis2Placement3D.h"
#include "include/IfcPositiveLengthMeasure.h"
#include "include/IfcPresentationLayerAssignment.h"
#include "include/IfcRectangularPyramid.h"
#include "include/IfcStyledItem.h"

// ENTITY IfcRectangularPyramid 
IfcRectangularPyramid::IfcRectangularPyramid() { m_entity_enum = IFCRECTANGULARPYRAMID; }
IfcRectangularPyramid::IfcRectangularPyramid( int id ) { m_id = id; m_entity_enum = IFCRECTANGULARPYRAMID; }
IfcRectangularPyramid::~IfcRectangularPyramid() {}

// method setEntity takes over all attributes from another instance of the class
void IfcRectangularPyramid::setEntity( shared_ptr<IfcPPEntity> other_entity )
{
	shared_ptr<IfcRectangularPyramid> other = dynamic_pointer_cast<IfcRectangularPyramid>(other_entity);
	if( !other) { return; }
	m_Position = other->m_Position;
	m_XLength = other->m_XLength;
	m_YLength = other->m_YLength;
	m_Height = other->m_Height;
}
void IfcRectangularPyramid::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_id << "=IFCRECTANGULARPYRAMID" << "(";
	if( m_Position ) { stream << "#" << m_Position->getId(); } else { stream << "$"; }
	stream << ",";
	if( m_XLength ) { m_XLength->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_YLength ) { m_YLength->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_Height ) { m_Height->getStepParameter( stream ); } else { stream << "$"; }
	stream << ");";
}
void IfcRectangularPyramid::getStepParameter( std::stringstream& stream, bool ) const { stream << "#" << m_id; }
void IfcRectangularPyramid::readStepData( std::vector<std::string>& args, const std::map<int,shared_ptr<IfcPPEntity> >& map )
{
	const int num_args = (int)args.size();
	if( num_args<4 ){ std::stringstream strserr; strserr << "Wrong parameter count for entity IfcRectangularPyramid, expecting 4, having " << num_args << ". Object id: " << getId() << std::endl; throw IfcPPException( strserr.str().c_str() ); }
	#ifdef _DEBUG
	if( num_args>4 ){ std::cout << "Wrong parameter count for entity IfcRectangularPyramid, expecting 4, having " << num_args << ". Object id: " << getId() << std::endl; }
	#endif
	readEntityReference( args[0], m_Position, map );
	m_XLength = IfcPositiveLengthMeasure::readStepData( args[1] );
	m_YLength = IfcPositiveLengthMeasure::readStepData( args[2] );
	m_Height = IfcPositiveLengthMeasure::readStepData( args[3] );
}
void IfcRectangularPyramid::setInverseCounterparts( shared_ptr<IfcPPEntity> ptr_self_entity )
{
	IfcCsgPrimitive3D::setInverseCounterparts( ptr_self_entity );
}
void IfcRectangularPyramid::unlinkSelf()
{
	IfcCsgPrimitive3D::unlinkSelf();
}
