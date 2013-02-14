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
#include "include/IfcAxis2Placement.h"
#include "include/IfcClothoid.h"
#include "include/IfcLengthMeasure.h"
#include "include/IfcPresentationLayerAssignment.h"
#include "include/IfcStyledItem.h"

// ENTITY IfcClothoid 
IfcClothoid::IfcClothoid() { m_entity_enum = IFCCLOTHOID; }
IfcClothoid::IfcClothoid( int id ) { m_id = id; m_entity_enum = IFCCLOTHOID; }
IfcClothoid::~IfcClothoid() {}

// method setEntity takes over all attributes from another instance of the class
void IfcClothoid::setEntity( shared_ptr<IfcPPEntity> other_entity )
{
	shared_ptr<IfcClothoid> other = dynamic_pointer_cast<IfcClothoid>(other_entity);
	if( !other) { return; }
	m_Position = other->m_Position;
	m_ClothoidConstant = other->m_ClothoidConstant;
}
void IfcClothoid::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_id << "=IFCCLOTHOID" << "(";
	if( m_Position ) { m_Position->getStepParameter( stream, true ); } else { stream << "$"; }
	stream << ",";
	if( m_ClothoidConstant ) { m_ClothoidConstant->getStepParameter( stream ); } else { stream << "$"; }
	stream << ");";
}
void IfcClothoid::getStepParameter( std::stringstream& stream, bool ) const { stream << "#" << m_id; }
void IfcClothoid::readStepData( std::vector<std::string>& args, const std::map<int,shared_ptr<IfcPPEntity> >& map )
{
	const int num_args = (int)args.size();
	if( num_args<2 ){ std::stringstream strserr; strserr << "Wrong parameter count for entity IfcClothoid, expecting 2, having " << num_args << ". Object id: " << getId() << std::endl; throw IfcPPException( strserr.str().c_str() ); }
	#ifdef _DEBUG
	if( num_args>2 ){ std::cout << "Wrong parameter count for entity IfcClothoid, expecting 2, having " << num_args << ". Object id: " << getId() << std::endl; }
	#endif
	m_Position = IfcAxis2Placement::readStepData( args[0], map );
	m_ClothoidConstant = IfcLengthMeasure::readStepData( args[1] );
}
void IfcClothoid::setInverseCounterparts( shared_ptr<IfcPPEntity> ptr_self_entity )
{
	IfcCurve::setInverseCounterparts( ptr_self_entity );
}
void IfcClothoid::unlinkSelf()
{
	IfcCurve::unlinkSelf();
}
