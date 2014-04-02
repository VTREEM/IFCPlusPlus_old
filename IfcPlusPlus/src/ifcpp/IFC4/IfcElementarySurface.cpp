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
#include "include/IfcElementarySurface.h"
#include "include/IfcPresentationLayerAssignment.h"
#include "include/IfcStyledItem.h"

// ENTITY IfcElementarySurface 
IfcElementarySurface::IfcElementarySurface() {}
IfcElementarySurface::IfcElementarySurface( int id ) { m_id = id; }
IfcElementarySurface::~IfcElementarySurface() {}

// method setEntity takes over all attributes from another instance of the class
void IfcElementarySurface::setEntity( shared_ptr<IfcPPEntity> other_entity )
{
	shared_ptr<IfcElementarySurface> other = dynamic_pointer_cast<IfcElementarySurface>(other_entity);
	if( !other) { return; }
	m_Position = other->m_Position;
}
void IfcElementarySurface::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_id << "=IFCELEMENTARYSURFACE" << "(";
	if( m_Position ) { stream << "#" << m_Position->getId(); } else { stream << "$"; }
	stream << ");";
}
void IfcElementarySurface::getStepParameter( std::stringstream& stream, bool ) const { stream << "#" << m_id; }
void IfcElementarySurface::readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<IfcPPEntity> >& map )
{
	const int num_args = (int)args.size();
	if( num_args<1 ){ std::stringstream strserr; strserr << "Wrong parameter count for entity IfcElementarySurface, expecting 1, having " << num_args << ". Object id: " << getId() << std::endl; throw IfcPPException( strserr.str().c_str() ); }
	#ifdef _DEBUG
	if( num_args>1 ){ std::cout << "Wrong parameter count for entity IfcElementarySurface, expecting 1, having " << num_args << ". Object id: " << getId() << std::endl; }
	#endif
	readEntityReference( args[0], m_Position, map );
}
void IfcElementarySurface::setInverseCounterparts( shared_ptr<IfcPPEntity> ptr_self_entity )
{
	IfcSurface::setInverseCounterparts( ptr_self_entity );
}
void IfcElementarySurface::unlinkSelf()
{
	IfcSurface::unlinkSelf();
}
