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
#include "include/IfcLocalPlacement.h"
#include "include/IfcObjectPlacement.h"
#include "include/IfcProduct.h"

// ENTITY IfcObjectPlacement 
IfcObjectPlacement::IfcObjectPlacement() {}
IfcObjectPlacement::IfcObjectPlacement( int id ) { m_id = id; }
IfcObjectPlacement::~IfcObjectPlacement() {}

// method setEntity takes over all attributes from another instance of the class
void IfcObjectPlacement::setEntity( shared_ptr<IfcPPEntity> other_entity )
{
	shared_ptr<IfcObjectPlacement> other = dynamic_pointer_cast<IfcObjectPlacement>(other_entity);
	if( !other) { return; }
}
void IfcObjectPlacement::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_id << "=IFCOBJECTPLACEMENT" << "(";
	stream << ");";
}
void IfcObjectPlacement::getStepParameter( std::stringstream& stream, bool ) const { stream << "#" << m_id; }
void IfcObjectPlacement::readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<IfcPPEntity> >& map )
{
}
void IfcObjectPlacement::setInverseCounterparts( shared_ptr<IfcPPEntity> )
{
}
void IfcObjectPlacement::unlinkSelf()
{
}
