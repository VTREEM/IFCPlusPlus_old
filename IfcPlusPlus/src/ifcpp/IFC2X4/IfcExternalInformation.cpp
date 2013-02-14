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
#include "include/IfcExternalInformation.h"

// ENTITY IfcExternalInformation 
IfcExternalInformation::IfcExternalInformation() { m_entity_enum = IFCEXTERNALINFORMATION; }
IfcExternalInformation::IfcExternalInformation( int id ) { m_id = id; m_entity_enum = IFCEXTERNALINFORMATION; }
IfcExternalInformation::~IfcExternalInformation() {}

// method setEntity takes over all attributes from another instance of the class
void IfcExternalInformation::setEntity( shared_ptr<IfcPPEntity> other_entity )
{
	shared_ptr<IfcExternalInformation> other = dynamic_pointer_cast<IfcExternalInformation>(other_entity);
	if( !other) { return; }
}
void IfcExternalInformation::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_id << "=IFCEXTERNALINFORMATION" << "(";
	stream << ");";
}
void IfcExternalInformation::getStepParameter( std::stringstream& stream, bool ) const { stream << "#" << m_id; }
void IfcExternalInformation::readStepData( std::vector<std::string>& args, const std::map<int,shared_ptr<IfcPPEntity> >& map )
{
}
void IfcExternalInformation::setInverseCounterparts( shared_ptr<IfcPPEntity> )
{
}
void IfcExternalInformation::unlinkSelf()
{
}
