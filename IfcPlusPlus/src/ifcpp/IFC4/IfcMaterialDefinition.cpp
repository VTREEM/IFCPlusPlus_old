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
#include "include/IfcExternalReferenceRelationship.h"
#include "include/IfcMaterialDefinition.h"
#include "include/IfcMaterialProperties.h"
#include "include/IfcRelAssociatesMaterial.h"

// ENTITY IfcMaterialDefinition 
IfcMaterialDefinition::IfcMaterialDefinition() {}
IfcMaterialDefinition::IfcMaterialDefinition( int id ) { m_id = id; }
IfcMaterialDefinition::~IfcMaterialDefinition() {}

// method setEntity takes over all attributes from another instance of the class
void IfcMaterialDefinition::setEntity( shared_ptr<IfcPPEntity> other_entity )
{
	shared_ptr<IfcMaterialDefinition> other = dynamic_pointer_cast<IfcMaterialDefinition>(other_entity);
	if( !other) { return; }
}
void IfcMaterialDefinition::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_id << "=IFCMATERIALDEFINITION" << "(";
	stream << ");";
}
void IfcMaterialDefinition::getStepParameter( std::stringstream& stream, bool ) const { stream << "#" << m_id; }
void IfcMaterialDefinition::readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<IfcPPEntity> >& map )
{
}
void IfcMaterialDefinition::setInverseCounterparts( shared_ptr<IfcPPEntity> )
{
}
void IfcMaterialDefinition::unlinkSelf()
{
}
