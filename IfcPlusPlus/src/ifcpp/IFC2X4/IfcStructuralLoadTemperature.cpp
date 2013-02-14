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
#include "include/IfcLabel.h"
#include "include/IfcStructuralLoadTemperature.h"
#include "include/IfcThermodynamicTemperatureMeasure.h"

// ENTITY IfcStructuralLoadTemperature 
IfcStructuralLoadTemperature::IfcStructuralLoadTemperature() { m_entity_enum = IFCSTRUCTURALLOADTEMPERATURE; }
IfcStructuralLoadTemperature::IfcStructuralLoadTemperature( int id ) { m_id = id; m_entity_enum = IFCSTRUCTURALLOADTEMPERATURE; }
IfcStructuralLoadTemperature::~IfcStructuralLoadTemperature() {}

// method setEntity takes over all attributes from another instance of the class
void IfcStructuralLoadTemperature::setEntity( shared_ptr<IfcPPEntity> other_entity )
{
	shared_ptr<IfcStructuralLoadTemperature> other = dynamic_pointer_cast<IfcStructuralLoadTemperature>(other_entity);
	if( !other) { return; }
	m_Name = other->m_Name;
	m_DeltaTConstant = other->m_DeltaTConstant;
	m_DeltaTY = other->m_DeltaTY;
	m_DeltaTZ = other->m_DeltaTZ;
}
void IfcStructuralLoadTemperature::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_id << "=IFCSTRUCTURALLOADTEMPERATURE" << "(";
	if( m_Name ) { m_Name->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_DeltaTConstant ) { m_DeltaTConstant->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_DeltaTY ) { m_DeltaTY->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_DeltaTZ ) { m_DeltaTZ->getStepParameter( stream ); } else { stream << "$"; }
	stream << ");";
}
void IfcStructuralLoadTemperature::getStepParameter( std::stringstream& stream, bool ) const { stream << "#" << m_id; }
void IfcStructuralLoadTemperature::readStepData( std::vector<std::string>& args, const std::map<int,shared_ptr<IfcPPEntity> >& map )
{
	const int num_args = (int)args.size();
	if( num_args<4 ){ std::stringstream strserr; strserr << "Wrong parameter count for entity IfcStructuralLoadTemperature, expecting 4, having " << num_args << ". Object id: " << getId() << std::endl; throw IfcPPException( strserr.str().c_str() ); }
	#ifdef _DEBUG
	if( num_args>4 ){ std::cout << "Wrong parameter count for entity IfcStructuralLoadTemperature, expecting 4, having " << num_args << ". Object id: " << getId() << std::endl; }
	#endif
	m_Name = IfcLabel::readStepData( args[0] );
	m_DeltaTConstant = IfcThermodynamicTemperatureMeasure::readStepData( args[1] );
	m_DeltaTY = IfcThermodynamicTemperatureMeasure::readStepData( args[2] );
	m_DeltaTZ = IfcThermodynamicTemperatureMeasure::readStepData( args[3] );
}
void IfcStructuralLoadTemperature::setInverseCounterparts( shared_ptr<IfcPPEntity> ptr_self_entity )
{
	IfcStructuralLoadStatic::setInverseCounterparts( ptr_self_entity );
}
void IfcStructuralLoadTemperature::unlinkSelf()
{
	IfcStructuralLoadStatic::unlinkSelf();
}
