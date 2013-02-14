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
#include "include/IfcProduct.h"
#include "include/IfcReferenceCurve.h"
#include "include/IfcReferenceCurvePlacement.h"
#include "include/IfcReferenceCurvePlacementSystem.h"

// ENTITY IfcReferenceCurvePlacementSystem 
IfcReferenceCurvePlacementSystem::IfcReferenceCurvePlacementSystem() { m_entity_enum = IFCREFERENCECURVEPLACEMENTSYSTEM; }
IfcReferenceCurvePlacementSystem::IfcReferenceCurvePlacementSystem( int id ) { m_id = id; m_entity_enum = IFCREFERENCECURVEPLACEMENTSYSTEM; }
IfcReferenceCurvePlacementSystem::~IfcReferenceCurvePlacementSystem() {}

// method setEntity takes over all attributes from another instance of the class
void IfcReferenceCurvePlacementSystem::setEntity( shared_ptr<IfcPPEntity> other_entity )
{
	shared_ptr<IfcReferenceCurvePlacementSystem> other = dynamic_pointer_cast<IfcReferenceCurvePlacementSystem>(other_entity);
	if( !other) { return; }
	m_Label = other->m_Label;
	m_BasedOn = other->m_BasedOn;
}
void IfcReferenceCurvePlacementSystem::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_id << "=IFCREFERENCECURVEPLACEMENTSYSTEM" << "(";
	stream << encodeStepString( m_Label );
	stream << ",";
	if( m_BasedOn ) { stream << "#" << m_BasedOn->getId(); } else { stream << "$"; }
	stream << ");";
}
void IfcReferenceCurvePlacementSystem::getStepParameter( std::stringstream& stream, bool ) const { stream << "#" << m_id; }
void IfcReferenceCurvePlacementSystem::readStepData( std::vector<std::string>& args, const std::map<int,shared_ptr<IfcPPEntity> >& map )
{
	const int num_args = (int)args.size();
	if( num_args<2 ){ std::stringstream strserr; strserr << "Wrong parameter count for entity IfcReferenceCurvePlacementSystem, expecting 2, having " << num_args << ". Object id: " << getId() << std::endl; throw IfcPPException( strserr.str().c_str() ); }
	#ifdef _DEBUG
	if( num_args>2 ){ std::cout << "Wrong parameter count for entity IfcReferenceCurvePlacementSystem, expecting 2, having " << num_args << ". Object id: " << getId() << std::endl; }
	#endif
	m_Label = args[0];
	readEntityReference( args[1], m_BasedOn, map );
}
void IfcReferenceCurvePlacementSystem::setInverseCounterparts( shared_ptr<IfcPPEntity> ptr_self_entity )
{
	IfcReferencePlacement::setInverseCounterparts( ptr_self_entity );
	shared_ptr<IfcReferenceCurvePlacementSystem> ptr_self = dynamic_pointer_cast<IfcReferenceCurvePlacementSystem>( ptr_self_entity );
	if( !ptr_self ) { throw IfcPPException( "IfcReferenceCurvePlacementSystem::setInverseCounterparts: type mismatch" ); }
	if( m_BasedOn )
	{
		m_BasedOn->m_HasPlacements_inverse.push_back( ptr_self );
	}
}
void IfcReferenceCurvePlacementSystem::unlinkSelf()
{
	IfcReferencePlacement::unlinkSelf();
	if( m_BasedOn )
	{
		std::vector<weak_ptr<IfcReferenceCurvePlacementSystem> >& HasPlacements_inverse = m_BasedOn->m_HasPlacements_inverse;
		std::vector<weak_ptr<IfcReferenceCurvePlacementSystem> >::iterator it_HasPlacements_inverse;
		for( it_HasPlacements_inverse = HasPlacements_inverse.begin(); it_HasPlacements_inverse != HasPlacements_inverse.end(); ++it_HasPlacements_inverse)
		{
			shared_ptr<IfcReferenceCurvePlacementSystem> self_candidate( *it_HasPlacements_inverse );
			if( self_candidate->getId() == this->getId() )
			{
				HasPlacements_inverse.erase( it_HasPlacements_inverse );
				break;
			}
		}
	}
}
