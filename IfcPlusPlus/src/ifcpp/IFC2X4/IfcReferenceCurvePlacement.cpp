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
#include "include/IfcDirection.h"
#include "include/IfcLengthMeasure.h"
#include "include/IfcLocalPlacement.h"
#include "include/IfcProduct.h"
#include "include/IfcReferenceCurvePlacement.h"
#include "include/IfcReferenceCurvePlacementSystem.h"

// ENTITY IfcReferenceCurvePlacement 
IfcReferenceCurvePlacement::IfcReferenceCurvePlacement() { m_entity_enum = IFCREFERENCECURVEPLACEMENT; }
IfcReferenceCurvePlacement::IfcReferenceCurvePlacement( int id ) { m_id = id; m_entity_enum = IFCREFERENCECURVEPLACEMENT; }
IfcReferenceCurvePlacement::~IfcReferenceCurvePlacement() {}

// method setEntity takes over all attributes from another instance of the class
void IfcReferenceCurvePlacement::setEntity( shared_ptr<IfcPPEntity> other_entity )
{
	shared_ptr<IfcReferenceCurvePlacement> other = dynamic_pointer_cast<IfcReferenceCurvePlacement>(other_entity);
	if( !other) { return; }
	m_CurvilinearAbscissa = other->m_CurvilinearAbscissa;
	m_Axis = other->m_Axis;
	m_RefDirection = other->m_RefDirection;
	m_RelativeTo = other->m_RelativeTo;
}
void IfcReferenceCurvePlacement::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_id << "=IFCREFERENCECURVEPLACEMENT" << "(";
	if( m_CurvilinearAbscissa ) { m_CurvilinearAbscissa->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_Axis ) { stream << "#" << m_Axis->getId(); } else { stream << "$"; }
	stream << ",";
	if( m_RefDirection ) { stream << "#" << m_RefDirection->getId(); } else { stream << "$"; }
	stream << ",";
	if( m_RelativeTo ) { stream << "#" << m_RelativeTo->getId(); } else { stream << "$"; }
	stream << ");";
}
void IfcReferenceCurvePlacement::getStepParameter( std::stringstream& stream, bool ) const { stream << "#" << m_id; }
void IfcReferenceCurvePlacement::readStepData( std::vector<std::string>& args, const std::map<int,shared_ptr<IfcPPEntity> >& map )
{
	const int num_args = (int)args.size();
	if( num_args<4 ){ std::stringstream strserr; strserr << "Wrong parameter count for entity IfcReferenceCurvePlacement, expecting 4, having " << num_args << ". Object id: " << getId() << std::endl; throw IfcPPException( strserr.str().c_str() ); }
	#ifdef _DEBUG
	if( num_args>4 ){ std::cout << "Wrong parameter count for entity IfcReferenceCurvePlacement, expecting 4, having " << num_args << ". Object id: " << getId() << std::endl; }
	#endif
	m_CurvilinearAbscissa = IfcLengthMeasure::readStepData( args[0] );
	readEntityReference( args[1], m_Axis, map );
	readEntityReference( args[2], m_RefDirection, map );
	readEntityReference( args[3], m_RelativeTo, map );
}
void IfcReferenceCurvePlacement::setInverseCounterparts( shared_ptr<IfcPPEntity> ptr_self_entity )
{
	IfcReferencePlacement::setInverseCounterparts( ptr_self_entity );
	shared_ptr<IfcReferenceCurvePlacement> ptr_self = dynamic_pointer_cast<IfcReferenceCurvePlacement>( ptr_self_entity );
	if( !ptr_self ) { throw IfcPPException( "IfcReferenceCurvePlacement::setInverseCounterparts: type mismatch" ); }
	if( m_RelativeTo )
	{
		m_RelativeTo->m_Includes_inverse.push_back( ptr_self );
	}
}
void IfcReferenceCurvePlacement::unlinkSelf()
{
	IfcReferencePlacement::unlinkSelf();
	if( m_RelativeTo )
	{
		std::vector<weak_ptr<IfcReferenceCurvePlacement> >& Includes_inverse = m_RelativeTo->m_Includes_inverse;
		std::vector<weak_ptr<IfcReferenceCurvePlacement> >::iterator it_Includes_inverse;
		for( it_Includes_inverse = Includes_inverse.begin(); it_Includes_inverse != Includes_inverse.end(); ++it_Includes_inverse)
		{
			shared_ptr<IfcReferenceCurvePlacement> self_candidate( *it_Includes_inverse );
			if( self_candidate->getId() == this->getId() )
			{
				Includes_inverse.erase( it_Includes_inverse );
				break;
			}
		}
	}
}
