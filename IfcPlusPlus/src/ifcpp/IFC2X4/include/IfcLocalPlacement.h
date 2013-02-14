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
#pragma once
#include <vector>
#include <map>
#include <sstream>
#include <string>
#include "ifcpp/model/shared_ptr.h"
#include "ifcpp/model/IfcPPObject.h"
#include "IfcObjectPlacement.h"
class IfcObjectPlacement;
class IfcAxis2Placement;
//ENTITY
class IfcLocalPlacement : public IfcObjectPlacement
{
public:
	IfcLocalPlacement();
	IfcLocalPlacement( int id );
	~IfcLocalPlacement();

	// method setEntity takes over all attributes from another instance of the class
	virtual void setEntity( shared_ptr<IfcPPEntity> other );
	virtual void getStepLine( std::stringstream& stream ) const;
	virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
	virtual void readStepData( std::vector<std::string>& args, const std::map<int,shared_ptr<IfcPPEntity> >& map );
	virtual void setInverseCounterparts( shared_ptr<IfcPPEntity> ptr_self );
	virtual void unlinkSelf();
	virtual const char* classname() const { return "IfcLocalPlacement"; }


	// IfcObjectPlacement -----------------------------------------------------------
	// inverse attributes:
	//  std::vector<weak_ptr<IfcProduct> >			m_PlacesObject_inverse;
	//  std::vector<weak_ptr<IfcLocalPlacement> >	m_ReferencedByPlacements_inverse;

	// IfcLocalPlacement -----------------------------------------------------------
	// attributes:
	shared_ptr<IfcObjectPlacement>				m_PlacementRelTo;			//optional
	shared_ptr<IfcAxis2Placement>				m_RelativePlacement;
};

