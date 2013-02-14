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
#include "IfcAppliedValueSelect.h"
#include "IfcMetricValueSelect.h"
class IfcIdentifier;
class IfcLabel;
class IfcReference;
//ENTITY
class IfcReference : public IfcAppliedValueSelect, public IfcMetricValueSelect, public IfcPPEntity
{
public:
	IfcReference();
	IfcReference( int id );
	~IfcReference();

	// method setEntity takes over all attributes from another instance of the class
	virtual void setEntity( shared_ptr<IfcPPEntity> other );
	virtual void getStepLine( std::stringstream& stream ) const;
	virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
	virtual void readStepData( std::vector<std::string>& args, const std::map<int,shared_ptr<IfcPPEntity> >& map );
	virtual void setInverseCounterparts( shared_ptr<IfcPPEntity> ptr_self );
	virtual void unlinkSelf();
	virtual const char* classname() const { return "IfcReference"; }


	// IfcReference -----------------------------------------------------------
	// attributes:
	shared_ptr<IfcIdentifier>					m_TypeIdentifier;			//optional
	shared_ptr<IfcIdentifier>					m_AttributeIdentifier;		//optional
	shared_ptr<IfcLabel>							m_InstanceName;				//optional
	std::vector<int >							m_ListPositions;			//optional
	shared_ptr<IfcReference>						m_InnerReference;			//optional
};

