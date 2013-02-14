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
#include "IfcStructuralConnectionCondition.h"
class IfcForceMeasure;
//ENTITY
class IfcFailureConnectionCondition : public IfcStructuralConnectionCondition
{
public:
	IfcFailureConnectionCondition();
	IfcFailureConnectionCondition( int id );
	~IfcFailureConnectionCondition();

	// method setEntity takes over all attributes from another instance of the class
	virtual void setEntity( shared_ptr<IfcPPEntity> other );
	virtual void getStepLine( std::stringstream& stream ) const;
	virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
	virtual void readStepData( std::vector<std::string>& args, const std::map<int,shared_ptr<IfcPPEntity> >& map );
	virtual void setInverseCounterparts( shared_ptr<IfcPPEntity> ptr_self );
	virtual void unlinkSelf();
	virtual const char* classname() const { return "IfcFailureConnectionCondition"; }


	// IfcStructuralConnectionCondition -----------------------------------------------------------
	// attributes:
	//  shared_ptr<IfcLabel>							m_Name;						//optional

	// IfcFailureConnectionCondition -----------------------------------------------------------
	// attributes:
	shared_ptr<IfcForceMeasure>					m_TensionFailureX;			//optional
	shared_ptr<IfcForceMeasure>					m_TensionFailureY;			//optional
	shared_ptr<IfcForceMeasure>					m_TensionFailureZ;			//optional
	shared_ptr<IfcForceMeasure>					m_CompressionFailureX;		//optional
	shared_ptr<IfcForceMeasure>					m_CompressionFailureY;		//optional
	shared_ptr<IfcForceMeasure>					m_CompressionFailureZ;		//optional
};

