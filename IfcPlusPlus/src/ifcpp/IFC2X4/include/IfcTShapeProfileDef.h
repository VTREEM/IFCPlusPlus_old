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
#include "IfcParameterizedProfileDef.h"
class IfcPositiveLengthMeasure;
class IfcNonNegativeLengthMeasure;
class IfcPlaneAngleMeasure;
//ENTITY
class IfcTShapeProfileDef : public IfcParameterizedProfileDef
{
public:
	IfcTShapeProfileDef();
	IfcTShapeProfileDef( int id );
	~IfcTShapeProfileDef();

	// method setEntity takes over all attributes from another instance of the class
	virtual void setEntity( shared_ptr<IfcPPEntity> other );
	virtual void getStepLine( std::stringstream& stream ) const;
	virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
	virtual void readStepData( std::vector<std::string>& args, const std::map<int,shared_ptr<IfcPPEntity> >& map );
	virtual void setInverseCounterparts( shared_ptr<IfcPPEntity> ptr_self );
	virtual void unlinkSelf();
	virtual const char* classname() const { return "IfcTShapeProfileDef"; }


	// IfcProfileDef -----------------------------------------------------------
	// attributes:
	//  shared_ptr<IfcProfileTypeEnum>				m_ProfileType;
	//  shared_ptr<IfcLabel>							m_ProfileName;				//optional
	// inverse attributes:
	//  std::vector<weak_ptr<IfcExternalReferenceRelationship> >	m_HasExternalReference_inverse;
	//  std::vector<weak_ptr<IfcProfileProperties> >	m_HasProperties_inverse;

	// IfcParameterizedProfileDef -----------------------------------------------------------
	// attributes:
	//  shared_ptr<IfcAxis2Placement2D>				m_Position;					//optional

	// IfcTShapeProfileDef -----------------------------------------------------------
	// attributes:
	shared_ptr<IfcPositiveLengthMeasure>			m_Depth;
	shared_ptr<IfcPositiveLengthMeasure>			m_FlangeWidth;
	shared_ptr<IfcPositiveLengthMeasure>			m_WebThickness;
	shared_ptr<IfcPositiveLengthMeasure>			m_FlangeThickness;
	shared_ptr<IfcNonNegativeLengthMeasure>		m_FilletRadius;				//optional
	shared_ptr<IfcNonNegativeLengthMeasure>		m_FlangeEdgeRadius;			//optional
	shared_ptr<IfcNonNegativeLengthMeasure>		m_WebEdgeRadius;			//optional
	shared_ptr<IfcPlaneAngleMeasure>				m_WebSlope;					//optional
	shared_ptr<IfcPlaneAngleMeasure>				m_FlangeSlope;				//optional
};

