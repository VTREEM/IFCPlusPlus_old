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
class IfcAsymmetricIShapeProfileDef : public IfcParameterizedProfileDef
{
public:
	IfcAsymmetricIShapeProfileDef();
	IfcAsymmetricIShapeProfileDef( int id );
	~IfcAsymmetricIShapeProfileDef();

	// method setEntity takes over all attributes from another instance of the class
	virtual void setEntity( shared_ptr<IfcPPEntity> other );
	virtual void getStepLine( std::stringstream& stream ) const;
	virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
	virtual void readStepData( std::vector<std::string>& args, const std::map<int,shared_ptr<IfcPPEntity> >& map );
	virtual void setInverseCounterparts( shared_ptr<IfcPPEntity> ptr_self );
	virtual void unlinkSelf();
	virtual const char* classname() const { return "IfcAsymmetricIShapeProfileDef"; }


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

	// IfcAsymmetricIShapeProfileDef -----------------------------------------------------------
	// attributes:
	shared_ptr<IfcPositiveLengthMeasure>			m_BottomFlangeWidth;
	shared_ptr<IfcPositiveLengthMeasure>			m_OverallDepth;
	shared_ptr<IfcPositiveLengthMeasure>			m_WebThickness;
	shared_ptr<IfcPositiveLengthMeasure>			m_BottomFlangeThickness;
	shared_ptr<IfcNonNegativeLengthMeasure>		m_BottomFlangeFilletRadius;	//optional
	shared_ptr<IfcPositiveLengthMeasure>			m_TopFlangeWidth;
	shared_ptr<IfcPositiveLengthMeasure>			m_TopFlangeThickness;		//optional
	shared_ptr<IfcNonNegativeLengthMeasure>		m_TopFlangeFilletRadius;	//optional
	shared_ptr<IfcNonNegativeLengthMeasure>		m_BottomFlangeEdgeRadius;	//optional
	shared_ptr<IfcPlaneAngleMeasure>				m_BottomFlangeSlope;		//optional
	shared_ptr<IfcNonNegativeLengthMeasure>		m_TopFlangeEdgeRadius;		//optional
	shared_ptr<IfcPlaneAngleMeasure>				m_TopFlangeSlope;			//optional
};

