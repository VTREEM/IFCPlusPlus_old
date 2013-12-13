/* -*-c++-*- IfcPlusPlus - www.ifcplusplus.com  - Copyright (C) 2011 Fabian Gerold
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

//! @author Fabian Gerold
//! @date 2013-12-06


#pragma once

#include <osgViewer/View>
#include <carve/matrix.hpp>
#include <ifcpp/model/shared_ptr.h>
#include "GeometryInputData.h"

#ifdef IFCPP_OPENMP
#include <omp.h>
#endif

class GeometrySettings;
class UnitConverter;
class ProfileConverter;
class ProfileCache;
class FaceConverter;
class CurveConverter;

class IfcExtrudedAreaSolid;
class IfcSolidModel;
class IfcBooleanResult;
class IfcBooleanOperand;
class IfcRevolvedAreaSolid;
class IfcCsgPrimitive3D;

class SolidModelConverter
{
public:
	SolidModelConverter( shared_ptr<GeometrySettings> settings, shared_ptr<UnitConverter> uc, shared_ptr<CurveConverter>	cc, shared_ptr<FaceConverter> fc, shared_ptr<ProfileCache>	pc );
	~SolidModelConverter();

	void convertIfcBooleanResult(		const shared_ptr<IfcBooleanResult>& operand,				const carve::math::Matrix& pos,		shared_ptr<ItemData> item_data );
	void convertIfcBooleanOperand(		const shared_ptr<IfcBooleanOperand>& operand,				const carve::math::Matrix& pos,		shared_ptr<ItemData> item_data );
	void convertIfcSolidModel(			const shared_ptr<IfcSolidModel>& solid_model,				const carve::math::Matrix& pos,		shared_ptr<ItemData> item_data );
	void convertIfcExtrudedAreaSolid(	const shared_ptr<IfcExtrudedAreaSolid>& extruded_area,		const carve::math::Matrix& pos,		shared_ptr<ItemData> item_data );
	void convertIfcRevolvedAreaSolid(	const shared_ptr<IfcRevolvedAreaSolid>& revolved_area,		const carve::math::Matrix& pos,		shared_ptr<ItemData> item_data );
	void convertIfcCsgPrimitive3D(		const shared_ptr<IfcCsgPrimitive3D>& csg_primitive,			const carve::math::Matrix& pos,		shared_ptr<ItemData> item_data );
	void detailedReport( std::stringstream& strs );

	osgViewer::View* m_debug_view;

protected:
	shared_ptr<GeometrySettings>			m_geom_settings;
	shared_ptr<UnitConverter>				m_unit_converter;
	shared_ptr<CurveConverter>				m_curve_converter;
	shared_ptr<FaceConverter>				m_face_converter;
	shared_ptr<ProfileCache>				m_profile_cache;
	std::stringstream						m_detailed_report;
	
#ifdef IFCPP_OPENMP
	omp_lock_t m_writelock_detailed_report;
#endif
};

