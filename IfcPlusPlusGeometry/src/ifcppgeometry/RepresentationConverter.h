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
//! @date 2011-07-18

#pragma once

#include <set>
#include <osg/ref_ptr>
#include <osg/Group>
#include <osgViewer/View>

#include <carve/matrix.hpp>
#include <carve/geom2d.hpp>
#include <carve/geom3d.hpp>
#include <carve/input.hpp>
#include <ifcpp/model/shared_ptr.h>
#include "GeometryInputData.h"

#ifdef IFCPP_OPENMP
#include <omp.h>
#endif

class GeometrySettings;
class UnitConverter;
class StylesConverter;
class CurveConverter;
class SolidModelConverter;
class FaceConverter;
class ProfileCache;

class IfcProduct;
class IfcRepresentation;
class IfcRepresentationItem;
class IfcGeometricRepresentationItem;
class IfcSectionedSpine;
class IfcReferencedSectionedSpine;
class IfcPropertySet;

class RepresentationConverter
{
public:
	RepresentationConverter( shared_ptr<GeometrySettings> geom_settings, shared_ptr<UnitConverter> unit_converter );
	~RepresentationConverter();

	void convertIfcRepresentation(				const shared_ptr<IfcRepresentation>& representation,			const carve::math::Matrix& pos,		shared_ptr<RepresentationData>& representation_data, std::set<int>& visited );
	void convertIfcGeometricRepresentationItem(	const shared_ptr<IfcGeometricRepresentationItem>& item,			const carve::math::Matrix& pos,		shared_ptr<ItemData> item_data );
	void convertIfcSectionedSpine(				const shared_ptr<IfcSectionedSpine>& spine,						const carve::math::Matrix& pos,		shared_ptr<ItemData> item_data );
	void convertIfcReferencedSectionedSpine(	const shared_ptr<IfcReferencedSectionedSpine>& spine,			const carve::math::Matrix& pos,		shared_ptr<ItemData> item_data );
	void convertIfcPropertySet(					const shared_ptr<IfcPropertySet>& property_set,	osg::Group* group );
	void convertStyledItem(						const shared_ptr<IfcRepresentationItem>& representation_item, shared_ptr<ItemData>& item_data );
	void subtractOpenings(						const shared_ptr<IfcProduct>& ifc_product, shared_ptr<ItemData>& item_data, osg::Group* item_group, std::stringstream& strs_err );

	shared_ptr<SolidModelConverter>& getSolidConverter() { return m_solid_converter; }
	shared_ptr<ProfileCache>& getProfileCache() { return m_profile_cache; }
	std::string getDetailedReport() { return m_detailed_report.str(); }
	void detailedReport( std::stringstream& strs );
	bool handleLayerAssignments() { return m_handle_layer_assignments; }
	void setHandleLayerAssignments( bool handle ) { m_handle_layer_assignments = handle; }
	bool handleStyledItems() { return m_handle_styled_items; }
	void setHandleStyledItems( bool handle ) { m_handle_styled_items = handle; }

	osgViewer::View* m_debug_view;

protected:
	shared_ptr<GeometrySettings>				m_geom_settings;
	shared_ptr<UnitConverter>					m_unit_converter;
	shared_ptr<StylesConverter>					m_styles_converter;
	shared_ptr<CurveConverter>					m_curve_converter;
	shared_ptr<SolidModelConverter>				m_solid_converter;
	shared_ptr<FaceConverter>					m_face_converter;
	shared_ptr<ProfileCache>					m_profile_cache;
	
	std::stringstream							m_detailed_report;
	bool										m_handle_styled_items;
	bool										m_handle_layer_assignments;
	
#ifdef IFCPP_OPENMP
	omp_lock_t m_writelock_detailed_report;
	omp_lock_t m_writelock_styles_converter;
#endif
};
