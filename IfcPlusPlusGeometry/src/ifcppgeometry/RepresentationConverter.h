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

#include <carve/matrix.hpp>
#include <carve/geom2d.hpp>
#include <carve/geom3d.hpp>
#include <carve/input.hpp>
#include <ifcpp/model/shared_ptr.h>

#include <osg/ref_ptr>
#include <osg/Array>
#include <osg/Geode>
#include <osg/Group>
#include <osgViewer/View>

#ifdef IFCPP_OPENMP
#include <omp.h>
#endif

class UnitConverter;
class StylesConverter;
class ProfileConverter;
class CurveConverter;
class IfcProfileDef;
class IfcRepresentation;
class IfcRepresentationItem;
class IfcGeometricRepresentationItem;
class IfcSite;
class IfcBuilding;
class IfcBuildingStorey;
class IfcFace;
class IfcSurface;
class IfcExtrudedAreaSolid;
class IfcPlacement;
class IfcObjectPlacement;
class IfcSolidModel;
class IfcBooleanResult;
class IfcBooleanOperand;
class IfcSectionedSpine;
class IfcReferencedSectionedSpine;
class IfcRevolvedAreaSolid;
class IfcCsgPrimitive3D;
class IfcCartesianPoint;
class IfcPropertySet;
class IfcRationalBSplineSurfaceWithKnots;

#define GEOM_TOLERANCE  0.0000001

class ItemData
{
public:
	std::vector<shared_ptr<carve::input::PolyhedronData> >	closed_shell_data;
	std::vector<shared_ptr<carve::input::PolyhedronData> >	open_shell_data;
	std::vector<shared_ptr<carve::input::PolyhedronData> >	open_or_closed_shell_data;
	std::vector<shared_ptr<carve::input::PolylineSetData> > polyline_data;
	std::vector<shared_ptr<carve::mesh::MeshSet<3> > >		polyhedrons;
	std::vector<osg::ref_ptr<osg::StateSet> >				statesets;
};

class RepresentationData
{
public:
	std::vector<shared_ptr<ItemData> >			vec_item_data;
	std::vector<osg::ref_ptr<osg::StateSet> >	statesets;
};

struct PlacementData
{
	std::set<int> placement_already_applied;
	carve::math::Matrix pos;
};

class RepresentationConverter
{
public:
	RepresentationConverter( shared_ptr<UnitConverter> unit_converter );
	~RepresentationConverter();

	void convertIfcRepresentation(			const shared_ptr<IfcRepresentation>& representation,				const carve::math::Matrix& pos,		shared_ptr<RepresentationData>& representation_data, std::set<int>& visited );
	void convertIfcGeometricRepresentationItem(	const shared_ptr<IfcGeometricRepresentationItem>& item,			const carve::math::Matrix& pos,		shared_ptr<ItemData> item_data );
	void convertIfcBooleanResult(			const shared_ptr<IfcBooleanResult>& operand,						const carve::math::Matrix& pos,		shared_ptr<ItemData> item_data );
	void convertIfcBooleanOperand(			const shared_ptr<IfcBooleanOperand>& operand,						const carve::math::Matrix& pos,		shared_ptr<ItemData> item_data );
	void convertIfcSolidModel(				const shared_ptr<IfcSolidModel>& solid_model,						const carve::math::Matrix& pos,		shared_ptr<ItemData> item_data );
	void convertIfcExtrudedAreaSolid(		const shared_ptr<IfcExtrudedAreaSolid>& extruded_area,				const carve::math::Matrix& pos,		shared_ptr<ItemData> item_data );
	void convertIfcSectionedSpine(			const shared_ptr<IfcSectionedSpine>& spine,							const carve::math::Matrix& pos,		shared_ptr<ItemData> item_data );
	void convertIfcReferencedSectionedSpine(const shared_ptr<IfcReferencedSectionedSpine>& spine,				const carve::math::Matrix& pos,		shared_ptr<ItemData> item_data );
	void convertIfcRevolvedAreaSolid(		const shared_ptr<IfcRevolvedAreaSolid>& revolved_area,				const carve::math::Matrix& pos,		shared_ptr<ItemData> item_data );
	void convertIfcCsgPrimitive3D(			const shared_ptr<IfcCsgPrimitive3D>& csg_primitive,					const carve::math::Matrix& pos,		shared_ptr<ItemData> item_data );
	void convertIfcFaceList(				const std::vector<shared_ptr<IfcFace> >& faces,						const carve::math::Matrix& pos,		shared_ptr<ItemData> item_data );
	void convertIfcSurface(					const shared_ptr<IfcSurface>& surface,								const carve::math::Matrix& pos,		shared_ptr<carve::input::PolylineSetData>& polyline_data );
	void convertIfcBSplineSurface(			const shared_ptr<IfcRationalBSplineSurfaceWithKnots>& ifc_surface,	const carve::math::Matrix& pos,		shared_ptr<carve::input::PolylineSetData>& polyline_data );
	void convertIfcPropertySet(				const shared_ptr<IfcPropertySet>& property_set,	osg::Group* group );
	void convertStyledItem(					const shared_ptr<IfcRepresentationItem>& representation_item, shared_ptr<ItemData>& item_data );

	shared_ptr<ProfileConverter> getProfileConverter( shared_ptr<IfcProfileDef>& ifc_profile );
	std::string getDetailedReport() { return m_detailed_report.str(); }
	void detailedReport( std::stringstream& strs );
	bool handleLayerAssignments() { return m_handle_layer_assignments; }
	void setHandleLayerAssignments( bool handle ) { m_handle_layer_assignments = handle; }
	bool handleStyledItems() { return m_handle_styled_items; }
	void setHandleStyledItems( bool handle ) { m_handle_styled_items = handle; }
	void setNumVerticesPerCircle( int num_vertices );

#ifdef _DEBUG
	osgViewer::View* m_debug_view;
	void renderPolyhedronInDebugViewer( shared_ptr<carve::poly::Polyhedron>& polyhedron, osg::Vec4f& color, bool wireframe );
#endif

protected:
	shared_ptr<UnitConverter>					m_unit_converter;
	shared_ptr<StylesConverter>					m_styles_converter;
	shared_ptr<CurveConverter>					m_curve_converter;
	std::map<int,shared_ptr<ProfileConverter> >	m_profile_cache;
	std::stringstream							m_detailed_report;
	bool										m_handle_styled_items;
	bool										m_handle_layer_assignments;
	int											m_num_vertices_per_circle;
	
#ifdef IFCPP_OPENMP
	omp_lock_t m_writelock_profile_cache;
	omp_lock_t m_writelock_detailed_report;
	omp_lock_t m_writelock_styles_converter;
#endif
};
