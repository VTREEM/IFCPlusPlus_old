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
#include <ifcpp/model/shared_ptr.h>

#include <osg/ref_ptr>
#include <osg/Array>

#ifdef IFCPP_OPENMP
#include <omp.h>
#endif

namespace osg
{
	class Vec3d;
	class Geode;
	class Geometry;
	class Group;
	class Matrixd;
	class StateSet;
}

class UnitConverter;
class StylesConverter;
class ProfileConverter;
class IfcProfileDef;
class IfcRepresentation;
class IfcRepresentationItem;
class IfcGeometricRepresentationItem;
class IfcSite;
class IfcBuilding;
class IfcBuildingStorey;
class IfcPolyline;
class IfcFace;
class IfcCurve;
class IfcSurface;
class IfcExtrudedAreaSolid;
class IfcPlacement;
class IfcObjectPlacement;
class IfcCartesianTransformationOperator;
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
class IfcTrimmingSelect;

namespace carve
{
	namespace input
	{
		struct PolyhedronData;
		struct PolylineSetData;
	}
	namespace mesh
	{
		template<unsigned int ndim> class MeshSet;
	}
	namespace poly
	{
		class Polyhedron;
		template<unsigned int ndim> class Face;
		template<unsigned int ndim>	class Vertex;
	}
	namespace geom
	{
		template<unsigned int ndim> struct vector;
	}
}

#define GEOM_TOLERANCE  0.0000001

class ItemData
{
public:
	std::vector<shared_ptr<carve::input::PolyhedronData> > closed_shell_data;
	std::vector<shared_ptr<carve::input::PolyhedronData> > open_shell_data;
	std::vector<shared_ptr<carve::input::PolyhedronData> > open_or_closed_shell_data;
	std::vector<shared_ptr<carve::input::PolylineSetData> > polyline_data;
	std::vector<shared_ptr<carve::mesh::MeshSet<3> > > polyhedrons;
	std::vector<osg::ref_ptr<osg::StateSet> > statesets;
};

class RepresentationData
{
public:
	std::vector<shared_ptr<ItemData> > vec_item_data;
	std::vector<osg::ref_ptr<osg::StateSet> > statesets;
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

	void convertIfcCurve( const shared_ptr<IfcCurve>& ifc_curve, std::vector<carve::geom::vector<3> >& loops, std::vector<carve::geom::vector<3> >& segment_start_points );
	void convertIfcCurve( const shared_ptr<IfcCurve>& ifc_curve, std::vector<carve::geom::vector<3> >& loops, std::vector<carve::geom::vector<3> >& segment_start_points,
		std::vector<shared_ptr<IfcTrimmingSelect> >& trim1_vec, std::vector<shared_ptr<IfcTrimmingSelect> >& trim2_vec, bool sense_agreement );
	void convertIfcPolyline( const shared_ptr<IfcPolyline>& poly_line,	std::vector<carve::geom::vector<3> >& loops );

	void convertIfcCartesianPoint(			const shared_ptr<IfcCartesianPoint>& ifc_point,				carve::geom::vector<3>& point );
	static void convertIfcCartesianPoint(	const shared_ptr<IfcCartesianPoint>& ifc_point,				carve::geom::vector<3>& point, double length_factor );
	void convertIfcCartesianPointVector(	const std::vector<shared_ptr<IfcCartesianPoint> >& points,	std::vector<carve::geom::vector<3> >& vertices );
	void convertIfcCartesianPointVectorSkipDuplicates( const std::vector<shared_ptr<IfcCartesianPoint> >& ifc_points, std::vector<carve::geom::vector<3> >& loop );
	
	static double getAngleOnCircle( const carve::geom::vector<3>& circle_center, double circle_radius, const carve::geom::vector<3>& trim_point );

	shared_ptr<ProfileConverter> getProfileConverter( shared_ptr<IfcProfileDef>& ifc_profile );
	void convertIfcPropertySet(	const shared_ptr<IfcPropertySet>& property_set,	osg::Group* group );
	void convertStyledItem( const shared_ptr<IfcRepresentationItem>& representation_item, shared_ptr<ItemData>& item_data );

	std::stringstream& getDetailedReport() { return m_detailed_report; }
	void detailedReport( std::stringstream& strs );
	bool handleLayerAssignments() { return m_handle_layer_assignments; }
	void setHandleLayerAssignments( bool handle ) { m_handle_layer_assignments = handle; }
	bool handleStyledItems() { return m_handle_styled_items; }
	void setHandleStyledItems( bool handle ) { m_handle_styled_items = handle; }

	// osg
	static double getAngleOnCircle( const osg::Vec3d& circle_center, double circle_radius, const osg::Vec3d& trim_point );
	static void convertIfcCartesianPoint( const shared_ptr<IfcCartesianPoint>& ifc_point,	osg::Vec3d& point, double length_factor );
	void convertIfcCartesianPointVector(    const std::vector<shared_ptr<IfcCartesianPoint> >& points,      osg::Vec3dArray* vertices );
	static void convertIfcCartesianPointVector(    const std::vector<shared_ptr<IfcCartesianPoint> >& points, osg::Vec3dArray* vertices, double length_factor );
	static bool bisectingPlane( osg::Vec3d& n, const osg::Vec3d& v1, const osg::Vec3d& v2, const osg::Vec3d& v3);

	osg::Group* getDebugGroupFirst() { return m_debug_group_first.get(); }
	osg::Group* getDebugGroupSecond() { return m_debug_group_second.get(); }

private:
	shared_ptr<UnitConverter>					m_unit_converter;
	shared_ptr<StylesConverter>					m_styles_converter;
	std::map<int,shared_ptr<ProfileConverter> >	m_profile_cache;
	std::stringstream							m_detailed_report;
	bool										m_handle_styled_items;
	bool										m_handle_layer_assignments;
	
	osg::ref_ptr<osg::Group> m_debug_group_first;
	osg::ref_ptr<osg::Group> m_debug_group_second;

#ifdef IFCPP_OPENMP
	omp_lock_t m_writelock_profile_cache;
	omp_lock_t m_writelock_detailed_report;
	omp_lock_t m_writelock_styles_converter;
#endif
};
