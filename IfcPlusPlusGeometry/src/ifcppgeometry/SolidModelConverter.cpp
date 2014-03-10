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

#include <vector>
#pragma warning (disable: 4267)
#include <carve/carve.hpp>
#include <carve/geom3d.hpp>
#include <carve/input.hpp>
#include <carve/csg_triangulator.hpp>
#include <carve/mesh_simplify.hpp>

#include "ifcpp/IFC4/include/IfcPositiveLengthMeasure.h"
#include "ifcpp/IFC4/include/IfcPlaneAngleMeasure.h"
#include "ifcpp/IFC4/include/IfcDirection.h"
#include "ifcpp/IFC4/include/IfcAxis1Placement.h"
#include "ifcpp/IFC4/include/IfcBoundingBox.h"
#include "ifcpp/IFC4/include/IfcBoundedCurve.h"

#include "ifcpp/IFC4/include/IfcExtrudedAreaSolid.h"
#include "ifcpp/IFC4/include/IfcFixedReferenceSweptAreaSolid.h"
#include "ifcpp/IFC4/include/IfcSurfaceCurveSweptAreaSolid.h"
#include "ifcpp/IFC4/include/IfcRevolvedAreaSolid.h"
#include "ifcpp/IFC4/include/IfcSweptDiskSolid.h"
#include "ifcpp/IFC4/include/IfcHalfSpaceSolid.h"
#include "ifcpp/IFC4/include/IfcBoxedHalfSpace.h"
#include "ifcpp/IFC4/include/IfcPolygonalBoundedHalfSpace.h"
#include "ifcpp/IFC4/include/IfcPlane.h"

#include "ifcpp/IFC4/include/IfcManifoldSolidBrep.h"
#include "ifcpp/IFC4/include/IfcClosedShell.h"
#include "ifcpp/IFC4/include/IfcFacetedBrep.h"
#include "ifcpp/IFC4/include/IfcAdvancedBrep.h"
#include "ifcpp/IFC4/include/IfcAdvancedBrepWithVoids.h"

#include "ifcpp/IFC4/include/IfcCsgSolid.h"
#include "ifcpp/IFC4/include/IfcBlock.h"
#include "ifcpp/IFC4/include/IfcSphere.h"
#include "ifcpp/IFC4/include/IfcRectangularPyramid.h"
#include "ifcpp/IFC4/include/IfcRightCircularCone.h"
#include "ifcpp/IFC4/include/IfcRightCircularCylinder.h"
#include "ifcpp/IFC4/include/IfcBooleanResult.h"
#include "ifcpp/IFC4/include/IfcBooleanOperator.h"
#include "ifcpp/IFC4/include/IfcBooleanOperand.h"
#include "ifcpp/IFC4/include/IfcBooleanClippingResult.h"

#include "ifcpp/model/UnitConverter.h"
#include "ifcpp/model/IfcPPException.h"
#include "GeometrySettings.h"
#include "ProfileConverter.h"
#include "ProfileCache.h"
#include "PlacementConverter.h"
#include "CurveConverter.h"
#include "FaceConverter.h"
#include "ConverterOSG.h"
#include "GeomUtils.h"
#include "DebugViewerCallback.h"
#include "UnhandledRepresentationException.h"
#include "RepresentationConverter.h"
#include "SolidModelConverter.h"

SolidModelConverter::SolidModelConverter( shared_ptr<GeometrySettings> geom_settings, shared_ptr<UnitConverter> uc, shared_ptr<CurveConverter>	cc, shared_ptr<FaceConverter> fc, shared_ptr<ProfileCache>	pc ) 
	: m_geom_settings(geom_settings), m_unit_converter(uc), m_curve_converter(cc), m_face_converter(fc), m_profile_cache( pc )
{
#ifdef IFCPP_OPENMP
	omp_init_lock(&m_writelock_detailed_report);
#endif

}

SolidModelConverter::~SolidModelConverter()
{
}

void SolidModelConverter::detailedReport( std::stringstream& strs )
{
#ifdef IFCPP_OPENMP
	omp_set_lock(&m_writelock_detailed_report);
	m_detailed_report << strs.str().c_str() << std::endl;
	omp_unset_lock(&m_writelock_detailed_report);
#else
	m_detailed_report << strs.str().c_str() << std::endl;
#endif
}

// ENTITY IfcSolidModel ABSTRACT SUPERTYPE OF(ONEOF(IfcCsgSolid, IfcManifoldSolidBrep, IfcSweptAreaSolid, IfcSweptDiskSolid))
void SolidModelConverter::convertIfcSolidModel( const shared_ptr<IfcSolidModel>& solid_model, const carve::math::Matrix& pos, shared_ptr<ItemData> item_data )
{
	shared_ptr<IfcSweptAreaSolid> swept_area_solid = dynamic_pointer_cast<IfcSweptAreaSolid>(solid_model);
	if( swept_area_solid )
	{
		//ENTITY IfcSweptAreaSolid
		//	ABSTRACT SUPERTYPE OF(ONEOF(IfcExtrudedAreaSolid, IfcFixedReferenceSweptAreaSolid, IfcRevolvedAreaSolid, IfcSurfaceCurveSweptAreaSolid))
		//	SUBTYPE OF IfcSolidModel;
		//	SweptArea	 :	IfcProfileDef;
		//	Position	 :	OPTIONAL IfcAxis2Placement3D;
		//	WHERE
		//	SweptAreaType	 :	SweptArea.ProfileType = IfcProfileTypeEnum.Area;
		//END_ENTITY;

		shared_ptr<IfcProfileDef>& swept_area = swept_area_solid->m_SweptArea;

		// check if local coordinate system is specified for extrusion
		carve::math::Matrix swept_area_pos( pos );
		if( swept_area_solid->m_Position )
		{
			double length_factor = m_unit_converter->getLengthInMeterFactor();
			shared_ptr<IfcAxis2Placement3D> swept_area_position = swept_area_solid->m_Position;
			PlacementConverter::convertIfcAxis2Placement3D( swept_area_position, swept_area_pos, length_factor );
			swept_area_pos = pos*swept_area_pos;
		}

		shared_ptr<IfcExtrudedAreaSolid> extruded_area = dynamic_pointer_cast<IfcExtrudedAreaSolid>(swept_area_solid);
		if( extruded_area )
		{
			convertIfcExtrudedAreaSolid( extruded_area, swept_area_pos, item_data );
			return;
		}

		shared_ptr<IfcFixedReferenceSweptAreaSolid> fixed_reference_swept_area_solid = dynamic_pointer_cast<IfcFixedReferenceSweptAreaSolid>(swept_area_solid);
		if( fixed_reference_swept_area_solid )
		{
			//Directrix	 : OPTIONAL IfcCurve;
			//StartParam	 : OPTIONAL IfcParameterValue;
			//EndParam	 : OPTIONAL IfcParameterValue;
			//FixedReference	 : IfcDirection;


			std::cout << "IfcFixedReferenceSweptAreaSolid not implemented" << std::endl;
			return;
		}

		shared_ptr<IfcRevolvedAreaSolid> revolved_area_solid = dynamic_pointer_cast<IfcRevolvedAreaSolid>(swept_area_solid);
		if( revolved_area_solid )
		{
			convertIfcRevolvedAreaSolid( revolved_area_solid, swept_area_pos, item_data );
			return;
		}

		shared_ptr<IfcSurfaceCurveSweptAreaSolid> surface_curve_swept_area_solid = dynamic_pointer_cast<IfcSurfaceCurveSweptAreaSolid>(swept_area_solid);
		if( surface_curve_swept_area_solid )
		{
			std::cout << "IfcSurfaceCurveSweptAreaSolid not implemented" << std::endl;
			// IfcSweptAreaSolid -----------------------------------------------------------
			// attributes:
			//  shared_ptr<IfcProfileDef>					m_SweptArea;
			//  shared_ptr<IfcAxis2Placement3D>				m_Position;					//optional

			shared_ptr<ProfileConverter> profile_converter = m_profile_cache->getProfileConverter( swept_area );
			const std::vector<std::vector<carve::geom::vector<2> > >& paths = profile_converter->getCoordinates();
			shared_ptr<carve::input::PolyhedronData> poly_data( new carve::input::PolyhedronData );


			shared_ptr<IfcCurve>& directrix_curve = surface_curve_swept_area_solid->m_Directrix;
			const int nvc = m_geom_settings->m_num_vertices_per_circle;
			double length_in_meter = m_unit_converter->getLengthInMeterFactor();

			std::vector<carve::geom::vector<3> > segment_start_points;
			std::vector<carve::geom::vector<3> > basis_curve_points;
			m_curve_converter->convertIfcCurve( directrix_curve, basis_curve_points, segment_start_points );

			shared_ptr<carve::input::PolylineSetData> polyline_data( new carve::input::PolylineSetData() );
			m_face_converter->convertIfcSurface( surface_curve_swept_area_solid->m_ReferenceSurface, swept_area_pos, polyline_data );


			//shared_ptr<IfcParameterValue>				m_StartParam;				//optional
			//shared_ptr<IfcParameterValue>				m_EndParam;					//optional



			return;
		}

		throw UnhandledRepresentationException( solid_model );
	}

	shared_ptr<IfcManifoldSolidBrep> manifold_solid_brep = dynamic_pointer_cast<IfcManifoldSolidBrep>(solid_model);	
	if( manifold_solid_brep )
	{
		//ENTITY IfcManifoldSolidBrep 
		//	ABSTRACT SUPERTYPE OF(ONEOF(IfcAdvancedBrep, IfcFacetedBrep))
		//	SUBTYPE OF IfcSolidModel;
		//		Outer	 :	IfcClosedShell;
		//END_ENTITY;

		shared_ptr<IfcClosedShell>& outer_shell = manifold_solid_brep->m_Outer;

		if( outer_shell )
		{
			// first convert outer shell
			std::vector<shared_ptr<IfcFace> >& vec_faces_outer_shell = outer_shell->m_CfsFaces;
			shared_ptr<ItemData> input_data_outer_shell( new ItemData() );
			m_face_converter->convertIfcFaceList( vec_faces_outer_shell, pos, input_data_outer_shell );
			std::copy( input_data_outer_shell->item_open_or_closed_mesh_data.begin(), input_data_outer_shell->item_open_or_closed_mesh_data.end(), std::back_inserter(item_data->item_closed_mesh_data) );
		}

		shared_ptr<IfcFacetedBrep> faceted_brep = dynamic_pointer_cast<IfcFacetedBrep>(manifold_solid_brep);
		if( faceted_brep )
		{
			// no additional attributes
			return;
		}

		shared_ptr<IfcAdvancedBrep> advanced_brep = dynamic_pointer_cast<IfcAdvancedBrep>(manifold_solid_brep);
		if( advanced_brep )
		{
			// ENTITY IfcAdvancedBrep	SUPERTYPE OF(IfcAdvancedBrepWithVoids)
			if( dynamic_pointer_cast<IfcAdvancedBrepWithVoids>(advanced_brep) )
			{
				shared_ptr<IfcAdvancedBrepWithVoids> advanced_brep_with_voids = dynamic_pointer_cast<IfcAdvancedBrepWithVoids>(solid_model);
				std::vector<shared_ptr<IfcClosedShell> >& vec_voids = advanced_brep_with_voids->m_Voids;

				// TODO: subtract voids from outer shell
				std::cout << "IfcAdvancedBrep not implemented" << std::endl;
			}
			return;
		}

		throw UnhandledRepresentationException( solid_model );
	}

	shared_ptr<IfcCsgSolid> csg_solid = dynamic_pointer_cast<IfcCsgSolid>(solid_model);
	if( csg_solid )
	{
		shared_ptr<IfcCsgSelect> csg_select = csg_solid->m_TreeRootExpression;

		if( dynamic_pointer_cast<IfcBooleanResult>(csg_select) )
		{
			shared_ptr<IfcBooleanResult> csg_select_boolean_result = dynamic_pointer_cast<IfcBooleanResult>(csg_select);
			convertIfcBooleanResult( csg_select_boolean_result, pos, item_data );
		}
		else if( dynamic_pointer_cast<IfcCsgPrimitive3D>(csg_select) )
		{
			shared_ptr<IfcCsgPrimitive3D> csg_select_primitive_3d = dynamic_pointer_cast<IfcCsgPrimitive3D>(csg_select);
			convertIfcCsgPrimitive3D( csg_select_primitive_3d, pos, item_data );
		}
		return;
	}

	//shared_ptr<IfcReferencedSectionedSpine> spine = dynamic_pointer_cast<IfcReferencedSectionedSpine>(solid_model);
	//if( spine )
	//{
	//	convertIfcReferencedSectionedSpine( spine, pos, item_data );
	//	return;
	//}

	shared_ptr<IfcSweptDiskSolid> swept_disp_solid = dynamic_pointer_cast<IfcSweptDiskSolid>(solid_model);
	if( swept_disp_solid )
	{
		//ENTITY IfcSweptDiskSolid;
		//	ENTITY IfcRepresentationItem;
		//	INVERSE
		//		LayerAssignments	 : 	SET OF IfcPresentationLayerAssignment FOR AssignedItems;
		//		StyledByItem	 : 	SET [0:1] OF IfcStyledItem FOR Item;
		//	ENTITY IfcGeometricRepresentationItem;
		//	ENTITY IfcSolidModel;
		//		DERIVE
		//		Dim	 : 	IfcDimensionCount :=  3;
		//	ENTITY IfcSweptDiskSolid;
		//		Directrix	 : 	IfcCurve;
		//		Radius	 : 	IfcPositiveLengthMeasure;
		//		InnerRadius	 : 	OPTIONAL IfcPositiveLengthMeasure;
		//		StartParam	 : 	OPTIONAL IfcParameterValue;
		//		EndParam	 : 	OPTIONAL IfcParameterValue;
		//END_ENTITY;	

		shared_ptr<IfcCurve>& directrix_curve = swept_disp_solid->m_Directrix;
		const int nvc = m_geom_settings->m_num_vertices_per_circle;
		double length_in_meter = m_unit_converter->getLengthInMeterFactor();
		double radius = 0.0;
		if( swept_disp_solid->m_Radius )
		{
			radius = swept_disp_solid->m_Radius->m_value*length_in_meter;
		}

		double radius_inner = 0.0;
		if( swept_disp_solid->m_InnerRadius )
		{
			radius_inner = swept_disp_solid->m_InnerRadius->m_value*length_in_meter;
		}

		// TODO: handle inner radius, start param, end param

		std::vector<carve::geom::vector<3> > segment_start_points;
		std::vector<carve::geom::vector<3> > basis_curve_points;
		m_curve_converter->convertIfcCurve( directrix_curve, basis_curve_points, segment_start_points );

		shared_ptr<carve::input::PolyhedronData> pipe_data( new carve::input::PolyhedronData() );
		item_data->item_closed_mesh_data.push_back(pipe_data);
		std::vector<carve::geom::vector<3> > inner_shape_points;

		double angle = 0;
		double delta_angle = 2.0*M_PI/double(nvc);	// TODO: adapt to model size and complexity
		std::vector<carve::geom::vector<3> > circle_points;
		std::vector<carve::geom::vector<3> > circle_points_inner;
		for( int i = 0; i < nvc; ++i )
		{
			// cross section (circle) is defined in YZ plane
			double x = sin(angle);
			double y = cos(angle);
			circle_points.push_back(		carve::geom::VECTOR( 0.0,	x*radius,		y*radius) );
			circle_points_inner.push_back(	carve::geom::VECTOR( 0.0,	x*radius_inner,	y*radius_inner) );
			angle += delta_angle;
		}

		int num_base_points = basis_curve_points.size();
		carve::math::Matrix matrix_sweep;

		carve::geom::vector<3> local_z( carve::geom::VECTOR( 0, 0, 1 ) );

		if( num_base_points < 2 )
		{
			std::cout << "IfcSweptDiskSolid: num curve points < 2" << std::endl;
			return;
		}

		bool bend_found = false;
		if( num_base_points > 3 )
		{
			// compute local z vector by dot product of the first bend of the reference line
			carve::geom::vector<3> vertex_back2 = basis_curve_points.at(0);
			carve::geom::vector<3> vertex_back1 = basis_curve_points.at(1);
			for( int i=2; i<num_base_points; ++i )
			{
				carve::geom::vector<3> vertex_current = basis_curve_points.at(i);
				carve::geom::vector<3> section1 = vertex_back1 - vertex_back2;
				carve::geom::vector<3> section2 = vertex_current - vertex_back1;
				section1.normalize();
				section2.normalize();

				double dot_product = dot( section1, section2 );
				double dot_product_abs = abs(dot_product);

				// if dot == 1 or -1, then points are colinear
				if( dot_product_abs < (1.0-0.0001) || dot_product_abs > (1.0+0.0001) )
				{
					// bend found, compute cross product
					carve::geom::vector<3> lateral_vec = cross( section1, section2 );
					local_z = cross( lateral_vec, section1 );
					local_z.normalize();
					bend_found = true;
					break;
				}
			}
		}

		if( !bend_found )
		{
			// sweeping curve is linear. assume any local z vector
			local_z = carve::geom::VECTOR( 0, 0, 1 );
			double dot_normal_local_z = dot( (basis_curve_points.at(1) - basis_curve_points.at(0)), local_z );
			if( abs(dot_normal_local_z) < 0.0001 )
			{
				local_z = carve::geom::VECTOR( 0, 1, 0 );
				local_z.normalize();
			}
		}

		for( int ii=0; ii<num_base_points; ++ii )
		{
			carve::geom::vector<3> vertex_current = basis_curve_points.at(ii);
			carve::geom::vector<3> vertex_next;
			carve::geom::vector<3> vertex_before;
			if( ii == 0 )
			{
				// first point
				vertex_next	= basis_curve_points.at(ii+1);
				carve::geom::vector<3> delta_element = vertex_next - vertex_current;
				vertex_before = vertex_current - (delta_element);
			}
			else if( ii == num_base_points-1 )
			{
				// last point
				vertex_before	= basis_curve_points.at(ii-1);
				carve::geom::vector<3> delta_element = vertex_current - vertex_before;
				vertex_next = vertex_before + (delta_element);
			}
			else
			{
				// inner point
				vertex_next		= basis_curve_points.at(ii+1);
				vertex_before	= basis_curve_points.at(ii-1);
			}

			carve::geom::vector<3> bisecting_normal;
			GeomUtils::bisectingPlane( vertex_before, vertex_current, vertex_next, bisecting_normal );

			carve::geom::vector<3> section1 = vertex_current - vertex_before;
			carve::geom::vector<3> section2 = vertex_next - vertex_current;
			section1.normalize();
			section2.normalize();
			double dot_product = dot( section1, section2 );
			double dot_product_abs = abs(dot_product);

			if( dot_product_abs < (1.0-0.0001) || dot_product_abs > (1.0+0.0001) )
			{
				// bend found, compute next local z vector
				carve::geom::vector<3> lateral_vec = cross( section1, section2 );
				local_z = cross( lateral_vec, section1 );
				local_z.normalize();
			}
			if( ii == num_base_points -1 )
			{
				bisecting_normal *= -1.0;
			}

			GeomUtils::convertPlane2Matrix( bisecting_normal, vertex_current, local_z, matrix_sweep );
			matrix_sweep = pos*matrix_sweep;

			for( int jj = 0; jj < nvc; ++jj )
			{
				carve::geom::vector<3> vertex = circle_points.at( jj );
				vertex = matrix_sweep*vertex;
				pipe_data->addVertex( vertex );
			}

			if( radius_inner > 0 )
			{
				for( int jj = 0; jj < nvc; ++jj )
				{
					carve::geom::vector<3> vertex = circle_points_inner.at( jj );
					vertex = matrix_sweep*vertex;
					inner_shape_points.push_back( vertex );
					//pipe_data->addVertex( vertex );
				}
			}
		}

		// outer shape
		size_t num_vertices_outer = pipe_data->getVertexCount();
		for( size_t i=0; i<num_base_points- 1; ++i )
		{
			int i_offset = i*nvc;
			int i_offset_next = (i+1)*nvc;
			for( int jj = 0; jj < nvc; ++jj )
			{
				int current_loop_pt = jj + i_offset;
				int current_loop_pt_next = (jj + 1)%nvc + i_offset;

				int next_loop_pt = jj + i_offset_next;
				int next_loop_pt_next = (jj + 1)%nvc + i_offset_next;
				pipe_data->addFace( current_loop_pt, next_loop_pt, next_loop_pt_next, current_loop_pt_next );  
			}
		}

		if( radius_inner > 0 )
		{
			if( inner_shape_points.size() != num_vertices_outer )
			{
				std::cout << "IfcSweptDiskSolid: inner_shape_points.size() != num_vertices_outer" << std::endl;
			}

			// add points for inner shape
			for( size_t i=0; i<inner_shape_points.size(); ++i )
			{
				pipe_data->addVertex( inner_shape_points[i] );
			}

			// faces of inner shape
			for( size_t i=0; i<num_base_points- 1; ++i )
			{
				int i_offset = i*nvc + num_vertices_outer;
				int i_offset_next = (i+1)*nvc + num_vertices_outer;
				for( int jj = 0; jj < nvc; ++jj )
				{
					int current_loop_pt = jj + i_offset;
					int current_loop_pt_next = (jj + 1)%nvc + i_offset;

					int next_loop_pt = jj + i_offset_next;
					int next_loop_pt_next = (jj + 1)%nvc + i_offset_next;
					//pipe_data->addFace( current_loop_pt, next_loop_pt, next_loop_pt_next, current_loop_pt_next );  
					pipe_data->addFace( current_loop_pt, current_loop_pt_next, next_loop_pt_next, next_loop_pt );  
				}
			}

			// front cap
			for( int jj = 0; jj < nvc; ++jj )
			{
				int outer_rim_next = (jj+1)%nvc;
				int inner_rim_next = outer_rim_next + num_vertices_outer;
				pipe_data->addFace( jj, outer_rim_next, num_vertices_outer + jj );
				pipe_data->addFace( outer_rim_next, inner_rim_next, num_vertices_outer + jj );
			}

			// back cap
			int back_offset = (num_base_points - 1)*nvc;
			for( int jj = 0; jj < nvc; ++jj )
			{
				int outer_rim_next = (jj+1)%nvc + back_offset;
				int inner_rim_next = outer_rim_next + num_vertices_outer;
				pipe_data->addFace( jj + back_offset, num_vertices_outer + jj + back_offset, outer_rim_next );
				pipe_data->addFace( outer_rim_next, num_vertices_outer + jj + back_offset, inner_rim_next );
			}
		}
		else
		{
			// front cap, full pipe, create triangle fan
			for( int jj = 0; jj < nvc - 2; ++jj )
			{
				pipe_data->addFace( 0, jj+1, jj+2 );
			}

			// back cap
			int back_offset = (num_base_points - 1)*nvc;
			for( int jj = 0; jj < nvc - 2; ++jj )
			{
				pipe_data->addFace( back_offset, back_offset + jj+2, back_offset + jj+1 );
			}
		}

#ifdef _DEBUG
		std::stringstream strs_err;
		bool poly_ok = ConverterOSG::checkMeshSet( shared_ptr<carve::mesh::MeshSet<3> >( pipe_data->createMesh(carve::input::opts()) ), strs_err, -1 );

		if( !poly_ok )
		{
			std::cout << strs_err.str().c_str() << std::endl;
		}
#endif

		return;
	}

	throw UnhandledRepresentationException( solid_model );
}

void SolidModelConverter::convertIfcExtrudedAreaSolid( const shared_ptr<IfcExtrudedAreaSolid>& extruded_area, const carve::math::Matrix& pos, shared_ptr<ItemData> item_data )
{
	if( !extruded_area->m_ExtrudedDirection )
	{
		throw IfcPPException( "Invalid ExtrudedDirection", __func__ );
		return;
	}

	if( !extruded_area->m_Depth )
	{
		throw IfcPPException( "Invalid Depth", __func__ );
		return;
	}
	double length_factor = m_unit_converter->getLengthInMeterFactor();

	// direction and length of extrusion
	const double depth = extruded_area->m_Depth->m_value*length_factor;
	carve::geom::vector<3>  extrusion_vector;
	std::vector<double>& vec_direction = extruded_area->m_ExtrudedDirection->m_DirectionRatios;
	if( vec_direction.size() > 2 )
	{
		extrusion_vector = carve::geom::VECTOR( vec_direction[0]*depth, vec_direction[1]*depth, vec_direction[2]*depth );
	}
	else if( vec_direction.size() > 1 )
	{
		extrusion_vector = carve::geom::VECTOR( vec_direction[0]*depth, vec_direction[1]*depth, 0 );
	}

	// swept area
	shared_ptr<IfcProfileDef>	swept_area = extruded_area->m_SweptArea;
	shared_ptr<ProfileConverter> profile_converter = m_profile_cache->getProfileConverter(swept_area);

	std::stringstream err;
	const std::vector<std::vector<carve::geom::vector<2> > >& paths = profile_converter->getCoordinates();

	if( paths.size() == 0 )
	{
		return;
	}
	shared_ptr<carve::input::PolyhedronData> poly_data( new carve::input::PolyhedronData );
	GeomUtils::extrude( paths, extrusion_vector, poly_data, err );

	// apply object coordinate system
	std::vector<carve::geom::vector<3> >& points = poly_data->points;
	for( std::vector<carve::geom::vector<3> >::iterator it_points = points.begin(); it_points != points.end(); ++it_points )
	{
		carve::geom::vector<3>& vertex = (*it_points);
		vertex = pos*vertex;
	}

	item_data->item_closed_mesh_data.push_back( poly_data );

#ifdef _DEBUG
	std::stringstream strs_err;
	shared_ptr<carve::mesh::MeshSet<3> > mesh_set( poly_data->createMesh(carve::input::opts()) );
	bool poly_ok = ConverterOSG::checkMeshSet( mesh_set, strs_err, -1 );

	if( !poly_ok )
	{
		std::cout << strs_err.str().c_str() << std::endl;

		renderMeshsetInDebugViewer( mesh_set, osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f), true );
		ConverterOSG::dumpMeshsets( mesh_set, shared_ptr<carve::mesh::MeshSet<3> >(), shared_ptr<carve::mesh::MeshSet<3> >() );

		shared_ptr<carve::input::PolylineSetData> polyline_data( new carve::input::PolylineSetData() );
		polyline_data->beginPolyline();
		if( paths.size() > 0 )
		{
			const std::vector<carve::geom::vector<2> >& loop = paths[0]; 
			for( int i=0; i<loop.size(); ++i )
			{
				const carve::geom::vector<2>& point = loop.at(i);
				carve::geom::vector<3> point3d( carve::geom::VECTOR( point.x, point.y, 0 ) );
				polyline_data->addVertex( point3d );
				polyline_data->addPolylineIndex(i);
			}

			osg::ref_ptr<osg::Geode> geode = new osg::Geode();
			ConverterOSG::drawPolyline( polyline_data, geode );

			renderPolylineInDebugViewer( polyline_data, osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f) );
		}

	}
#endif

	if( err.tellp() > 0 )
	{
		throw IfcPPException( err.str().c_str(), __func__ );
	}
}


void SolidModelConverter::convertIfcCsgPrimitive3D(	const shared_ptr<IfcCsgPrimitive3D>& csg_primitive,	const carve::math::Matrix& pos, shared_ptr<ItemData> item_data )
{
	shared_ptr<carve::input::PolyhedronData> polyhedron_data( new carve::input::PolyhedronData() );
	double length_factor = m_unit_converter->getLengthInMeterFactor();

	// ENTITY IfcCsgPrimitive3D  ABSTRACT SUPERTYPE OF(ONEOF(IfcBlock, IfcRectangularPyramid, IfcRightCircularCone, IfcRightCircularCylinder, IfcSphere
	shared_ptr<IfcAxis2Placement3D>& primitive_placement = csg_primitive->m_Position;

	carve::math::Matrix primitive_placement_matrix( pos );
	if( primitive_placement )
	{
		PlacementConverter::convertIfcAxis2Placement3D( primitive_placement, primitive_placement_matrix, length_factor );
		primitive_placement_matrix = pos*primitive_placement_matrix;
	}

	shared_ptr<IfcBlock> block = dynamic_pointer_cast<IfcBlock>(csg_primitive);
	if( block )
	{
		double x_length = length_factor;
		double y_length = length_factor;
		double z_length = length_factor;

		if( block->m_XLength )
		{
			x_length = block->m_XLength->m_value*0.5*length_factor;
		}
		if( block->m_YLength )
		{
			y_length = block->m_YLength->m_value*0.5*length_factor;
		}
		if( block->m_ZLength )
		{
			z_length = block->m_ZLength->m_value*0.5*length_factor;
		}

		polyhedron_data->addVertex( primitive_placement_matrix*carve::geom::VECTOR( x_length, y_length, z_length));
		polyhedron_data->addVertex( primitive_placement_matrix*carve::geom::VECTOR(-x_length, y_length, z_length));
		polyhedron_data->addVertex( primitive_placement_matrix*carve::geom::VECTOR(-x_length,-y_length, z_length));
		polyhedron_data->addVertex( primitive_placement_matrix*carve::geom::VECTOR( x_length,-y_length, z_length));
		polyhedron_data->addVertex( primitive_placement_matrix*carve::geom::VECTOR( x_length, y_length,-z_length));
		polyhedron_data->addVertex( primitive_placement_matrix*carve::geom::VECTOR(-x_length, y_length,-z_length));
		polyhedron_data->addVertex( primitive_placement_matrix*carve::geom::VECTOR(-x_length,-y_length,-z_length));
		polyhedron_data->addVertex( primitive_placement_matrix*carve::geom::VECTOR( x_length,-y_length,-z_length));

		polyhedron_data->addFace(0,	1, 2, 3);
		polyhedron_data->addFace(7, 6, 5, 4);
		polyhedron_data->addFace(0,	4, 5, 1);
		polyhedron_data->addFace(1, 5, 6, 2);
		polyhedron_data->addFace(2, 6, 7, 3);
		polyhedron_data->addFace(3, 7, 4, 0);

		item_data->item_closed_mesh_data.push_back( polyhedron_data );
		return;
	}

	shared_ptr<IfcRectangularPyramid> rectangular_pyramid = dynamic_pointer_cast<IfcRectangularPyramid>(csg_primitive);
	if( rectangular_pyramid )
	{
		double x_length = length_factor;
		double y_length = length_factor;
		double height = length_factor;

		if( rectangular_pyramid->m_XLength )
		{
			x_length = rectangular_pyramid->m_XLength->m_value*0.5*length_factor;
		}
		if( rectangular_pyramid->m_YLength )
		{
			y_length = rectangular_pyramid->m_YLength->m_value*0.5*length_factor;
		}
		if( rectangular_pyramid->m_Height )
		{
			height = rectangular_pyramid->m_Height->m_value*0.5*length_factor;
		}

		polyhedron_data->addVertex( primitive_placement_matrix*carve::geom::VECTOR(0, 0, height) );
		polyhedron_data->addVertex( primitive_placement_matrix*carve::geom::VECTOR( x_length,-y_length, 0.0) );
		polyhedron_data->addVertex( primitive_placement_matrix*carve::geom::VECTOR(-x_length,-y_length, 0.0) );
		polyhedron_data->addVertex( primitive_placement_matrix*carve::geom::VECTOR(-x_length, y_length, 0.0) );
		polyhedron_data->addVertex( primitive_placement_matrix*carve::geom::VECTOR( x_length, y_length, 0.0) );

		polyhedron_data->addFace(1,	2, 3, 4);
		polyhedron_data->addFace(0,	2, 1);
		polyhedron_data->addFace(0,	1, 4);
		polyhedron_data->addFace(0,	4, 3);
		polyhedron_data->addFace(0,	3, 2);

		item_data->item_closed_mesh_data.push_back( polyhedron_data );
		return;
	}

	shared_ptr<IfcRightCircularCone> right_circular_cone = dynamic_pointer_cast<IfcRightCircularCone>(csg_primitive);
	if( right_circular_cone )
	{
		if( !right_circular_cone->m_Height )
		{
			std::cout << "IfcRightCircularCone: height not given" << std::endl;
			return;
		}
		if( !right_circular_cone->m_BottomRadius )
		{
			std::cout << "IfcRightCircularCone: radius not given" << std::endl;
			return;
		}

		double height = right_circular_cone->m_Height->m_value*length_factor;
		double radius = right_circular_cone->m_BottomRadius->m_value*length_factor;

		polyhedron_data->addVertex( primitive_placement_matrix*carve::geom::VECTOR(0.0, 0.0, height) ); // top
		polyhedron_data->addVertex( primitive_placement_matrix*carve::geom::VECTOR(0.0, 0.0, 0.0) ); // bottom center

		double angle = 0;
		double d_angle = 2.0*M_PI/double(m_geom_settings->m_num_vertices_per_circle);	// TODO: adapt to model size and complexity
		for( int i = 0; i < m_geom_settings->m_num_vertices_per_circle; ++i )
		{
			polyhedron_data->addVertex( primitive_placement_matrix*carve::geom::VECTOR(sin(angle)*radius, cos(angle)*radius, 0.0) );
			angle += d_angle;
		}

		// outer shape
		for( int i = 0; i < m_geom_settings->m_num_vertices_per_circle-1; ++i )
		{
			polyhedron_data->addFace(0, i+3, i+2);
		}
		polyhedron_data->addFace( 0, 2, m_geom_settings->m_num_vertices_per_circle+1 );

		// bottom circle
		for( int i = 0; i < m_geom_settings->m_num_vertices_per_circle-1; ++i )
		{
			polyhedron_data->addFace(1, i+2, i+3 );
		}
		polyhedron_data->addFace(1, m_geom_settings->m_num_vertices_per_circle+1, 2 );

		item_data->item_closed_mesh_data.push_back( polyhedron_data );
		return;
	}

	shared_ptr<IfcRightCircularCylinder> right_circular_cylinder = dynamic_pointer_cast<IfcRightCircularCylinder>(csg_primitive);
	if( right_circular_cylinder )
	{
		if( !right_circular_cylinder->m_Height )
		{
			std::cout << "IfcRightCircularCylinder: height not given" << std::endl;
			return;
		}

		if( !right_circular_cylinder->m_Radius )
		{
			std::cout << "IfcRightCircularCylinder: radius not given" << std::endl;
			return;
		}

		int slices = m_geom_settings->m_num_vertices_per_circle;
		double rad = 0;

		//carve::mesh::MeshSet<3> * cylinder_mesh = makeCylinder( slices, rad, height, primitive_placement_matrix);
		double height = right_circular_cylinder->m_Height->m_value*length_factor;
		double radius = right_circular_cylinder->m_Radius->m_value*length_factor;

		double angle = 0;
		double d_angle = 2.0*M_PI/double(m_geom_settings->m_num_vertices_per_circle);	// TODO: adapt to model size and complexity
		for( int i = 0; i < m_geom_settings->m_num_vertices_per_circle; ++i )
		{
			polyhedron_data->addVertex( primitive_placement_matrix*carve::geom::VECTOR(sin(angle)*radius, cos(angle)*radius, height) );
			polyhedron_data->addVertex( primitive_placement_matrix*carve::geom::VECTOR(sin(angle)*radius, cos(angle)*radius, 0.0) );
			angle += d_angle;
		}

		for( int i = 0; i < m_geom_settings->m_num_vertices_per_circle-1; ++i )
		{
			polyhedron_data->addFace( 0, i*2+2, i*2+4 );		// top cap:		0-2-4	0-4-6		0-6-8
			polyhedron_data->addFace( 1, i*2+3, i*2+5 );		// bottom cap:	1-3-5	1-5-7		1-7-9
			polyhedron_data->addFace( i, i+1, i+3, i+2 );		// side
		}
		polyhedron_data->addFace( 2*m_geom_settings->m_num_vertices_per_circle-2, 2*m_geom_settings->m_num_vertices_per_circle-1, 1, 0 );		// side

		item_data->item_closed_mesh_data.push_back( polyhedron_data );
		return;
	}

	shared_ptr<IfcSphere> sphere = dynamic_pointer_cast<IfcSphere>(csg_primitive);
	if( sphere )
	{
		if( !sphere->m_Radius )
		{
			std::cout << "IfcSphere: radius not given" << std::endl;
			return;
		}

		double radius = sphere->m_Radius->m_value;

		//        \   |   /
		//         2- 1 -nvc
		//        / \ | / \
		//    ---3--- 0 ---7---
		//       \  / | \ /
		//         4- 5 -6
		//        /   |   \

		shared_ptr<carve::input::PolyhedronData> polyhedron_data( new carve::input::PolyhedronData() );
		polyhedron_data->addVertex( primitive_placement_matrix*carve::geom::VECTOR(0.0, 0.0, radius) ); // top

		const int nvc = m_geom_settings->m_num_vertices_per_circle;
		const int num_vertical_edges = nvc*0.5;
		double d_vertical_angle = M_PI/double(num_vertical_edges-1);	// TODO: adapt to model size and complexity
		double vertical_angle = d_vertical_angle;

		for( int vertical = 1; vertical < num_vertical_edges-1; ++vertical )
		{
			// for each vertical angle, add one horizontal circle
			double vertical_level = cos( vertical_angle )*radius;
			double radius_at_level = sin( vertical_angle )*radius;
			double horizontal_angle = 0;
			double d_horizontal_angle = 2.0*M_PI/double(nvc);
			for( int i = 0; i < nvc; ++i )
			{
				polyhedron_data->addVertex( primitive_placement_matrix*carve::geom::VECTOR(sin(horizontal_angle)*radius_at_level, cos(horizontal_angle)*radius_at_level, vertical_level) );
				horizontal_angle += d_horizontal_angle;
			}
			vertical_angle += d_vertical_angle;
		}
		polyhedron_data->addVertex( primitive_placement_matrix*carve::geom::VECTOR(0.0, 0.0, -radius) ); // bottom

		// uppper triangle fan
		for( int i = 0; i < nvc-1; ++i )
		{
			polyhedron_data->addFace(0, i+2, i+1);
		}
		polyhedron_data->addFace(0, 1, nvc);

		for( int vertical = 1; vertical < num_vertical_edges-2; ++vertical )
		{
			int offset_inner = nvc*(vertical-1) + 1;
			int offset_outer = nvc*vertical + 1;
			for( int i = 0; i < nvc-1; ++i )
			{
				polyhedron_data->addFace( offset_inner+i, offset_inner+1+i, offset_outer+1+i,  offset_outer+i );
			}
			polyhedron_data->addFace( offset_inner+nvc-1, offset_inner, offset_outer,  offset_outer+nvc-1 );

		}

		// lower triangle fan
		int last_index = (num_vertical_edges-2)*nvc + 1;
		for( int i = 0; i < nvc-1; ++i )
		{
			polyhedron_data->addFace(last_index, last_index-(i+2), last_index-(i+1) );
		}
		polyhedron_data->addFace(last_index, last_index-1, last_index-nvc);
		item_data->item_closed_mesh_data.push_back( polyhedron_data );
		return;
	}
	throw UnhandledRepresentationException(csg_primitive);
}



void SolidModelConverter::convertIfcRevolvedAreaSolid( const shared_ptr<IfcRevolvedAreaSolid>& revolved_area, const carve::math::Matrix& pos, shared_ptr<ItemData> item_data )
{
	if( !revolved_area->m_SweptArea )
	{
		return;
	}
	std::stringstream err;
	double length_factor = m_unit_converter->getLengthInMeterFactor();

	// angle and axis
	double angle_factor = m_unit_converter->getAngleInRadianFactor();
	shared_ptr<IfcProfileDef> swept_area_profile = revolved_area->m_SweptArea;
	double revolution_angle = revolved_area->m_Angle->m_value*angle_factor;

	carve::geom::vector<3>  axis_location;
	carve::geom::vector<3>  axis_direction;
	if(revolved_area->m_Axis)
	{
		shared_ptr<IfcAxis1Placement> axis_placement = revolved_area->m_Axis;

		if( axis_placement->m_Location )
		{
			shared_ptr<IfcCartesianPoint> location_point = axis_placement->m_Location;
			m_curve_converter->convertIfcCartesianPoint( location_point, axis_location );
		}

		if( axis_placement->m_Axis )
		{
			shared_ptr<IfcDirection> axis = axis_placement->m_Axis;
			axis_direction = carve::geom::VECTOR( axis->m_DirectionRatios[0], axis->m_DirectionRatios[1], axis->m_DirectionRatios[2] );
		}
	}

	// rotation base point is the one with the smallest distance on the rotation axis
	carve::geom::vector<3>  origin;
	carve::geom::vector<3>  base_point;
	GeomUtils::closestPointOnLine( origin, axis_location, axis_direction, base_point );
	base_point *= -1;

	// swept area
	shared_ptr<ProfileConverter> profile_converter = m_profile_cache->getProfileConverter(swept_area_profile);
	const std::vector<std::vector<carve::geom::vector<2> > >& profile_coords = profile_converter->getCoordinates();

	// tesselate
	std::vector<std::vector<carve::geom::vector<2> > > profile_coords_2d;
	for( int i = 0; i<profile_coords.size(); ++i )
	{
		const std::vector<carve::geom::vector<2> >& profile_loop = profile_coords[i];
		//std::vector<carve::geom::vector<2> > profile_loop_2d;
		//for( int j = 0; j<profile_loop.size(); ++j )
		//{
		//	profile_loop_2d.push_back( carve::geom::VECTOR( profile_loop[j].x, profile_loop[j].y ) );
		//}
		profile_coords_2d.push_back( profile_loop );
	}

	std::vector<carve::geom::vector<2> > merged;
	std::vector<carve::triangulate::tri_idx> triangulated;
	try
	{
		std::vector<std::pair<size_t, size_t> > result = carve::triangulate::incorporateHolesIntoPolygon(profile_coords_2d);	// first is loop index, second is vertex index in loop
		merged.reserve(result.size());
		for( size_t i = 0; i < result.size(); ++i )
		{
			int loop_number = result[i].first;
			int index_in_loop = result[i].second;

			if( loop_number >= profile_coords_2d.size() )
			{
				std::cout << "convertIfcRevolvedAreaSolid: loop_number >= face_loops_projected.size()" << std::endl;
				continue;
			}

			std::vector<carve::geom2d::P2>& loop_projected = profile_coords_2d[loop_number];

			carve::geom2d::P2& point_projected = loop_projected[index_in_loop];
			merged.push_back( point_projected );
		}
		carve::triangulate::triangulate(merged, triangulated);
		carve::triangulate::improve(merged, triangulated);
	}
	catch(...)
	{
		err << "carve::triangulate::incorporateHolesIntoPolygon failed " << std::endl;
		return;
	}



	if( profile_coords.size() == 0 )
	{
		std::stringstream strs;
		strs << "#" << revolved_area->getId() << " = IfcRevolvedAreaSolid: convertIfcRevolvedAreaSolid: num_loops == 0";
		detailedReport( strs );
		return;
		//throw IfcPPException("RepresentationConverter::convertIfcRevolvedAreaSolid: num_loops == 0", __func__);
	}
	if( profile_coords[0].size() < 3 )
	{
		std::stringstream strs;
		strs << "#" << revolved_area->getId() << " = IfcRevolvedAreaSolid: convertIfcRevolvedAreaSolid: num_polygon_points < 3";
		detailedReport( strs );
		//throw IfcPPException("RepresentationConverter::convertIfcRevolvedAreaSolid: num_polygon_points < 3", __func__);
	}

	if( revolution_angle > M_PI*2 ) revolution_angle = M_PI*2;
	if( revolution_angle < -M_PI*2 ) revolution_angle = M_PI*2;

	// TODO: calculate num segments according to length/width/height ratio and overall size of the object
	int num_segments = m_geom_settings->m_num_vertices_per_circle*(abs(revolution_angle)/(2.0*M_PI));
	if( num_segments < 6 )
	{
		num_segments = 6;
	}
	double angle = 0.0;
	double d_angle = revolution_angle/num_segments;

	// check if we have to change the direction
	carve::geom::vector<3>  polygon_normal = GeomUtils::computePolygon2DNormal( profile_coords[0] );
	const carve::geom::vector<2>&  pt0_2d = profile_coords[0][0];
	carve::geom::vector<3>  pt0_3d( carve::geom::VECTOR( pt0_2d.x, pt0_2d.y, 0 ) );
	carve::geom::vector<3>  pt0 = carve::math::Matrix::ROT(d_angle, axis_direction )*(pt0_3d + base_point);
	if( polygon_normal.z*pt0.z > 0 )
	{
		angle = revolution_angle;
		d_angle = -d_angle;
	}

	shared_ptr<carve::input::PolyhedronData> polyhedron_data( new carve::input::PolyhedronData() );
	item_data->item_closed_mesh_data.push_back(polyhedron_data);

	// create vertices
	carve::math::Matrix m;
	for( int i = 0; i <= num_segments; ++i )
	{
		m = carve::math::Matrix::ROT( angle, -axis_direction );
		for( int j=0; j<profile_coords.size(); ++j )
		{
			const std::vector<carve::geom::vector<2> >& loop = profile_coords[j];

			for( int k=0; k<loop.size(); ++k )
			{
				const carve::geom::vector<2>& point = loop[k];

				carve::geom::vector<3>  vertex= m*( carve::geom::VECTOR( point.x, point.y, 0 ) + base_point) - base_point;
				polyhedron_data->addVertex( pos*vertex );
			}
		}
		angle += d_angle;
	}

	// front cap
	std::vector<int> front_face_loop;
	int num_polygon_points = 0;
	for( int j=0; j<profile_coords.size(); ++j )
	{
		const std::vector<carve::geom::vector<2> >& loop = profile_coords[j];

		for( int k=0; k<loop.size(); ++k )
		{
			front_face_loop.push_back( j*loop.size() + k );
			++num_polygon_points;
		}
	}
	// TODO: use triangulated
	polyhedron_data->addFace( front_face_loop.rbegin(), front_face_loop.rend() );

	// end cap
	std::vector<int> end_face_loop;
	const int end_face_begin = num_segments*num_polygon_points;
	for( int j = 0; j < num_polygon_points; ++j )
	{
		end_face_loop.push_back( end_face_begin + j );
	}
	polyhedron_data->addFace( end_face_loop.begin(), end_face_loop.end() );

	// faces of revolved shape
	for( int i = 0; i < num_polygon_points-1; ++i )
	{
		int i_offset_next = i + num_polygon_points;
		for( int j = 0; j < num_segments; ++j )
		{
			int j_offset = j*num_polygon_points;
			polyhedron_data->addFace( j_offset+i, j_offset+i+1, j_offset+1+i_offset_next, j_offset+i_offset_next );
		}
	}

	for( int j = 0; j < num_segments; ++j )
	{
		int j_offset = j*num_polygon_points;
		polyhedron_data->addFace( j_offset+num_polygon_points-1, j_offset, j_offset+num_polygon_points, j_offset+num_polygon_points+num_polygon_points-1 );
	}

#ifdef _DEBUG
	std::stringstream strs_err;
	shared_ptr<carve::mesh::MeshSet<3> > mesh_set( polyhedron_data->createMesh(carve::input::opts()) );
	if( mesh_set->meshes.size() != 1 )
	{
		std::cout << "IfcRevolvedAreaSolid: mesh_set->meshes.size() != 1" << std::endl;
	}
	bool meshset_ok = ConverterOSG::checkMeshSet( mesh_set, strs_err, -1 );

	if( !meshset_ok )
	{
		std::cout << strs_err.str().c_str() << std::endl;
	}
#endif
}


void SolidModelConverter::convertIfcBooleanResult( const shared_ptr<IfcBooleanResult>& bool_result, const carve::math::Matrix& pos, shared_ptr<ItemData> item_data )
{
	const int boolean_result_id = bool_result->getId();
	std::stringstream strs_err;
	shared_ptr<IfcBooleanClippingResult> boolean_clipping_result = dynamic_pointer_cast<IfcBooleanClippingResult>(bool_result);
	if( boolean_clipping_result )
	{
		shared_ptr<IfcBooleanOperator> ifc_boolean_operator = boolean_clipping_result->m_Operator;
		shared_ptr<IfcBooleanOperand> ifc_first_operand = boolean_clipping_result->m_FirstOperand;
		shared_ptr<IfcBooleanOperand> ifc_second_operand = boolean_clipping_result->m_SecondOperand;

		if( !ifc_boolean_operator || !ifc_first_operand || !ifc_second_operand )
		{
			std::cout << "RepresentationConverter::convertIfcBooleanResult: invalid IfcBooleanOperator or IfcBooleanOperand" << std::endl;
			return;
		}

		carve::csg::CSG::OP csg_operation = carve::csg::CSG::A_MINUS_B;
		if( ifc_boolean_operator->m_enum == IfcBooleanOperator::ENUM_UNION )
		{
			csg_operation = carve::csg::CSG::UNION;
		}
		else if( ifc_boolean_operator->m_enum == IfcBooleanOperator::ENUM_INTERSECTION )
		{
			csg_operation = carve::csg::CSG::INTERSECTION;
		}
		else if( ifc_boolean_operator->m_enum == IfcBooleanOperator::ENUM_DIFFERENCE )
		{
			csg_operation = carve::csg::CSG::A_MINUS_B;
		}
		else
		{
			std::cout << "RepresentationConverter::convertIfcBooleanResult: invalid IfcBooleanOperator" << std::endl;
		}

		// convert the first operand
		//const int first_operand_id = ifc_first_operand->getId();
		shared_ptr<ItemData> first_operand_data( new ItemData() );
		convertIfcBooleanOperand( ifc_first_operand, pos, first_operand_data );
		first_operand_data->createMeshSetsFromClosedPolyhedrons();

		// convert the second operand
		shared_ptr<ItemData> second_operand_data( new ItemData() );
		convertIfcBooleanOperand( ifc_second_operand, pos, second_operand_data );
		second_operand_data->createMeshSetsFromClosedPolyhedrons();

		// for every first operand polyhedrons, apply all second operand polyhedrons
		std::vector<shared_ptr<carve::mesh::MeshSet<3> > >::iterator it_first_operands;
		for( it_first_operands=first_operand_data->item_meshsets.begin(); it_first_operands!=first_operand_data->item_meshsets.end(); ++it_first_operands )
		{
			shared_ptr<carve::mesh::MeshSet<3> >& first_operand_meshset = (*it_first_operands);
			
			// check meshset
			bool first_op_meshset_ok = ConverterOSG::checkMeshSet( first_operand_meshset, strs_err, -1 );
			if( !first_op_meshset_ok )
			{
#ifdef _DEBUG
				std::cout << strs_err.str().c_str() << std::endl;
				renderMeshsetInDebugViewer( first_operand_meshset, osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f), true );
#endif
				continue;
			}

			if( m_geom_settings->m_use_mesh_simplifier_before_csg )
			{
				// simplify
				carve::mesh::MeshSimplifier simplifier;
				simplifier.improveMesh( first_operand_meshset.get(), m_geom_settings->m_min_colinearity, m_geom_settings->m_min_delta_v, m_geom_settings->m_min_normal_angle );
			}

			std::vector<shared_ptr<carve::mesh::MeshSet<3> > >::iterator it_second_operands;
			for( it_second_operands=second_operand_data->item_meshsets.begin(); it_second_operands!=second_operand_data->item_meshsets.end(); ++it_second_operands )
			{
				shared_ptr<carve::mesh::MeshSet<3> >& second_operand_meshset = (*it_second_operands);

				bool second_op_meshset_ok = ConverterOSG::checkMeshSet( second_operand_meshset, strs_err, -1 );
				if( !second_op_meshset_ok )
				{
#ifdef _DEBUG
					std::cout << strs_err.str().c_str() << std::endl;
					renderMeshsetInDebugViewer( second_operand_meshset, osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f), true );
#endif
					continue;
				}

				if( m_geom_settings->m_use_mesh_simplifier_before_csg )
				{
					// simplify
					carve::mesh::MeshSimplifier simplifier;
					simplifier.improveMesh( second_operand_meshset.get(), m_geom_settings->m_min_colinearity, m_geom_settings->m_min_delta_v, m_geom_settings->m_min_normal_angle );
				}

				bool csg_operation_successful = true;
				try
				{
					// TODO: switch off std::cerr output in carve in release mode
					carve::csg::CSG csg;
					if( m_geom_settings->m_set_process_output_face )
					{
						csg.hooks.registerHook(new carve::csg::CarveTriangulator(), carve::csg::CSG::Hooks::PROCESS_OUTPUT_FACE_BIT);
					}
					shared_ptr<carve::mesh::MeshSet<3> > result_meshset( csg.compute( first_operand_meshset.get(), second_operand_meshset.get(), csg_operation, NULL, m_geom_settings->m_classify_type) );
					
					if( m_geom_settings->m_use_mesh_simplifier_after_csg )
					{
						carve::mesh::MeshSimplifier simplifier;
						simplifier.improveMesh( result_meshset.get(), m_geom_settings->m_min_colinearity, m_geom_settings->m_min_delta_v, m_geom_settings->m_min_normal_angle );
						// TODO: map with meshes that have been simplified already to avoid double improveMesh calls before other csg operations
					}
					bool result_meshset_ok = ConverterOSG::checkMeshSet( result_meshset, strs_err, -1 );
#ifdef _DEBUG
					if( !result_meshset_ok )
					{
						renderMeshsetInDebugViewer( first_operand_meshset, osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f), true );
						renderMeshsetInDebugViewer( second_operand_meshset, osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f), true );
						renderMeshsetInDebugViewer( result_meshset, osg::Vec4(1.0f, 1.0f, 0.0f, 1.0f), false );
						ConverterOSG::dumpMeshsets( first_operand_meshset, second_operand_meshset, result_meshset );
					}
#endif
					first_operand_meshset = result_meshset;
				}
				catch( carve::exception& ce )
				{
					csg_operation_successful = false;
					strs_err << ce.str();
				}
				catch(std::exception& e)
				{
					csg_operation_successful = false;
					strs_err << e.what();
				}
				catch(...)
				{
					csg_operation_successful = false;
					strs_err << "convertIfcProduct: csg operation failed" << std::endl;
				}

#ifdef _DEBUG
				if( !csg_operation_successful )
				{
					std::cout << "!csg_operation_successful. IfcBooleanResult id: " << boolean_result_id << std::endl;
					renderMeshsetInDebugViewer( first_operand_meshset, osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f), true );
					renderMeshsetInDebugViewer( second_operand_meshset, osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f), true );
					ConverterOSG::dumpMeshsets( first_operand_meshset, second_operand_meshset, shared_ptr<carve::mesh::MeshSet<3> >() );
				}
#endif
			}
		}

		// now copy processed first operands to result input data
		std::copy( first_operand_data->item_meshsets.begin(), first_operand_data->item_meshsets.end(), std::back_inserter(item_data->item_meshsets) );
	}

	if( strs_err.tellp() > 0 )
	{
		detailedReport( strs_err );
		//throw IfcPPException( err.str().c_str(), __func__ );
	}
}


void extrudeBox( const std::vector<carve::geom::vector<3> >& boundary_points, const carve::geom::vector<3>& extrusion_vector, shared_ptr<carve::input::PolyhedronData>& box_data )
{
	box_data->addVertex( boundary_points[0] );
	box_data->addVertex( boundary_points[1] );
	box_data->addVertex( boundary_points[2] );
	box_data->addVertex( boundary_points[3] );
	box_data->addVertex( boundary_points[0] + extrusion_vector );
	box_data->addVertex( boundary_points[1] + extrusion_vector );
	box_data->addVertex( boundary_points[2] + extrusion_vector );
	box_data->addVertex( boundary_points[3] + extrusion_vector );
	box_data->addFace( 0,1,2 );
	box_data->addFace( 2,3,0 );
	box_data->addFace( 1,5,6 );
	box_data->addFace( 6,2,1 );
	box_data->addFace( 5,4,7 );
	box_data->addFace( 7,6,5 );
	box_data->addFace( 0,3,7 );
	box_data->addFace( 7,4,0 );
	box_data->addFace( 0,4,5 );
	box_data->addFace( 5,1,0 );
	box_data->addFace( 2,6,7 );
	box_data->addFace( 7,3,2 );
}

void SolidModelConverter::convertIfcBooleanOperand( const shared_ptr<IfcBooleanOperand>& operand, const carve::math::Matrix& pos, shared_ptr<ItemData> item_data )
{
	shared_ptr<IfcSolidModel> solid_model = dynamic_pointer_cast<IfcSolidModel>(operand);
	if( solid_model )
	{
		convertIfcSolidModel( solid_model, pos, item_data );
		return;
	}
	double length_factor = m_unit_converter->getLengthInMeterFactor();
	std::stringstream strs_err;

	shared_ptr<IfcHalfSpaceSolid> half_space_solid = dynamic_pointer_cast<IfcHalfSpaceSolid>(operand);
	if( half_space_solid )
	{
		//ENTITY IfcHalfSpaceSolid SUPERTYPE OF(ONEOF(IfcBoxedHalfSpace, IfcPolygonalBoundedHalfSpace))
		shared_ptr<IfcSurface> base_surface = half_space_solid->m_BaseSurface;
		if( !dynamic_pointer_cast<IfcPlane>(base_surface) )
		{
			std::cout << "RepresentationConverter::convertIfcBooleanOperand: The BaseSurface defined at supertype IfcHalfSpaceSolid shall be of type IfcPlane" << std::endl;
			return;
		}

		shared_ptr<carve::input::PolylineSetData> surface_data ( new carve::input::PolylineSetData() );
		carve::math::Matrix matrix_ident( carve::math::Matrix::IDENT() );

		m_face_converter->convertIfcSurface( base_surface, matrix_ident, surface_data );
		std::vector<carve::geom::vector<3> > base_surface_points = surface_data->points;

		if( base_surface_points.size() != 4 )
		{
			std::cout << "RepresentationConverter::convertIfcBooleanOperand: invalid IfcHalfSpaceSolid.BaseSurface" << std::endl;
			return;
		}

		// If the agreement flag is TRUE, then the subset is the one the normal points away from
		bool agreement = half_space_solid->m_AgreementFlag;
		if( !agreement )
		{
			std::reverse( base_surface_points.begin(), base_surface_points.end() );
		}

		// plane equation of the base surface
		carve::geom::vector<3>  base_surface_normal = GeomUtils::computePolygonNormal( base_surface_points );
				
		shared_ptr<IfcBoxedHalfSpace> boxed_half_space = dynamic_pointer_cast<IfcBoxedHalfSpace>(half_space_solid);
		if( boxed_half_space )
		{
			shared_ptr<IfcBoundingBox> bbox = boxed_half_space->m_Enclosure;
			shared_ptr<IfcCartesianPoint>&			bbox_corner = bbox->m_Corner;
			shared_ptr<IfcPositiveLengthMeasure>&	bbox_x_dim = bbox->m_XDim;
			shared_ptr<IfcPositiveLengthMeasure>&	bbox_y_dim = bbox->m_YDim;
			shared_ptr<IfcPositiveLengthMeasure>&	bbox_z_dim = bbox->m_ZDim;
			// TODO: implement according to http://www.buildingsmart-tech.org/ifc/IFC4/final/html/figures/IfcBoxedHalfSpace_01.png
			std::cout << "convertIfcBooleanOperand: IfcBoxedHalfSpace not implemented" << std::endl;
			
			// if we do not return here, an unbounded plane will be used for now
			//return;
		}
		
		
		shared_ptr<IfcPolygonalBoundedHalfSpace> polygonal_half_space = dynamic_pointer_cast<IfcPolygonalBoundedHalfSpace>(half_space_solid);
		if( polygonal_half_space )
		{
			// ENTITY IfcPolygonalBoundedHalfSpace 
			//	SUBTYPE OF IfcHalfSpaceSolid;
			//	Position	 :	IfcAxis2Placement3D;
			//	PolygonalBoundary	 :	IfcBoundedCurve;

			carve::math::Matrix boundary_position_matrix( carve::math::Matrix::IDENT() );
			if( polygonal_half_space->m_Position )
			{
				PlacementConverter::convertIfcAxis2Placement3D( polygonal_half_space->m_Position, boundary_position_matrix, length_factor );
			}

			// PolygonalBoundary is given in 2D
			std::vector<carve::geom::vector<2> > polygonal_boundary;
			std::vector<carve::geom::vector<2> > segment_start_points_2d;
			shared_ptr<IfcBoundedCurve> bounded_curve = polygonal_half_space->m_PolygonalBoundary;
			m_curve_converter->convertIfcCurve2D( bounded_curve, polygonal_boundary, segment_start_points_2d );
			ProfileConverter::deleteLastPointIfEqualToFirst( polygonal_boundary );

			carve::geom::vector<3> polygonal_boundary_normal = GeomUtils::computePolygon2DNormal( polygonal_boundary );
			//if( polygonal_boundary_normal.z > 0 )
			{
			//	std::reverse( polygonal_boundary.begin(), polygonal_boundary.end() );
			}
			
			std::stringstream err;
			std::vector<std::vector<carve::geom::vector<2> > > paths;
			paths.push_back( polygonal_boundary );
			shared_ptr<carve::input::PolyhedronData> poly_data( new carve::input::PolyhedronData );
			GeomUtils::extrude( paths, carve::geom::vector<3>( carve::geom::VECTOR( 0, 0, HALF_SPACE_BOX_SIZE*2.0 ) ), poly_data, err );

			// move down, so that it can be cut away by base surface
			for( std::vector<carve::geom::vector<3> >::iterator it_points = poly_data->points.begin(); it_points != poly_data->points.end(); ++it_points )
			{
				carve::geom::vector<3>& vertex = (*it_points);
				vertex.z -= HALF_SPACE_BOX_SIZE;
			}

			// apply object coordinate system
			boundary_position_matrix = pos*boundary_position_matrix;
			
			// apply position of PolygonalBoundary
			for( std::vector<carve::geom::vector<3> >::iterator it_points = poly_data->points.begin(); it_points != poly_data->points.end(); ++it_points )
			{
				carve::geom::vector<3>& vertex = (*it_points);
				vertex = boundary_position_matrix*vertex;
			}

			// create simple box
			carve::geom::vector<3>  half_space_extrusion_vector = base_surface_normal*HALF_SPACE_BOX_SIZE*2.0;
			shared_ptr<carve::input::PolyhedronData> half_space_box_data( new carve::input::PolyhedronData() );
			std::vector<carve::geom::vector<3> > box_points;

			carve::geom::vector<3>  polygon_centroid = GeomUtils::computePolygonCentroid( base_surface_points );
            double box_depth = HALF_SPACE_BOX_SIZE;         // TODO: adapt to model bounding box

			const int num_vertices = base_surface_points.size();
			for( int k=0; k<num_vertices; ++k )
			{
				carve::geom::vector<3>  point = base_surface_points.at(k);

				// move the point far away so that it becomes practically unbounded
				carve::geom::vector<3>  centroid_to_point_vec = point - polygon_centroid;
				centroid_to_point_vec.normalize();
				carve::geom::vector<3>  point_far = polygon_centroid + centroid_to_point_vec*HALF_SPACE_BOX_SIZE;
				box_points.push_back( point_far );
			}

			// until now, the normal vector points in same direction as extrusion vector. flip normal vector by reversing the box boundary points
			std::reverse( box_points.begin(), box_points.end() );
			extrudeBox( box_points, half_space_extrusion_vector, half_space_box_data );

			// apply object coordinate system
			for( std::vector<carve::geom::vector<3> >::iterator it_points = half_space_box_data->points.begin(); it_points != half_space_box_data->points.end(); ++it_points )
			{
				carve::geom::vector<3> & vertex = (*it_points);
				vertex = pos*vertex;
			}

			// now cut with base surface
			shared_ptr<carve::mesh::MeshSet<3> > half_space_box_meshset( half_space_box_data->createMesh(carve::input::opts()) );
			shared_ptr<carve::mesh::MeshSet<3> > poly_data_meshset( poly_data->createMesh(carve::input::opts()) );

#ifdef _DEBUG
			
			bool box_ok = ConverterOSG::checkMeshSet( half_space_box_meshset, strs_err, 0 );
			if( !box_ok )
			{
				renderMeshsetInDebugViewer( half_space_box_meshset, osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f), false );
				ConverterOSG::dumpMeshsets( half_space_box_meshset, shared_ptr<carve::mesh::MeshSet<3> >(), shared_ptr<carve::mesh::MeshSet<3> >() );
				std::cout << "half_space_box_meshset not ok " << __func__ << std::endl;
			}
			if( !ConverterOSG::checkMeshSet( poly_data_meshset, strs_err, 0 ) )
			{
				renderMeshsetInDebugViewer( poly_data_meshset, osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f), false );
				ConverterOSG::dumpMeshsets( poly_data_meshset, shared_ptr<carve::mesh::MeshSet<3> >(), shared_ptr<carve::mesh::MeshSet<3> >() );
				std::cout << "poly_data_meshset not ok " << __func__ << std::endl;
			}
#endif
			if( m_geom_settings->m_use_mesh_simplifier_before_csg )
			{
				// simplify
				carve::mesh::MeshSimplifier simplifier;
				simplifier.improveMesh( half_space_box_meshset.get(), m_geom_settings->m_min_colinearity, m_geom_settings->m_min_delta_v, m_geom_settings->m_min_normal_angle );
				carve::mesh::MeshSimplifier simplifier2;
				simplifier2.improveMesh( poly_data_meshset.get(), m_geom_settings->m_min_colinearity, m_geom_settings->m_min_delta_v, m_geom_settings->m_min_normal_angle );
			}

			carve::csg::CSG csg;
			if( m_geom_settings->m_set_process_output_face )
			{
				csg.hooks.registerHook(new carve::csg::CarveTriangulator(), carve::csg::CSG::Hooks::PROCESS_OUTPUT_FACE_BIT);
			}
			shared_ptr<carve::mesh::MeshSet<3> > halfspace_meshset( csg.compute( poly_data_meshset.get(), half_space_box_meshset.get(), carve::csg::CSG::A_MINUS_B, NULL, m_geom_settings->m_classify_type ) );
			
			if( m_geom_settings->m_use_mesh_simplifier_after_csg )
			{
				// simplify result
				carve::mesh::MeshSimplifier simplifier;
				simplifier.improveMesh( halfspace_meshset.get(), m_geom_settings->m_min_colinearity, m_geom_settings->m_min_delta_v, m_geom_settings->m_min_normal_angle );
			}

			bool result_meshset_ok = ConverterOSG::checkMeshSet( halfspace_meshset, strs_err, 0 );

#ifdef _DEBUG
			if( !result_meshset_ok )
			{
				renderMeshsetInDebugViewer( half_space_box_meshset, osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f), true );
				renderMeshsetInDebugViewer( poly_data_meshset, osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f), true );
				renderMeshsetInDebugViewer( halfspace_meshset, osg::Vec4(0.0f, 0.0f, 1.0f, 1.0f), false );
				ConverterOSG::dumpMeshsets( half_space_box_meshset, poly_data_meshset, halfspace_meshset );
				std::cout << "result_meshset_ok not ok " << __func__ << std::endl;
			}
#endif
		
			item_data->item_meshsets.push_back( halfspace_meshset );

		}
		else
		{
			// else, its an unbounded half space solid, create simple box
			carve::geom::vector<3>  half_space_extrusion_direction = -base_surface_normal;
			carve::geom::vector<3>  half_space_extrusion_vector = half_space_extrusion_direction*HALF_SPACE_BOX_SIZE;
			shared_ptr<carve::input::PolyhedronData> half_space_box_data( new carve::input::PolyhedronData() );
			item_data->item_closed_mesh_data.push_back(half_space_box_data);
			extrudeBox( base_surface_points, half_space_extrusion_vector, half_space_box_data );

			// apply object coordinate system
			for( std::vector<carve::geom::vector<3> >::iterator it_points = half_space_box_data->points.begin(); it_points != half_space_box_data->points.end(); ++it_points )
			{
				carve::geom::vector<3> & vertex = (*it_points);
				vertex = pos*vertex;
			}

			return;
		}
		
		if( strs_err.tellp() > 0 )
		{
			throw IfcPPException( strs_err.str().c_str(), __func__ );
		}
		return;
		
	}
	
	shared_ptr<IfcBooleanResult> boolean_result = dynamic_pointer_cast<IfcBooleanResult>(operand);
	if( boolean_result )
	{
		int item_id = boolean_result->getId();
		convertIfcBooleanResult( boolean_result, pos, item_data );
		return;
	}

	shared_ptr<IfcCsgPrimitive3D> csg_primitive3D = dynamic_pointer_cast<IfcCsgPrimitive3D>(operand);
	if( csg_primitive3D )
	{
		convertIfcCsgPrimitive3D( csg_primitive3D, pos, item_data );
		return;
	}

	UnhandledRepresentationException unhadled_abstract;
	unhadled_abstract.m_select = operand;
	throw unhadled_abstract;
}
