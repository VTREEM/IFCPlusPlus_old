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

#include <vector>
#pragma warning (disable: 4267)
#include <carve/carve.hpp>
#include <carve/geom3d.hpp>
#include <carve/input.hpp>
#include <carve/csg_triangulator.hpp>

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
#include "UnhandledRepresentationException.h"
#include "RepresentationConverter.h"
#include "SolidModelConverter.h"

SolidModelConverter::SolidModelConverter( shared_ptr<GeometrySettings> geom_settings, shared_ptr<UnitConverter> uc, shared_ptr<CurveConverter>	cc, shared_ptr<FaceConverter> fc, shared_ptr<ProfileCache>	pc ) 
	: m_geom_settings(geom_settings), m_unit_converter(uc), m_curve_converter(cc), m_face_converter(fc), m_profile_cache( pc )
{
	m_debug_view = NULL;

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
			std::copy( input_data_outer_shell->open_or_closed_mesh_data.begin(), input_data_outer_shell->open_or_closed_mesh_data.end(), std::back_inserter(item_data->closed_mesh_data) );
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
		item_data->closed_mesh_data.push_back(pipe_data);
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
			bisectingPlane( vertex_before, vertex_current, vertex_next, bisecting_normal );

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
			
			convertPlane2Matrix( bisecting_normal, vertex_current, local_z, matrix_sweep );
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
		carve::input::Options carve_options;
		std::stringstream strs_err;
		bool poly_ok = ConverterOSG::checkMeshSet( shared_ptr<carve::mesh::MeshSet<3> >( pipe_data->createMesh(carve_options) ), strs_err, -1 );
				
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
	extrude( paths, extrusion_vector, poly_data, err );

	// apply object coordinate system
	std::vector<carve::geom::vector<3> >& points = poly_data->points;
	for( std::vector<carve::geom::vector<3> >::iterator it_points = points.begin(); it_points != points.end(); ++it_points )
	{
		carve::geom::vector<3>& vertex = (*it_points);
		vertex = pos*vertex;
	}

	item_data->closed_mesh_data.push_back( poly_data );

#ifdef _DEBUG
	carve::input::Options carve_options;
	std::stringstream strs_err;
	shared_ptr<carve::mesh::MeshSet<3> > mesh_set( poly_data->createMesh(carve_options) );
	bool poly_ok = ConverterOSG::checkMeshSet( mesh_set, strs_err, -1 );
				
	if( !poly_ok )
	{
		std::cout << strs_err.str().c_str() << std::endl;
#ifdef GEOM_DEBUG
		renderMeshsetInDebugViewer( m_debug_view, mesh_set, osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f), true );
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
			
			renderPolylineInDebugViewer( m_debug_view, polyline_data, osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f) );
		}
#endif

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
		
		item_data->closed_mesh_data.push_back( polyhedron_data );
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

		item_data->closed_mesh_data.push_back( polyhedron_data );
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

		item_data->closed_mesh_data.push_back( polyhedron_data );
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

		item_data->closed_mesh_data.push_back( polyhedron_data );
		return;
	}

	shared_ptr<IfcSphere> sphere = dynamic_pointer_cast<IfcSphere>(csg_primitive);
	if( sphere )
	{
		// TODO: implement

		std::cout << "IfcSphere not implemented" << std::endl;

		//input_data.closed_mesh_data.push_back( polyhedron_data );
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
	closestPointOnLine( base_point, origin, axis_location, axis_direction );
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
	carve::geom::vector<3>  polygon_normal = computePolygon2DNormal( profile_coords[0] );
	const carve::geom::vector<2>&  pt0_2d = profile_coords[0][0];
	carve::geom::vector<3>  pt0_3d( carve::geom::VECTOR( pt0_2d.x, pt0_2d.y, 0 ) );
	carve::geom::vector<3>  pt0 = carve::math::Matrix::ROT(d_angle, axis_direction )*(pt0_3d + base_point);
	if( polygon_normal.z*pt0.z > 0 )
	{
		angle = revolution_angle;
		d_angle = -d_angle;
	}

	shared_ptr<carve::input::PolyhedronData> polyhedron_data( new carve::input::PolyhedronData() );
	item_data->closed_mesh_data.push_back(polyhedron_data);

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
		carve::input::Options carve_options;
		std::stringstream strs_err;
		shared_ptr<carve::mesh::MeshSet<3> > mesh_set( polyhedron_data->createMesh(carve_options) );
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
		shared_ptr<ItemData> first_operand_input_data( new ItemData() );
		convertIfcBooleanOperand( ifc_first_operand, pos, first_operand_input_data );

		std::vector<shared_ptr<carve::mesh::MeshSet<3> > > meshsets_first_operand( first_operand_input_data->meshsets );
		for( unsigned int i=0; i<first_operand_input_data->closed_mesh_data.size(); ++i )
		{
			shared_ptr<carve::input::PolyhedronData>& first_operand_poly_data = first_operand_input_data->closed_mesh_data[i];
			if( first_operand_poly_data->getVertexCount() < 3 )
			{
				continue;
			}

			carve::input::Options carve_options;
			shared_ptr<carve::mesh::MeshSet<3> > first_operand_meshset( first_operand_poly_data->createMesh(carve_options) );
			meshsets_first_operand.push_back( first_operand_meshset );
		}

		// convert the second operand
		shared_ptr<ItemData> input_data_second_operand( new ItemData() );
		convertIfcBooleanOperand( ifc_second_operand, pos, input_data_second_operand );

		std::vector<shared_ptr<carve::mesh::MeshSet<3> > > polyhedrons_second_operand( input_data_second_operand->meshsets );
		for( unsigned int i=0; i<input_data_second_operand->closed_mesh_data.size(); ++i )
		{
			shared_ptr<carve::input::PolyhedronData>& polyhedron_data = input_data_second_operand->closed_mesh_data[i];
			if( polyhedron_data->getVertexCount() < 3 )
			{
				continue;
			}

			carve::input::Options carve_options;
			shared_ptr<carve::mesh::MeshSet<3> > second_operand_meshset(polyhedron_data->createMesh(carve_options) );
			polyhedrons_second_operand.push_back( second_operand_meshset );
		}

		// for every first operand polyhedrons, apply all second operand polyhedrons
		std::vector<shared_ptr<carve::mesh::MeshSet<3> > >::iterator it_first_operands;
		for( it_first_operands=meshsets_first_operand.begin(); it_first_operands!=meshsets_first_operand.end(); ++it_first_operands )
		{
			shared_ptr<carve::mesh::MeshSet<3> >& first_operand_meshset = (*it_first_operands);

			std::vector<shared_ptr<carve::mesh::MeshSet<3> > >::iterator it_second_operands;
			for( it_second_operands=polyhedrons_second_operand.begin(); it_second_operands!=polyhedrons_second_operand.end(); ++it_second_operands )
			{
				shared_ptr<carve::mesh::MeshSet<3> >& second_operand_meshset = (*it_second_operands);

				// check polyhedron
				bool meshset_ok = ConverterOSG::checkMeshSet( first_operand_meshset, strs_err, -1 );
				
				if( !meshset_ok )
				{
					continue;
#ifdef GEOM_DEBUG
					std::cout << strs_err.str().c_str() << std::endl;
					renderMeshsetInDebugViewer( m_debug_view, first_operand_meshset, osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f), true );
#endif
				}
				

				//shared_ptr<carve::poly::Polyhedron> second_op_polyhedron( carve::polyhedronFromMesh(second_operand.get(), -1 ) ); // -1 takes all meshes
				meshset_ok = ConverterOSG::checkMeshSet( second_operand_meshset, strs_err, -1 );
				
				if( !meshset_ok )
				{
					continue;
#ifdef GEOM_DEBUG
					std::cout << strs_err.str().c_str() << std::endl;
					renderMeshsetInDebugViewer( m_debug_view, second_operand_meshset, osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f), true );
#endif
				}

				bool csg_operation_successful = true;
				carve::csg::CSG csg;
				try
				{
					//std::cout << bool_result_id << std::endl;
					// TODO: switch off std::cerr output in carve in release mode
					shared_ptr<carve::mesh::MeshSet<3> > result_meshset( csg.compute( first_operand_meshset.get(), second_operand_meshset.get(), csg_operation, NULL, carve::csg::CSG::CLASSIFY_NORMAL) );
					bool result_meshset_ok = ConverterOSG::checkMeshSet( result_meshset, strs_err, -1 );
#ifdef _DEBUG
					if( !result_meshset_ok )
					{
						renderMeshsetInDebugViewer( m_debug_view, first_operand_meshset, osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f), true );
						renderMeshsetInDebugViewer( m_debug_view, second_operand_meshset, osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f), true );
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

#ifdef GEOM_DEBUG
				if( !csg_operation_successful )
				{
					renderMeshsetInDebugViewer( m_debug_view, first_operand_meshset, osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f), true );
					renderMeshsetInDebugViewer( m_debug_view, second_operand_meshset, osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f), true );
					
					//shared_ptr<carve::poly::Polyhedron> first_op_polyhedron( carve::polyhedronFromMesh(result_poly.get(), -1 ) ); // -1 takes all meshes
					//renderPolyhedronInDebugViewer( result_poly, osg::Vec4(0.0f, 0.0f, 1.0f, 1.0f), true );

#ifdef _DEBUG
					throw DebugBreakException( "!csg_operation_successful" );
#endif
				}
#endif
			}
		}

		// now copy processed first operands to result input data
		for( it_first_operands=meshsets_first_operand.begin(); it_first_operands!=meshsets_first_operand.end(); ++it_first_operands )
		{
			shared_ptr<carve::mesh::MeshSet<3> >& first_operand_meshset = (*it_first_operands);
			item_data->meshsets.push_back(first_operand_meshset);
		}
	}

	if( strs_err.tellp() > 0 )
	{
		detailedReport( strs_err );
		//throw IfcPPException( err.str().c_str(), __func__ );
	}
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

	shared_ptr<IfcHalfSpaceSolid> half_space_solid = dynamic_pointer_cast<IfcHalfSpaceSolid>(operand);
	if( half_space_solid )
	{
		//ENTITY IfcHalfSpaceSolid SUPERTYPE OF(ONEOF(IfcBoxedHalfSpace, IfcPolygonalBoundedHalfSpace))
		shared_ptr<IfcSurface> base_surface = half_space_solid->m_BaseSurface;
		
		shared_ptr<carve::input::PolylineSetData> surface_data ( new carve::input::PolylineSetData() );
		carve::math::Matrix matrix_ident( carve::math::Matrix::IDENT() );
		m_face_converter->convertIfcSurface( base_surface, matrix_ident, surface_data );
		std::vector<carve::geom::vector<3> > base_surface_points = surface_data->points;

		if( base_surface_points.size() < 3 )
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
		carve::geom::vector<3>  base_surface_normal = computePolygonNormal( base_surface_points );
		carve::geom::vector<3>  point_on_base_surface = *(base_surface_points.begin());
		carve::geom::plane<3> base_surface_plane_eqn( base_surface_normal, point_on_base_surface );

		std::vector<carve::geom::vector<3> > boundary_points;
		std::vector<carve::geom::vector<3> > segment_start_points;
		carve::geom::vector<3>  half_space_extrusion_direction;
				
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
			return;
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
			std::vector<carve::geom::vector<3> > polygonal_boundary;
			shared_ptr<IfcBoundedCurve> bounded_curve = polygonal_half_space->m_PolygonalBoundary;
			m_curve_converter->convertIfcCurve( bounded_curve, polygonal_boundary, segment_start_points );


			// apply position to polygonal boundary surface
			// http://www.buildingsmart-tech.org/: IfcPolygonalBoundedHalfSpace.Position. This coordinate system is relative to the object coordinate system. 
			// The extrusion direction of the subtraction body is the positive Z axis.
			for( unsigned int i=0; i<polygonal_boundary.size(); ++i )
			{
				carve::geom::vector<3>& vertex = polygonal_boundary.at(i);
				vertex = boundary_position_matrix*vertex;
			}

			// extrusion direction, take only rotation of the positioning matrix
			carve::math::Matrix boundary_rotation_matrix( boundary_position_matrix );
			boundary_rotation_matrix._41 = 0.0;
			boundary_rotation_matrix._42 = 0.0;
			boundary_rotation_matrix._43 = 0.0;
			half_space_extrusion_direction = boundary_rotation_matrix*carve::geom::VECTOR( 0, 0, 1 );
			half_space_extrusion_direction.normalize();

			// the base surface normal and the extrusion direction should point into the opposite half space
			double cos_angle = dot( half_space_extrusion_direction, base_surface_normal );
			
			if( cos_angle > 0 )
			{
				std::cout << "IfcPolygonalBoundedHalfSpace: base surface normal points into opposite halfspace than the extrusion vector" << std::endl;
			}

			// project 2D boundary into base surface
			const int num_boundary_points = (int)polygonal_boundary.size();
			for( int k=0; k<num_boundary_points; ++k )
			{
				carve::geom::vector<3> boundary_point = polygonal_boundary.at(k);
				carve::geom::vector<3>  boundary_point_in_surface;
				carve::geom3d::LineSegment ray( boundary_point, boundary_point + half_space_extrusion_direction );
				double t;
				carve::IntersectionClass intersects = carve::geom3d::rayPlaneIntersection(base_surface_plane_eqn, ray.v1, ray.v2, boundary_point_in_surface, t);

				if( intersects == carve::INTERSECT_NONE || intersects == carve::INTERSECT_BAD )
				{
					std::cout << "IfcPolygonalBoundedHalfSpace: PolygonalBoundary cannot be projected into BaseSurface" << std::endl;
				}

				boundary_points.push_back( boundary_point_in_surface );
			}
		}
		else
		{
			// else, its an unbounded half space solid
			carve::geom::vector<3>  polygon_centroid = computePolygonCentroid( base_surface_points );
			double box_depth = HALF_SPACE_BOX_SIZE;		// TODO: adapt to model bounding box

			const int num_vertices = base_surface_points.size();
			for( int k=0; k<num_vertices; ++k )
			{
				carve::geom::vector<3>  point = base_surface_points.at(k);

				// move the point far away so that it becomes practically unbounded
				carve::geom::vector<3>  centroid_to_point_vec = point - polygon_centroid;
				centroid_to_point_vec.normalize();
				carve::geom::vector<3>  point_far = polygon_centroid + centroid_to_point_vec*HALF_SPACE_BOX_SIZE;
				boundary_points.push_back( point_far );
			}

			half_space_extrusion_direction = -base_surface_normal;
		}

		// extrude the polyhedron
		std::stringstream err;
		shared_ptr<carve::input::PolyhedronData> box_data( new carve::input::PolyhedronData() );
		item_data->closed_mesh_data.push_back(box_data);
		std::vector<std::vector<carve::geom::vector<3> > > vec_boundary_paths;
		vec_boundary_paths.push_back( boundary_points );
		carve::geom::vector<3>  half_space_extrusion_vector = half_space_extrusion_direction*HALF_SPACE_BOX_SIZE;
		extrude3D( vec_boundary_paths, half_space_extrusion_vector, box_data, err );

		// apply object coordinate system
		std::vector<carve::geom::vector<3> >& half_space_solid_points = box_data->points;
		for( std::vector<carve::geom::vector<3> >::iterator it_points = half_space_solid_points.begin(); it_points != half_space_solid_points.end(); ++it_points )
		{
			carve::geom::vector<3> & vertex = (*it_points);
			vertex = pos*vertex;
		}
			
#ifdef _DEBUG
		carve::input::Options carve_options;
		std::stringstream strs_err;
		shared_ptr<carve::mesh::MeshSet<3> > mesh_set( box_data->createMesh(carve_options) );
		if( mesh_set->meshes.size() != 1 )
		{
			std::cout << "IfcPolygonalBoundedHalfSpace: mesh_set->meshes.size() != 1" << std::endl;
		}
		bool poly_ok = ConverterOSG::checkMeshSet( mesh_set, strs_err, -1 );

		//renderMeshsetInDebugViewer( m_debug_view, mesh_set, osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f), true );
		if( !poly_ok )
		{
			std::cout << strs_err.str().c_str() << std::endl;

#ifdef GEOM_DEBUG
			shared_ptr<carve::input::PolylineSetData> polyline_data( new carve::input::PolylineSetData() );
			polyline_data->beginPolyline();
			if( vec_boundary_paths.size() > 0 )
			{
				const std::vector<carve::geom::vector<3> >& loop = vec_boundary_paths[0]; 
				for( int i=0; i<loop.size(); ++i )
				{
					const carve::geom::vector<3> & point = loop.at(i);
					polyline_data->addVertex( point );
					polyline_data->addPolylineIndex(i);
				}

				osg::ref_ptr<osg::Geode> geode = new osg::Geode();
				ConverterOSG::drawPolyline( polyline_data, geode );
			
				renderPolylineInDebugViewer( m_debug_view, polyline_data, osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f) );
			}
#endif
		}
#endif

		if( err.tellp() > 0 )
		{
			throw IfcPPException( err.str().c_str(), __func__ );
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

