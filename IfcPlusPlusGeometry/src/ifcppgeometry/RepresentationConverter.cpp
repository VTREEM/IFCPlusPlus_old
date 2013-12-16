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

#define _USE_MATH_DEFINES 
#include <math.h>
#include <iterator>
#include <osg/Group>
#include <osg/MatrixTransform>

#include <carve/carve.hpp>
#include <carve/geom3d.hpp>
#include <carve/poly.hpp>
#include <carve/polyhedron_base.hpp>
//#include <carve/csg.hpp>
#include <carve/csg_triangulator.hpp>

#include "ifcpp/IFC4/include/IfcProduct.h"
#include "ifcpp/IFC4/include/IfcProductRepresentation.h"
#include "ifcpp/IFC4/include/IfcRepresentation.h"
#include "ifcpp/IFC4/include/IfcRepresentationItem.h"
#include "ifcpp/IFC4/include/IfcRelVoidsElement.h"
#include "ifcpp/IFC4/include/IfcFeatureElementSubtraction.h"
#include "ifcpp/IFC4/include/IfcElement.h"
#include "ifcpp/IFC4/include/IfcMappedItem.h"
#include "ifcpp/IFC4/include/IfcRepresentationMap.h"
#include "ifcpp/IFC4/include/IfcCartesianTransformationOperator.h"
#include "ifcpp/IFC4/include/IfcAxis2Placement.h"
#include "ifcpp/IFC4/include/IfcPlacement.h"

#include "ifcpp/IFC4/include/IfcLabel.h"
#include "ifcpp/IFC4/include/IfcStyledItem.h"
#include "ifcpp/IFC4/include/IfcProperty.h"
#include "ifcpp/IFC4/include/IfcPropertySet.h"
#include "ifcpp/IFC4/include/IfcComplexProperty.h"
#include "ifcpp/IFC4/include/IfcSimpleProperty.h"
#include "ifcpp/IFC4/include/IfcPropertySingleValue.h"
#include "ifcpp/IFC4/include/IfcIdentifier.h"

#include "ifcpp/IFC4/include/IfcLengthMeasure.h"
#include "ifcpp/IFC4/include/IfcPositiveLengthMeasure.h"
#include "ifcpp/IFC4/include/IfcPlaneAngleMeasure.h"
#include "ifcpp/IFC4/include/IfcCartesianPoint.h"
#include "ifcpp/IFC4/include/IfcPolyline.h"
#include "ifcpp/IFC4/include/IfcBoundingBox.h"
#include "ifcpp/IFC4/include/IfcEdge.h"
#include "ifcpp/IFC4/include/IfcClosedShell.h"
#include "ifcpp/IFC4/include/IfcFaceBasedSurfaceModel.h"
#include "ifcpp/IFC4/include/IfcConnectedFaceSet.h"
#include "ifcpp/IFC4/include/IfcShellBasedSurfaceModel.h"
#include "ifcpp/IFC4/include/IfcOpenShell.h"
#include "ifcpp/IFC4/include/IfcFace.h"
#include "ifcpp/IFC4/include/IfcVertexPoint.h"
#include "ifcpp/IFC4/include/IfcVertex.h"
#include "ifcpp/IFC4/include/IfcGeometricSet.h"
#include "ifcpp/IFC4/include/IfcGeometricCurveSet.h"
#include "ifcpp/IFC4/include/IfcCurve.h"
#include "ifcpp/IFC4/include/IfcSurface.h"

#include "ifcpp/IFC4/include/IfcSolidModel.h"
#include "ifcpp/IFC4/include/IfcSectionedSpine.h"
#include "ifcpp/IFC4/include/IfcBooleanResult.h"
#include "ifcpp/IFC4/include/IfcPresentationLayerWithStyle.h"


#include "ifcpp/model/IfcPPModel.h"
#include "ifcpp/model/UnitConverter.h"
#include "ifcpp/model/IfcPPException.h"

#include "Utility.h"
#include "ConverterOSG.h"
#include "GeometrySettings.h"
#include "PlacementConverter.h"
#include "ProfileConverter.h"
#include "ProfileCache.h"
#include "StylesConverter.h"
#include "UnhandledRepresentationException.h"
#include "CurveConverter.h"
#include "SolidModelConverter.h"
#include "FaceConverter.h"
#include "RepresentationConverter.h"

RepresentationConverter::RepresentationConverter( shared_ptr<GeometrySettings> geom_settings, shared_ptr<UnitConverter> unit_converter ) 
	: m_geom_settings(geom_settings), m_unit_converter(unit_converter)
{
	m_handle_styled_items = true;
	m_handle_layer_assignments = true;

	m_styles_converter = shared_ptr<StylesConverter>( new StylesConverter() );
	m_profile_cache = shared_ptr<ProfileCache>( new ProfileCache( m_geom_settings, m_unit_converter ) );
	m_curve_converter = shared_ptr<CurveConverter>( new CurveConverter( m_geom_settings, m_unit_converter ) );
	m_face_converter = shared_ptr<FaceConverter>( new FaceConverter( m_geom_settings, m_unit_converter, m_curve_converter ) );
	m_solid_converter = shared_ptr<SolidModelConverter>( new SolidModelConverter( m_geom_settings, m_unit_converter, m_curve_converter, m_face_converter, m_profile_cache ) );

#ifdef IFCPP_OPENMP
	omp_init_lock(&m_writelock_detailed_report);
	omp_init_lock(&m_writelock_styles_converter);
#endif

	m_debug_view = NULL;
}

RepresentationConverter::~RepresentationConverter()
{
#ifdef IFCPP_OPENMP
	omp_destroy_lock(&m_writelock_detailed_report);
	omp_destroy_lock(&m_writelock_styles_converter);
#endif
}

void RepresentationConverter::convertStyledItem( const shared_ptr<IfcRepresentationItem>& representation_item, shared_ptr<ItemData>& item_data )
{
	std::vector<weak_ptr<IfcStyledItem> >&	StyledByItem_inverse_vec = representation_item->m_StyledByItem_inverse;
	for( unsigned int i=0; i<StyledByItem_inverse_vec.size(); ++i )
	{
		weak_ptr<IfcStyledItem> styled_item_weak = StyledByItem_inverse_vec[i];
		shared_ptr<IfcStyledItem> styled_item = shared_ptr<IfcStyledItem>(styled_item_weak);

	#ifdef IFCPP_OPENMP
		omp_set_lock(&m_writelock_styles_converter);
		osg::StateSet* stateset = m_styles_converter->convertIfcStyledItem( styled_item );
		omp_unset_lock(&m_writelock_styles_converter);
	#else
		osg::StateSet* stateset = m_styles_converter->convertIfcStyledItem( styled_item );
	#endif
		if( stateset != NULL )
		{
			item_data->statesets.push_back(stateset);
		}
	}
}

void RepresentationConverter::convertIfcRepresentation(  const shared_ptr<IfcRepresentation>& representation, const carve::math::Matrix& pos, shared_ptr<RepresentationData>& input_data, std::set<int>& visited )
{
	std::stringstream err;
	double length_factor = m_unit_converter->getLengthInMeterFactor();
	const std::vector<shared_ptr<IfcRepresentationItem> >&	vec_representation_items = representation->m_Items;
	std::vector<shared_ptr<ItemData> >& vec_item_data = input_data->vec_item_data;

	std::vector<shared_ptr<IfcRepresentationItem> > unhandled_representation_items;
	std::vector<shared_ptr<IfcRepresentationItem> >::const_iterator it_representation_items;
	for( it_representation_items=vec_representation_items.begin(); it_representation_items!=vec_representation_items.end(); ++it_representation_items )
	{
		shared_ptr<IfcRepresentationItem> representation_item = (*it_representation_items);
		shared_ptr<ItemData> item_data( new ItemData() );
		vec_item_data.push_back( item_data );

		if( m_handle_styled_items )
		{
			convertStyledItem( representation_item, item_data );
		}

		//ENTITY IfcRepresentationItem  ABSTRACT SUPERTYPE OF(ONEOF(IfcGeometricRepresentationItem, IfcMappedItem, IfcStyledItem, IfcTopologicalRepresentationItem));
		shared_ptr<IfcGeometricRepresentationItem> geom_item = dynamic_pointer_cast<IfcGeometricRepresentationItem>(representation_item);
		if( geom_item )
		{
			try
			{
				convertIfcGeometricRepresentationItem( geom_item, pos, item_data );
			}
			catch( UnhandledRepresentationException& e)
			{
				if( e.m_item )
				{
					unhandled_representation_items.push_back( e.m_item );
				}
				else if( e.m_select )
				{
					err << "Unhandled Representation: " << e.m_select->classname() << std::endl;

				}
			}
			catch( IfcPPException& e )
			{
				err << e.what() << std::endl;
			}
#ifdef _DEBUG
			catch( DebugBreakException& e )
			{
				// pass exception up so that only the geometry with errors is shown
				throw DebugBreakException( e.what() );
			}
#endif
			catch( std::exception& e )
			{
				err << e.what() << std::endl;
			}
			catch( ... )
			{
				err << "convertIfcRepresentation: failed at item #" << representation_item->getId() << std::endl;
			}
			continue;
		}
		
		shared_ptr<IfcMappedItem> mapped_item = dynamic_pointer_cast<IfcMappedItem>(representation_item);
		if( mapped_item )
		{
			shared_ptr<IfcRepresentationMap> map_source = mapped_item->m_MappingSource;
			if( !map_source )
			{
				unhandled_representation_items.push_back( representation_item );
				continue;
			}
			shared_ptr<IfcRepresentation> mapped_representation = map_source->m_MappedRepresentation;
			if( !mapped_representation )
			{
				unhandled_representation_items.push_back( representation_item );
				continue;
			}
		
			carve::math::Matrix map_matrix_target( carve::math::Matrix::IDENT() );
			if( mapped_item->m_MappingTarget )
			{
				shared_ptr<IfcCartesianTransformationOperator> transform_operator = mapped_item->m_MappingTarget;
				PlacementConverter::convertTransformationOperator(	transform_operator, map_matrix_target, length_factor );
			}

			carve::math::Matrix map_matrix_origin( carve::math::Matrix::IDENT() );
			if( map_source->m_MappingOrigin )
			{
				shared_ptr<IfcAxis2Placement> mapping_origin = map_source->m_MappingOrigin;
				shared_ptr<IfcPlacement> mapping_origin_placement = dynamic_pointer_cast<IfcPlacement>( mapping_origin );
				if( mapping_origin_placement )
				{
					PlacementConverter::convertIfcPlacement( mapping_origin_placement, map_matrix_origin, length_factor );
				}
				else
				{
					std::stringstream strs;
					strs << "#" << mapping_origin_placement->getId() << " = IfcPlacement: !dynamic_pointer_cast<IfcPlacement>( mapping_origin ) )";
					detailedReport( strs );
					continue;
					//throw IfcPPException( "! dynamic_pointer_cast<IfcPlacement>( mapping_origin ) )" );
				}
			}

			carve::math::Matrix mapped_pos( (map_matrix_origin*pos)*map_matrix_target );
			convertIfcRepresentation( mapped_representation, mapped_pos, input_data, visited );
			continue;
		}
		
		shared_ptr<IfcStyledItem> styled_item = dynamic_pointer_cast<IfcStyledItem>(representation_item);
		if( styled_item )
		{
			continue;
		}

		shared_ptr<IfcTopologicalRepresentationItem> topo_item = dynamic_pointer_cast<IfcTopologicalRepresentationItem>(representation_item);
		if( topo_item )
		{
			//IfcTopologicalRepresentationItem 		ABSTRACT SUPERTYPE OF(ONEOF(IfcConnectedFaceSet, IfcEdge, IfcFace, IfcFaceBound, IfcLoop, IfcPath, IfcVertex))

			shared_ptr<IfcConnectedFaceSet> topo_connected_face_set = dynamic_pointer_cast<IfcConnectedFaceSet>(topo_item);
			if( topo_connected_face_set )
			{
				continue;
			}

			shared_ptr<IfcEdge> topo_edge = dynamic_pointer_cast<IfcEdge>(topo_item);
			if( topo_edge )
			{
				shared_ptr<carve::input::PolylineSetData> polyline_data( new carve::input::PolylineSetData() );
				polyline_data->beginPolyline();
				shared_ptr<IfcVertex>& vertex_start = topo_edge->m_EdgeStart;

				shared_ptr<IfcVertexPoint> vertex_start_point = dynamic_pointer_cast<IfcVertexPoint>(vertex_start);
				if( vertex_start_point )
				{
					if( vertex_start_point->m_VertexGeometry )
					{
						shared_ptr<IfcPoint> edge_start_point_geometry = vertex_start_point->m_VertexGeometry;
						shared_ptr<IfcCartesianPoint> ifc_point = dynamic_pointer_cast<IfcCartesianPoint>(edge_start_point_geometry);
						if( ifc_point )
						{
							if( ifc_point->m_Coordinates.size() > 2 )
							{
								carve::geom::vector<3> point = carve::geom::VECTOR( ifc_point->m_Coordinates[0]->m_value*length_factor, ifc_point->m_Coordinates[1]->m_value*length_factor, ifc_point->m_Coordinates[2]->m_value*length_factor );
								polyline_data->addVertex( point );
								polyline_data->addPolylineIndex(0);
							}
						}
					}
				}

				shared_ptr<IfcVertex>& vertex_end = topo_edge->m_EdgeEnd;

				shared_ptr<IfcVertexPoint> vertex_end_point = dynamic_pointer_cast<IfcVertexPoint>(vertex_end);
				if( vertex_end_point )
				{
					if( vertex_end_point->m_VertexGeometry )
					{
						shared_ptr<IfcPoint> edge_point_geometry = vertex_end_point->m_VertexGeometry;
						shared_ptr<IfcCartesianPoint> ifc_point = dynamic_pointer_cast<IfcCartesianPoint>(edge_point_geometry);
						if( ifc_point )
						{
							if( ifc_point->m_Coordinates.size() > 2 )
							{
								carve::geom::vector<3> point = carve::geom::VECTOR( ifc_point->m_Coordinates[0]->m_value*length_factor, ifc_point->m_Coordinates[1]->m_value*length_factor, ifc_point->m_Coordinates[2]->m_value*length_factor );
								polyline_data->addVertex( point );
								polyline_data->addPolylineIndex(1);
							}
						}
					}
				}
				item_data->polyline_data.push_back( polyline_data );
				continue;
			}
		}

		unhandled_representation_items.push_back( representation_item );
			

		//std::string representation_type;
		//std::string	representation_identifier;
		//if( representation->m_RepresentationType )
		//{
		//	representation_type =  representation->m_RepresentationType->m_value;
		//}
		//if( representation->m_RepresentationIdentifier )
		//{
		//	representation_identifier =  representation->m_RepresentationIdentifier->m_value;
		//}

		//if( representation_identifier.compare( "Axis" ) == 0 )
		//{
		//	if( representation_type.compare( "Curve2D" ) == 0 || representation_type.compare( "GeometricCurveSet" ) == 0 )
		//	{
		//		//osg::StateSet* stateset_axis = item_group->getOrCreateStateSet();
		//		//stateset_axis->setMode( GL_LIGHTING,osg::StateAttribute::OFF );

		//		//osg::LineStipple* linestipple = new osg::LineStipple;
		//		//linestipple->setFactor( 2 );
		//		//linestipple->setPattern( 0xAAAA );
		//		//stateset_axis->setAttributeAndModes( linestipple, osg::StateAttribute::OVERRIDE|osg::StateAttribute::ON );
		//	}
		//}
	}

	if( m_handle_layer_assignments )
	{
		std::vector<weak_ptr<IfcPresentationLayerAssignment> >& LayerAssignments_inverse = representation->m_LayerAssignments_inverse;
		std::vector<weak_ptr<IfcPresentationLayerAssignment> >::iterator it_LayerAssignments_inverse;
		for( it_LayerAssignments_inverse = LayerAssignments_inverse.begin(); it_LayerAssignments_inverse != LayerAssignments_inverse.end(); ++it_LayerAssignments_inverse)
		{
			shared_ptr<IfcPresentationLayerAssignment> layer_assignment( (*it_LayerAssignments_inverse) );

			shared_ptr<IfcPresentationLayerWithStyle> layer_assignment_with_style = dynamic_pointer_cast<IfcPresentationLayerWithStyle>( layer_assignment );
			if( layer_assignment_with_style )
			{
				std::vector<shared_ptr<IfcPresentationStyle> >& vec_presentation_styles = layer_assignment_with_style->m_LayerStyles;
				for( int i=0; i<vec_presentation_styles.size(); ++i )
				{
					shared_ptr<IfcPresentationStyle>&  presentation_style = vec_presentation_styles[i];

#ifdef IFCPP_OPENMP
					omp_set_lock(&m_writelock_styles_converter);
					osg::StateSet* stateset = m_styles_converter->convertIfcPresentationStyle( presentation_style );
					omp_unset_lock(&m_writelock_styles_converter);
#else
					osg::StateSet* stateset = m_styles_converter->convertIfcPresentationStyle( presentation_style );
#endif
					if( stateset != NULL )
					{
						input_data->statesets.push_back(stateset);
					}
				}
			}
		}
	}

	if( unhandled_representation_items.size() > 0 )
	{
		for( unsigned int i=0; i<unhandled_representation_items.size(); ++i )
		{
			shared_ptr<IfcRepresentationItem> representation_item = unhandled_representation_items[i];
			
			std::string representation_type;
			std::string	representation_identifier;
			if( representation->m_RepresentationType )
			{
				representation_type =  representation->m_RepresentationType->m_value;
			}
			if( representation->m_RepresentationIdentifier )
			{
				representation_identifier =  representation->m_RepresentationIdentifier->m_value;
			}

			std::stringstream strs;
			strs << "unhandled representation: " << strs.str().c_str() << representation_type << " " << representation_identifier << std::endl;
			detailedReport( strs );
		}
	}
	if( err.tellp() > 0 )
	{
		detailedReport( err );
		//throw IfcPPException( err.str().c_str() );
	}
}

void RepresentationConverter::convertIfcGeometricRepresentationItem( const shared_ptr<IfcGeometricRepresentationItem>& geom_item, const carve::math::Matrix& pos, shared_ptr<ItemData> item_data )
{
	//ENTITY IfcGeometricRepresentationItem
	//ABSTRACT SUPERTYPE OF(ONEOF(
	//	IfcAnnotationFillArea, IfcBooleanResult, IfcBoundingBox, IfcCartesianTransformationOperator, IfcCompositeCurveSegment, IfcCsgPrimitive3D,
	//	IfcCurve, IfcDefinedSymbol, IfcDirection, IfcFaceBasedSurfaceModel, IfcFillAreaStyleHatching, IfcFillAreaStyleTiles, IfcFillAreaStyleTileSymbolWithStyle,
	//	IfcGeometricSet, IfcHalfSpaceSolid, IfcLightSource, IfcOneDirectionRepeatFactor, IfcPlacement, IfcPlanarExtent, IfcPoint, IfcSectionedSpine,
	//	IfcShellBasedSurfaceModel, IfcSolidModel, IfcSurface, IfcTextLiteral, IfcTextureCoordinate, IfcTextureVertex, IfcVector))

	shared_ptr<IfcBoundingBox> bbox = dynamic_pointer_cast<IfcBoundingBox>(geom_item);
	if( bbox )
	{
		shared_ptr<IfcCartesianPoint> corner = bbox->m_Corner;
		shared_ptr<IfcPositiveLengthMeasure> x_dim = bbox->m_XDim;
		shared_ptr<IfcPositiveLengthMeasure> y_dim = bbox->m_YDim;
		shared_ptr<IfcPositiveLengthMeasure> z_dim = bbox->m_ZDim;
		return;
	}

	shared_ptr<IfcFaceBasedSurfaceModel> surface_model = dynamic_pointer_cast<IfcFaceBasedSurfaceModel>(geom_item);
	if( surface_model )
	{
		std::vector<shared_ptr<IfcConnectedFaceSet> >& vec_face_sets = surface_model->m_FbsmFaces;
		std::vector<shared_ptr<IfcConnectedFaceSet> >::iterator it_face_sets;

		
		for( it_face_sets=vec_face_sets.begin(); it_face_sets!=vec_face_sets.end(); ++it_face_sets )
		{
			shared_ptr<IfcConnectedFaceSet> face_set = (*it_face_sets);
			std::vector<shared_ptr<IfcFace> >& vec_ifc_faces = face_set->m_CfsFaces;

			shared_ptr<ItemData> input_data_face_set( new ItemData );
			m_face_converter->convertIfcFaceList( vec_ifc_faces, pos, input_data_face_set );
			std::copy( input_data_face_set->open_or_closed_mesh_data.begin(), input_data_face_set->open_or_closed_mesh_data.end(), std::back_inserter(item_data->open_mesh_data) );
		}
		
		return;
	}

	shared_ptr<IfcBooleanResult> boolean_result = dynamic_pointer_cast<IfcBooleanResult>(geom_item);
	if( boolean_result )
	{
		m_solid_converter->convertIfcBooleanResult( boolean_result, pos, item_data );
		return;
	}

	shared_ptr<IfcSolidModel> solid_model = dynamic_pointer_cast<IfcSolidModel>(geom_item);
	if( solid_model )
	{
		m_solid_converter->convertIfcSolidModel( solid_model, pos, item_data );
		return;
	}

	shared_ptr<IfcCurve> curve = dynamic_pointer_cast<IfcCurve>(geom_item);
	if( curve )
	{
		std::vector<carve::geom::vector<3> > loops;
		std::vector<carve::geom::vector<3> > segment_start_points;
		m_curve_converter->convertIfcCurve( curve, loops, segment_start_points );

		shared_ptr<carve::input::PolylineSetData> polyline_data( new carve::input::PolylineSetData() );
		polyline_data->beginPolyline();
		for( int i=0; i<loops.size(); ++i )
		{
			carve::geom::vector<3> point = loops.at(i);
			polyline_data->addVertex( pos*point );
			polyline_data->addPolylineIndex(i);
		}
		item_data->polyline_data.push_back( polyline_data );
		
		return;
	}

	shared_ptr<IfcShellBasedSurfaceModel> shell_based_surface_model = dynamic_pointer_cast<IfcShellBasedSurfaceModel>(geom_item);
	if( shell_based_surface_model )
	{
		std::vector<shared_ptr<IfcShell> >& vec_shells = shell_based_surface_model->m_SbsmBoundary;
		for( std::vector<shared_ptr<IfcShell> >::iterator it_shells=vec_shells.begin(); it_shells!=vec_shells.end(); ++it_shells )
		{
			shared_ptr<IfcShell> shell_select = (*it_shells);
			if( dynamic_pointer_cast<IfcClosedShell>( shell_select ) )
			{
				shared_ptr<IfcClosedShell> closed_shell = dynamic_pointer_cast<IfcClosedShell>( shell_select );
				std::vector<shared_ptr<IfcFace> >& vec_ifc_faces = closed_shell->m_CfsFaces;

				shared_ptr<ItemData> input_data( new ItemData() );
				m_face_converter->convertIfcFaceList( vec_ifc_faces, pos, input_data );
				std::copy( input_data->open_or_closed_mesh_data.begin(), input_data->open_or_closed_mesh_data.end(), std::back_inserter(item_data->closed_mesh_data) );
			}
			else if( dynamic_pointer_cast<IfcOpenShell>( shell_select ) )
			{
				shared_ptr<IfcOpenShell> open_shell = dynamic_pointer_cast<IfcOpenShell>( shell_select );
				std::vector<shared_ptr<IfcFace> >& vec_ifc_faces = open_shell->m_CfsFaces;

				shared_ptr<ItemData> input_data( new ItemData() );
				m_face_converter->convertIfcFaceList( vec_ifc_faces, pos, input_data );

				std::copy( input_data->open_or_closed_mesh_data.begin(), input_data->open_or_closed_mesh_data.end(), std::back_inserter(item_data->open_mesh_data) );
			}
		}
		return;
	}

	shared_ptr<IfcSurface> surface = dynamic_pointer_cast<IfcSurface>(geom_item);
	if( surface )
	{
		shared_ptr<carve::input::PolylineSetData> polyline( new carve::input::PolylineSetData() );
		m_face_converter->convertIfcSurface( surface, pos, polyline );
		if( polyline->getVertexCount() > 1 )
		{
			item_data->polyline_data.push_back( polyline );
		}
		return;
	}
	
	shared_ptr<IfcPolyline> poly_line = dynamic_pointer_cast<IfcPolyline>(geom_item);
	if( poly_line )
	{
		std::vector<carve::geom::vector<3> > poly_vertices;
		m_curve_converter->convertIfcPolyline( poly_line, poly_vertices );

		const unsigned int num_points = poly_vertices.size();
		shared_ptr<carve::input::PolylineSetData> polyline_data( new carve::input::PolylineSetData() );
		
		polyline_data->beginPolyline();

		// apply position
		for( unsigned int i=0; i<num_points; ++i )
		{
			carve::geom::vector<3>& vertex = poly_vertices.at(i);
			vertex = pos*vertex;

			polyline_data->addVertex(vertex);
			polyline_data->addPolylineIndex(i);
		}
		item_data->polyline_data.push_back(polyline_data);

		return;
	}

	shared_ptr<IfcGeometricSet> geometric_set = dynamic_pointer_cast<IfcGeometricSet>(geom_item);
	if( geometric_set )
	{
		// ENTITY IfcGeometricSet SUPERTYPE OF(IfcGeometricCurveSet)
		std::vector<shared_ptr<IfcGeometricSetSelect> >& geom_set_elements = geometric_set->m_Elements;
		std::vector<shared_ptr<IfcGeometricSetSelect> >::iterator it_set_elements;
		for( it_set_elements=geom_set_elements.begin(); it_set_elements != geom_set_elements.end(); ++it_set_elements )
		{
			// TYPE IfcGeometricSetSelect = SELECT (IfcPoint, IfcCurve, IfcSurface);
			shared_ptr<IfcGeometricSetSelect>& geom_select = (*it_set_elements);

			shared_ptr<IfcPoint> select_point = dynamic_pointer_cast<IfcPoint>(geom_select);
			if( select_point )
			{

				continue;
			}

			shared_ptr<IfcCurve> select_curve = dynamic_pointer_cast<IfcCurve>(geom_select);
			if( select_curve )
			{
				//convertIfcCurve( select_curve )
				continue;
			}

			shared_ptr<IfcSurface> select_surface = dynamic_pointer_cast<IfcSurface>(geom_select);
			if( select_surface )
			{
				shared_ptr<carve::input::PolylineSetData> polyline( new carve::input::PolylineSetData() );
				m_face_converter->convertIfcSurface( select_surface, pos, polyline );
				if( polyline->getVertexCount() > 1 )
				{
					item_data->polyline_data.push_back( polyline );
				}

				//convertIfcSurface( select_surface, pos, input_data );
				continue;
			}
		}

		shared_ptr<IfcGeometricCurveSet> geometric_curve_set = dynamic_pointer_cast<IfcGeometricCurveSet>(geometric_set);
		if( geometric_curve_set )
		{
			std::cout << "IfcGeometricCurveSet not implemented" << std::endl;	
			return;
		}
		return;
	}

	shared_ptr<IfcSectionedSpine> sectioned_spine = dynamic_pointer_cast<IfcSectionedSpine>(geom_item);
	if( sectioned_spine )
	{
		convertIfcSectionedSpine( sectioned_spine, pos, item_data );
		return;
	}
	
	throw UnhandledRepresentationException( geom_item );
}

void RepresentationConverter::subtractOpenings( const shared_ptr<IfcElement>& ifc_element, shared_ptr<ItemData>& item_data, std::stringstream& strs_err )
{
	const int product_id = ifc_element->getId();
	double length_factor = m_unit_converter->getLengthInMeterFactor();

	// now go through all meshsets of the item
	for( int i=0; i<item_data->meshsets.size(); ++i )
	{
		shared_ptr<carve::mesh::MeshSet<3> >& product_meshset = item_data->meshsets[i];

		std::vector<weak_ptr<IfcRelVoidsElement> > vec_rel_voids( ifc_element->m_HasOpenings_inverse );

		if( vec_rel_voids.size() > 0 )
		{
			shared_ptr<carve::mesh::MeshSet<3> > meshset_openings;
			for( int i=0; i<vec_rel_voids.size(); ++i )
			{
				shared_ptr<IfcRelVoidsElement> rel_voids( vec_rel_voids[i] );
				shared_ptr<IfcFeatureElementSubtraction> opening = rel_voids->m_RelatedOpeningElement;
				if( !opening )
				{
					continue;
				}
				if( !opening->m_Representation )
				{
					continue;
				}

				int opening_id = opening->getId();

				// opening can have its own relative placement
				shared_ptr<IfcObjectPlacement>	opening_placement = opening->m_ObjectPlacement;			//optional
				carve::math::Matrix opening_placement_matrix( carve::math::Matrix::IDENT() );
				if( opening_placement )
				{
					std::set<int> opening_placements_applied;
					PlacementConverter::convertIfcObjectPlacement( opening_placement, opening_placement_matrix, length_factor, opening_placements_applied );
				}

				carve::csg::CSG csg;
				std::vector<shared_ptr<IfcRepresentation> >& vec_opening_representations = opening->m_Representation->m_Representations;
				std::vector<shared_ptr<IfcRepresentation> >::iterator it_representations;
				for( it_representations=vec_opening_representations.begin(); it_representations!=vec_opening_representations.end(); ++it_representations )
				{
					shared_ptr<IfcRepresentation> opening_representation = (*it_representations);

					// TODO: bounding box test
					shared_ptr<RepresentationData> opening_representation_data( new RepresentationData() );
					try
					{
						// TODO: Representation caching, one element could be used for several openings
						std::set<int> visited_representation;
						//m_representation_converter->convertIfcRepresentation( opening_representation, opening_placement_matrix, opening_representation_data, visited_representation );
						convertIfcRepresentation( opening_representation, opening_placement_matrix, opening_representation_data, visited_representation );
					}
					catch( IfcPPException& e )
					{
						strs_err << e.what();
					}
					catch( carve::exception& ce )
					{
						strs_err << ce.str();
					}
					catch( std::exception& e )
					{
						strs_err << e.what();
					}
					catch(...)
					{
						strs_err << "subtractOpenings: convertIfcRepresentation failed at opening id " << opening_id << std::endl;
					}

					std::vector<shared_ptr<ItemData> >& opening_representation_items = opening_representation_data->vec_item_data;
					for( int i_item=0; i_item<opening_representation_items.size(); ++i_item )
					{
						shared_ptr<ItemData> opening_item_data = opening_representation_items[i_item];

						std::vector<shared_ptr<carve::input::PolyhedronData> >::iterator it_opening_polyhedron_data = opening_item_data->closed_mesh_data.begin();
						for( ; it_opening_polyhedron_data != opening_item_data->closed_mesh_data.end(); ++it_opening_polyhedron_data )
						{
							shared_ptr<carve::input::PolyhedronData>& polyhedron_data = (*it_opening_polyhedron_data);
							if( polyhedron_data->getVertexCount() < 3 )
							{
								continue;
							}

							carve::input::Options carve_options;
							shared_ptr<carve::mesh::MeshSet<3> > opening_meshset_single( polyhedron_data->createMesh(carve_options) );

							bool meshset_ok = ConverterOSG::checkMeshSet( opening_meshset_single, strs_err, product_id );
							if( !meshset_ok )
							{
								// error meshset is already written to strs_err
								continue;
							}

							if( !meshset_openings )
							{
								meshset_openings = opening_meshset_single;
								continue;
							}
								
							if( meshset_openings->meshes.size() == 0 || opening_meshset_single->meshes.size() == 0 )
							{
								strs_err << "subtractOpenings: meshset_openings->meshes.size() == " << meshset_openings->meshes.size() 
									<< ", opening_meshset_single->meshes.size() == " << opening_meshset_single->meshes.size() << std::endl;
								continue;
							}

							bool csg_operation_successful = true;
							try
							{
								meshset_openings =  shared_ptr<carve::mesh::MeshSet<3> >( csg.compute( meshset_openings.get(), opening_meshset_single.get(), carve::csg::CSG::UNION, NULL, carve::csg::CSG::CLASSIFY_NORMAL) );
								bool result_meshset_ok = ConverterOSG::checkMeshSet( meshset_openings, strs_err, product_id );
#ifdef _DEBUG
								if( !result_meshset_ok )
								{
									renderMeshsetInDebugViewer( m_debug_view, meshset_openings, osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f), true );
									renderMeshsetInDebugViewer( m_debug_view, opening_meshset_single, osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f), true );
								}
#endif
							}
							catch( IfcPPException& e )
							{
								strs_err << e.what();
								csg_operation_successful = false;
							}
							catch( carve::exception& ce )
							{
								strs_err << ce.str();
								csg_operation_successful = false;
							}
							catch( std::exception& e )
							{
								strs_err << e.what();
								csg_operation_successful = false;
							}
							catch(...)
							{
								strs_err << "RepresentationConverter::subtractOpenings: opening csg operation failed at product id " << product_id << std::endl;
								csg_operation_successful = false;
							}

							if( !csg_operation_successful )
							{
								strs_err << "RepresentationConverter::subtractOpenings: csg operation failed at product id " << product_id << std::endl;
#ifdef _DEBUG
								renderMeshsetInDebugViewer( m_debug_view, product_meshset, osg::Vec4f( 1.f, 0.f, 0.f, 1.f ), true );
#endif
							}
						}
					}
				}
			}
					
#ifdef _DEBUG
			bool opening_polyhedron_ok = ConverterOSG::checkMeshSet( meshset_openings, strs_err, product_id );
			if( !opening_polyhedron_ok )
			{
				continue;
			}
#endif

			// do the subtraction
			carve::csg::CSG csg;
			bool csg_operation_successful = true;
			try
			{
				if( product_meshset->meshes.size() && meshset_openings->meshes.size() > 0 )
				{
					shared_ptr<carve::mesh::MeshSet<3> > result( csg.compute( product_meshset.get(), meshset_openings.get(), carve::csg::CSG::A_MINUS_B, NULL, carve::csg::CSG::CLASSIFY_NORMAL) );
					bool result_meshset_ok = ConverterOSG::checkMeshSet( result, strs_err, product_id );
					product_meshset = result;
#ifdef _DEBUG
					if( !result_meshset_ok )
					{
						renderMeshsetInDebugViewer( m_debug_view, product_meshset, osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f), true );
						renderMeshsetInDebugViewer( m_debug_view, meshset_openings, osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f), true );
					}
#endif
				}
			}
			catch( IfcPPException& e )
			{
				strs_err << e.what();
				csg_operation_successful = false;
			}
			catch( carve::exception& ce )
			{
				strs_err << ce.str();
				csg_operation_successful = false;
			}
			catch( std::exception& e )
			{
				strs_err << e.what();
				csg_operation_successful = false;
			}
			catch(...)
			{
				csg_operation_successful = false;
			}
						
			if( !csg_operation_successful )
			{
				strs_err << "RepresentationConverter::subtractOpenings: csg operation failed at product id " << product_id << std::endl;
#ifdef _DEBUG
				renderMeshsetInDebugViewer( m_debug_view, product_meshset, osg::Vec4f( 1.f, 0.f, 0.f, 1.f ), true );
#endif
			}
		}
	
		//osg::ref_ptr<osg::Geode> geode_result = new osg::Geode();
		//ConverterOSG::drawMeshSet( product_meshset, geode_result );
		//item_group->addChild(geode_result);
	}
}

void RepresentationConverter::convertIfcPropertySet( const shared_ptr<IfcPropertySet>& prop_set, osg::Group* group )
{
	std::vector<shared_ptr<IfcProperty> >& vec_hasProperties = prop_set->m_HasProperties;

	std::vector<shared_ptr<IfcProperty> >::iterator it;
	for( it=vec_hasProperties.begin(); it!=vec_hasProperties.end(); ++it )
	{
		shared_ptr<IfcProperty> prop = (*it);

		shared_ptr<IfcSimpleProperty> simple_property = dynamic_pointer_cast<IfcSimpleProperty>(prop);
		if( simple_property )
		{
			shared_ptr<IfcIdentifier> name = simple_property->m_Name;
			shared_ptr<IfcText> description = simple_property->m_Description;

			//ENTITY IfcSimpleProperty
			//ABSTRACT SUPERTYPE OF(ONEOF(
			//IfcPropertyBoundedValue,
			//IfcPropertyEnumeratedValue,
			//IfcPropertyListValue,
			//IfcPropertyReferenceValue,
			//IfcPropertySingleValue,
			//IfcPropertyTableValue))
			continue;
		}
		
		shared_ptr<IfcComplexProperty> complex_property = dynamic_pointer_cast<IfcComplexProperty>(prop);
		if( complex_property )
		{
			std::vector<shared_ptr<IfcProperty> >& vec_HasProperties = complex_property->m_HasProperties;
			if( !complex_property->m_UsageName ) continue;
			if( complex_property->m_UsageName->m_value.compare( "Color" ) == 0 )
			{

#ifdef IFCPP_OPENMP
				omp_set_lock(&m_writelock_styles_converter);
				osg::ref_ptr<osg::StateSet> stateset = m_styles_converter->convertIfcComplexPropertyColor( complex_property );
				omp_unset_lock(&m_writelock_styles_converter);
#else
				osg::ref_ptr<osg::StateSet> stateset = m_styles_converter->convertIfcComplexPropertyColor( complex_property );
#endif
				if( stateset.valid() )
				{
					group->setStateSet( stateset );
				}
			}
		}
	}
}

void RepresentationConverter::detailedReport( std::stringstream& strs )
{
#ifdef IFCPP_OPENMP
	omp_set_lock(&m_writelock_detailed_report);
	m_detailed_report << strs.str().c_str() << std::endl;
	omp_unset_lock(&m_writelock_detailed_report);
#else
	m_detailed_report << strs.str().c_str() << std::endl;
#endif
}
