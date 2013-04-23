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
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Depth>
#include <osg/Material>
#include <osg/LineStipple>
#include <osg/MatrixTransform>
#include <osgText/Text>
double failed_geom_offset = 10;

#include <carve/carve.hpp>
#include <carve/geom3d.hpp>
#include <carve/poly.hpp>
#include <carve/polyhedron_base.hpp>
#include <carve/faceloop.hpp>
#include <carve/input.hpp>
#include <carve/csg.hpp>
#include <carve/csg_triangulator.hpp>

#include "ifcpp/IFC4/include/IfcProductRepresentation.h"
#include "ifcpp/IFC4/include/IfcRepresentation.h"
#include "ifcpp/IFC4/include/IfcRepresentationItem.h"
#include "ifcpp/IFC4/include/IfcMappedItem.h"
#include "ifcpp/IFC4/include/IfcRepresentationMap.h"
#include "ifcpp/IFC4/include/IfcCartesianTransformationOperator.h"
#include "ifcpp/IFC4/include/IfcAxis1Placement.h"
#include "ifcpp/IFC4/include/IfcAxis2Placement.h"
#include "ifcpp/IFC4/include/IfcAxis2Placement2D.h"
#include "ifcpp/IFC4/include/IfcAxis2Placement3D.h"
#include "ifcpp/IFC4/include/IfcPlacement.h"
#include "ifcpp/IFC4/include/IfcExtrudedAreaSolid.h"
#include "ifcpp/IFC4/include/IfcLabel.h"
#include "ifcpp/IFC4/include/IfcPolyline.h"
#include "ifcpp/IFC4/include/IfcBoundingBox.h"
#include "ifcpp/IFC4/include/IfcProfileDef.h"
#include "ifcpp/IFC4/include/IfcFacetedBrep.h"
#include "ifcpp/IFC4/include/IfcEdge.h"
#include "ifcpp/IFC4/include/IfcClosedShell.h"
#include "ifcpp/IFC4/include/IfcFaceBasedSurfaceModel.h"
#include "ifcpp/IFC4/include/IfcConnectedFaceSet.h"
#include "ifcpp/IFC4/include/IfcShellBasedSurfaceModel.h"
#include "ifcpp/IFC4/include/IfcOpenShell.h"
#include "ifcpp/IFC4/include/IfcSectionedSpine.h"
#include "ifcpp/IFC4/include/IfcRevolvedAreaSolid.h"
#include "ifcpp/IFC4/include/IfcLengthMeasure.h"
#include "ifcpp/IFC4/include/IfcPositiveLengthMeasure.h"
#include "ifcpp/IFC4/include/IfcPlaneAngleMeasure.h"
#include "ifcpp/IFC4/include/IfcCartesianPoint.h"
#include "ifcpp/IFC4/include/IfcVector.h"
#include "ifcpp/IFC4/include/IfcDirection.h"
#include "ifcpp/IFC4/include/IfcStyledItem.h"
#include "ifcpp/IFC4/include/IfcProperty.h"
#include "ifcpp/IFC4/include/IfcPropertySet.h"
#include "ifcpp/IFC4/include/IfcComplexProperty.h"
#include "ifcpp/IFC4/include/IfcSimpleProperty.h"
#include "ifcpp/IFC4/include/IfcPropertySingleValue.h"
#include "ifcpp/IFC4/include/IfcIdentifier.h"
#include "ifcpp/IFC4/include/IfcInteger.h"
#include "ifcpp/IFC4/include/IfcFace.h"
#include "ifcpp/IFC4/include/IfcFaceSurface.h"
#include "ifcpp/IFC4/include/IfcFaceBound.h"
#include "ifcpp/IFC4/include/IfcBoundedCurve.h"
#include "ifcpp/IFC4/include/IfcPcurve.h"
#include "ifcpp/IFC4/include/IfcConic.h"
#include "ifcpp/IFC4/include/IfcCircle.h"
#include "ifcpp/IFC4/include/IfcEllipse.h"
#include "ifcpp/IFC4/include/IfcLine.h"
#include "ifcpp/IFC4/include/IfcOffsetCurve2D.h"
#include "ifcpp/IFC4/include/IfcOffsetCurve3D.h"
#include "ifcpp/IFC4/include/IfcCompositeCurve.h"
#include "ifcpp/IFC4/include/IfcCompositeCurveSegment.h"
#include "ifcpp/IFC4/include/IfcPolyline.h"
#include "ifcpp/IFC4/include/IfcTrimmedCurve.h"
#include "ifcpp/IFC4/include/IfcTrimmingSelect.h"
#include "ifcpp/IFC4/include/IfcParameterValue.h"
#include "ifcpp/IFC4/include/IfcBSplineCurve.h"
#include "ifcpp/IFC4/include/IfcPolyLoop.h"
#include "ifcpp/IFC4/include/IfcEdgeLoop.h"
#include "ifcpp/IFC4/include/IfcOrientedEdge.h"
#include "ifcpp/IFC4/include/IfcVertexPoint.h"
#include "ifcpp/IFC4/include/IfcVertexLoop.h"
#include "ifcpp/IFC4/include/IfcGeometricSet.h"
#include "ifcpp/IFC4/include/IfcGeometricCurveSet.h"
#include "ifcpp/IFC4/include/IfcGeometricSetSelect.h"
#include "ifcpp/IFC4/include/IfcFeatureElementSubtraction.h"
#include "ifcpp/IFC4/include/IfcRelVoidsElement.h"
#include "ifcpp/IFC4/include/IfcRationalBSplineSurfaceWithKnots.h"
#include "ifcpp/IFC4/include/IfcSweptSurface.h"
#include "ifcpp/IFC4/include/IfcCylindricalSurface.h"
#include "ifcpp/IFC4/include/IfcSurfaceOfLinearExtrusion.h"
#include "ifcpp/IFC4/include/IfcSurfaceOfRevolution.h"
#include "ifcpp/IFC4/include/IfcCurveBoundedPlane.h"
#include "ifcpp/IFC4/include/IfcCurveBoundedSurface.h"
#include "ifcpp/IFC4/include/IfcRectangularTrimmedSurface.h"
#include "ifcpp/IFC4/include/IfcPlane.h"
#include "ifcpp/IFC4/include/IfcFaceOuterBound.h"
#include "ifcpp/IFC4/include/IfcBooleanClippingResult.h"
#include "ifcpp/IFC4/include/IfcBooleanOperator.h"
#include "ifcpp/IFC4/include/IfcBooleanOperand.h"
#include "ifcpp/IFC4/include/IfcHalfSpaceSolid.h"
#include "ifcpp/IFC4/include/IfcBoxedHalfSpace.h"
#include "ifcpp/IFC4/include/IfcPolygonalBoundedHalfSpace.h"
#include "ifcpp/IFC4/include/IfcCsgPrimitive3D.h"
#include "ifcpp/IFC4/include/IfcCsgSolid.h"
#include "ifcpp/IFC4/include/IfcSweptDiskSolid.h"
#include "ifcpp/IFC4/include/IfcAdvancedBrep.h"
#include "ifcpp/IFC4/include/IfcAdvancedBrepWithVoids.h"
#include "ifcpp/IFC4/include/IfcFixedReferenceSweptAreaSolid.h"
#include "ifcpp/IFC4/include/IfcSurfaceCurveSweptAreaSolid.h"
#include "ifcpp/IFC4/include/IfcBlock.h"
#include "ifcpp/IFC4/include/IfcRectangularPyramid.h"
#include "ifcpp/IFC4/include/IfcRightCircularCone.h"
#include "ifcpp/IFC4/include/IfcRightCircularCylinder.h"
#include "ifcpp/IFC4/include/IfcSphere.h"
#include "ifcpp/IFC4/include/IfcPresentationLayerWithStyle.h"

#include "ifcpp/model/IfcPPModel.h"
#include "ifcpp/model/UnitConverter.h"
#include "ifcpp/model/IfcPPException.h"
#include "ifcpp/model/IfcPPUtil.h"

#include "Utility.h"
#include "ConverterOSG.h"
#include "PlacementConverter.h"
#include "ProfileConverter.h"
#include "StylesConverter.h"
#include "UnhandledRepresentationException.h"
#include "RecursiveCallException.h"
#include "RepresentationConverter.h"

RepresentationConverter::RepresentationConverter( shared_ptr<UnitConverter> unit_converter ) 
	: m_unit_converter(unit_converter)
{
	m_styles_converter = shared_ptr<StylesConverter>( new StylesConverter() );
	m_handle_styled_items = true;
	m_handle_layer_assignments = true;

	m_debug_group_first = new osg::Group();
	m_debug_group_second = new osg::Group();
#ifdef IFCPP_OPENMP
	omp_init_lock(&m_writelock_profile_cache);
	omp_init_lock(&m_writelock_detailed_report);
	omp_init_lock(&m_writelock_styles_converter);
#endif
}

RepresentationConverter::~RepresentationConverter()
{
#ifdef IFCPP_OPENMP
	omp_destroy_lock(&m_writelock_profile_cache);
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

void RepresentationConverter::convertIfcRepresentation( const shared_ptr<IfcRepresentation>& representation, const carve::math::Matrix& pos, shared_ptr<RepresentationData>& input_data, std::set<int>& visited )
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
					throw IfcPPException( "! dynamic_pointer_cast<IfcPlacement>( mapping_origin ) )" );
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
		throw IfcPPException( err.str().c_str() );
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
			convertIfcFaceList( vec_ifc_faces, pos, input_data_face_set );
			std::copy( input_data_face_set->open_or_closed_shell_data.begin(), input_data_face_set->open_or_closed_shell_data.end(), std::back_inserter(item_data->open_shell_data) );
		}
		
		return;
	}

	shared_ptr<IfcBooleanResult> boolean_result = dynamic_pointer_cast<IfcBooleanResult>(geom_item);
	if( boolean_result )
	{
		convertIfcBooleanResult( boolean_result, pos, item_data );
		return;
	}

	shared_ptr<IfcSolidModel> solid_model = dynamic_pointer_cast<IfcSolidModel>(geom_item);
	if( solid_model )
	{
		convertIfcSolidModel( solid_model, pos, item_data );
		return;
	}

	shared_ptr<IfcCurve> curve = dynamic_pointer_cast<IfcCurve>(geom_item);
	if( curve )
	{
		std::vector<carve::geom::vector<3> > loops;
		std::vector<carve::geom::vector<3> > segment_start_points;
		convertIfcCurve( curve, loops, segment_start_points );

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
		std::vector<shared_ptr<IfcShell> >::iterator it_shells;

		for( it_shells=vec_shells.begin(); it_shells!=vec_shells.end(); ++it_shells )
		{
			shared_ptr<IfcShell> shell_select = (*it_shells);
			if( dynamic_pointer_cast<IfcClosedShell>( shell_select ) )
			{
				shared_ptr<IfcClosedShell> closed_shell = dynamic_pointer_cast<IfcClosedShell>( shell_select );
				std::vector<shared_ptr<IfcFace> >& vec_ifc_faces = closed_shell->m_CfsFaces;

				shared_ptr<ItemData> input_data_closed_shell( new ItemData() );
				convertIfcFaceList( vec_ifc_faces, pos, input_data_closed_shell );
				std::copy( input_data_closed_shell->open_or_closed_shell_data.begin(), input_data_closed_shell->open_or_closed_shell_data.end(), std::back_inserter(item_data->closed_shell_data) );
			}
			else if( dynamic_pointer_cast<IfcOpenShell>( shell_select ) )
			{
				shared_ptr<IfcOpenShell> open_shell = dynamic_pointer_cast<IfcOpenShell>( shell_select );
				std::vector<shared_ptr<IfcFace> >& vec_ifc_faces = open_shell->m_CfsFaces;

				shared_ptr<ItemData> input_data_open_shell( new ItemData() );
				convertIfcFaceList( vec_ifc_faces, pos, input_data_open_shell );

				std::copy( input_data_open_shell->open_or_closed_shell_data.begin(), input_data_open_shell->open_or_closed_shell_data.end(), std::back_inserter(item_data->open_shell_data) );
			}
		}
		return;
	}

	shared_ptr<IfcSurface> surface = dynamic_pointer_cast<IfcSurface>(geom_item);
	if( surface )
	{
		shared_ptr<carve::input::PolylineSetData> polyline( new carve::input::PolylineSetData() );
		convertIfcSurface( surface, pos, polyline );
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
		convertIfcPolyline( poly_line, poly_vertices );

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
				convertIfcSurface( select_surface, pos, polyline );
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

// ENTITY IfcSolidModel ABSTRACT SUPERTYPE OF(ONEOF(IfcCsgSolid, IfcManifoldSolidBrep, IfcSweptAreaSolid, IfcSweptDiskSolid))
void RepresentationConverter::convertIfcSolidModel( const shared_ptr<IfcSolidModel>& solid_model, const carve::math::Matrix& pos, shared_ptr<ItemData> item_data )
{
	shared_ptr<IfcSweptAreaSolid> swept_area_solid = dynamic_pointer_cast<IfcSweptAreaSolid>(solid_model);
	if( swept_area_solid )
	{
		// ENTITY IfcSweptAreaSolid ABSTRACT SUPERTYPE OF(ONEOF(IfcExtrudedAreaSolid, IfcFixedReferenceSweptAreaSolid, IfcRevolvedAreaSolid, IfcSurfaceCurveSweptAreaSolid))
		shared_ptr<IfcExtrudedAreaSolid> extruded_area = dynamic_pointer_cast<IfcExtrudedAreaSolid>(swept_area_solid);
		if( extruded_area )
		{
			convertIfcExtrudedAreaSolid( extruded_area, pos, item_data );
			return;
		}
		
		shared_ptr<IfcFixedReferenceSweptAreaSolid> fixed_reference_swept_area_solid = dynamic_pointer_cast<IfcFixedReferenceSweptAreaSolid>(swept_area_solid);
		if( fixed_reference_swept_area_solid )
		{
			//Directrix	 : OPTIONAL IfcCurve;
			//StartParam	 : OPTIONAL IfcParameterValue;
			//EndParam	 : OPTIONAL IfcParameterValue;
			//FixedReference	 : IfcDirection;

			return;
		}

		shared_ptr<IfcRevolvedAreaSolid> revolved_area_solid = dynamic_pointer_cast<IfcRevolvedAreaSolid>(swept_area_solid);
		if( revolved_area_solid )
		{
			convertIfcRevolvedAreaSolid( revolved_area_solid, pos, item_data );
			return;
		}
		
		shared_ptr<IfcSurfaceCurveSweptAreaSolid> surface_curve_swept_area_solid = dynamic_pointer_cast<IfcSurfaceCurveSweptAreaSolid>(swept_area_solid);
		if( surface_curve_swept_area_solid )
		{
			return;
		}
		
		throw UnhandledRepresentationException( solid_model );
	}

	shared_ptr<IfcManifoldSolidBrep> manifold_solid_brep = dynamic_pointer_cast<IfcManifoldSolidBrep>(solid_model);	
	if( manifold_solid_brep )
	{
		// ENTITY IfcManifoldSolidBrep ABSTRACT SUPERTYPE OF(ONEOF(IfcAdvancedBrep, IfcFacetedBrep)
		shared_ptr<IfcClosedShell>& outer_shell = manifold_solid_brep->m_Outer;

		shared_ptr<IfcFacetedBrep> faceted_brep = dynamic_pointer_cast<IfcFacetedBrep>(solid_model);
		if( faceted_brep )
		{
			shared_ptr<IfcClosedShell> shell = faceted_brep->m_Outer;
			if( !shell )
			{
				return;
			}
			std::vector<shared_ptr<IfcFace> >& vec_ifc_faces = shell->m_CfsFaces;


			shared_ptr<ItemData> input_data_closed_shell( new ItemData() );
			convertIfcFaceList( vec_ifc_faces, pos, input_data_closed_shell );
			std::copy( input_data_closed_shell->open_or_closed_shell_data.begin(), input_data_closed_shell->open_or_closed_shell_data.end(), std::back_inserter(item_data->closed_shell_data) );

			return;
		}

		shared_ptr<IfcManifoldSolidBrep> advanced_brep = dynamic_pointer_cast<IfcManifoldSolidBrep>(solid_model);
		if( advanced_brep )
		{
			// ENTITY IfcAdvancedBrep	SUPERTYPE OF(IfcAdvancedBrepWithVoids)
			if( dynamic_pointer_cast<IfcAdvancedBrepWithVoids>(advanced_brep) )
			{
				shared_ptr<IfcAdvancedBrepWithVoids> advanced_brep_with_voids = dynamic_pointer_cast<IfcAdvancedBrepWithVoids>(solid_model);
				std::vector<shared_ptr<IfcClosedShell> >& vec_voids = advanced_brep_with_voids->m_Voids;
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
		
		return;
	}

	throw UnhandledRepresentationException( solid_model );
}

void RepresentationConverter::convertIfcBooleanResult( const shared_ptr<IfcBooleanResult>& bool_result, const carve::math::Matrix& pos, shared_ptr<ItemData> item_data )
{
	const int bool_result_id = bool_result->getId();
	
	std::stringstream err;
	shared_ptr<IfcBooleanClippingResult> boolean_clipping_result = dynamic_pointer_cast<IfcBooleanClippingResult>(bool_result);
	if( boolean_clipping_result )
	{
		shared_ptr<IfcBooleanOperator> boolean_operator = boolean_clipping_result->m_Operator;
		shared_ptr<IfcBooleanOperand> first_operand = boolean_clipping_result->m_FirstOperand;
		shared_ptr<IfcBooleanOperand> second_operand = boolean_clipping_result->m_SecondOperand;

		if( !boolean_operator || !first_operand || !second_operand )
		{
			return;
		}
		
		carve::csg::CSG::OP csg_operation = carve::csg::CSG::A_MINUS_B;
		if( boolean_operator->m_enum == IfcBooleanOperator::ENUM_UNION )
		{
			csg_operation = carve::csg::CSG::UNION;
		}
		else if( boolean_operator->m_enum == IfcBooleanOperator::ENUM_INTERSECTION )
		{
			csg_operation = carve::csg::CSG::INTERSECTION;
		}
		else if( boolean_operator->m_enum == IfcBooleanOperator::ENUM_DIFFERENCE )
		{
			csg_operation = carve::csg::CSG::A_MINUS_B;
		}

		shared_ptr<ItemData> input_data_first_operand( new ItemData() );
		convertIfcBooleanOperand( first_operand, pos, input_data_first_operand );

		std::vector<shared_ptr<carve::mesh::MeshSet<3> > > polyhedrons_first_operand( input_data_first_operand->polyhedrons );
		for( unsigned int i=0; i<input_data_first_operand->closed_shell_data.size(); ++i )
		{
			shared_ptr<carve::input::PolyhedronData>& polyhedron_data = input_data_first_operand->closed_shell_data[i];
			if( polyhedron_data->getVertexCount() < 3 )
			{
				continue;
			}

			carve::input::Options carve_options;
			shared_ptr<carve::mesh::MeshSet<3> > polyhedron( polyhedron_data->createMesh(carve_options) );
			polyhedrons_first_operand.push_back( polyhedron );

#ifdef _DEBUG
#ifndef IFCPP_OPENMP

			bool show_operands = false;
			if( show_operands )
			{
				osg::ref_ptr<osg::Geode> geode = new osg::Geode();
				ConverterOSG carve_converter;
				carve_converter.drawMeshSet( polyhedron, geode );
				m_debug_group_first->addChild(geode);
			}
#endif
#endif
		}

		shared_ptr<ItemData> input_data_second_operand( new ItemData() );
		convertIfcBooleanOperand( second_operand, pos, input_data_second_operand );

		std::vector<shared_ptr<carve::mesh::MeshSet<3> > > polyhedrons_second_operand( input_data_second_operand->polyhedrons );
		for( unsigned int i=0; i<input_data_second_operand->closed_shell_data.size(); ++i )
		{
			shared_ptr<carve::input::PolyhedronData>& polyhedron_data = input_data_second_operand->closed_shell_data[i];
			if( polyhedron_data->getVertexCount() < 3 )
			{
				continue;
			}

			carve::input::Options carve_options;
			polyhedrons_second_operand.push_back( shared_ptr<carve::mesh::MeshSet<3> >(polyhedron_data->createMesh(carve_options) ) );
		}

		// for every first operands, apply all second operands
		std::vector<shared_ptr<carve::mesh::MeshSet<3> > >::iterator it_first_operands;
		for( it_first_operands=polyhedrons_first_operand.begin(); it_first_operands!=polyhedrons_first_operand.end(); ++it_first_operands )
		{
			shared_ptr<carve::mesh::MeshSet<3> >& first_operand_poly = (*it_first_operands);

			std::vector<shared_ptr<carve::mesh::MeshSet<3> > >::iterator it_second_operands;
			for( it_second_operands=polyhedrons_second_operand.begin(); it_second_operands!=polyhedrons_second_operand.end(); ++it_second_operands )
			{
				shared_ptr<carve::mesh::MeshSet<3> >& second_operand = (*it_second_operands);

#ifdef _DEBUG
				shared_ptr<carve::poly::Polyhedron> first_op_polyhedron( carve::polyhedronFromMesh(first_operand_poly.get(), -1 ) ); // -1 takes all meshes
				std::stringstream strs;
				for( int mani=0; mani<first_op_polyhedron->manifoldCount(); ++mani )
				{
					if( !first_op_polyhedron->manifold_is_closed[mani] )
					{
						strs << "!first_operand_poly->manifold_is_closed[mani]";
					}
					if( first_op_polyhedron->manifold_is_negative[mani] )
					{
						strs << "!first_operand_poly->manifold_is_negative[mani]";
					}
				}

				shared_ptr<carve::poly::Polyhedron> second_op_polyhedron( carve::polyhedronFromMesh(second_operand.get(), -1 ) ); // -1 takes all meshes
				for( unsigned int mani=0; mani<second_op_polyhedron->manifoldCount(); ++mani )
				{
					if( !second_op_polyhedron->manifold_is_closed[mani] )
					{
						strs << "!second_operand->manifold_is_closed[mani]";
					}
					if( second_op_polyhedron->manifold_is_negative[mani] )
					{
						strs << "!second_operand->manifold_is_negative[mani]";
					}
				}
				
				detailedReport( strs );
#endif
				bool csg_failed = false;
				carve::csg::CSG csg;
				try
				{
					// TODO: switch off std::cerr output in carve in release mode
					if( first_operand_poly->meshes.size() > 0 && second_operand->meshes.size() > 0 )
					{
						shared_ptr<carve::mesh::MeshSet<3> > result_poly( csg.compute( first_operand_poly.get(), second_operand.get(), csg_operation, NULL, carve::csg::CSG::CLASSIFY_NORMAL) );
						if( result_poly->meshes.size() > 0 )
						{
							first_operand_poly = result_poly;
						}
					}
					else
					{
#ifdef _DEBUG
						std::cout << "second_operand->meshes.size() == 0" << std::endl;
#endif
					}
				}
				catch( carve::exception& ce )
				{
					csg_failed = true;
					err << ce.str();
				}
				catch(std::exception& e)
				{
					csg_failed = true;
					err << e.what();
				}
				catch(...)
				{
					csg_failed = true;
					err << "convertIfcProduct: csg operation failed" << std::endl;
				}

#ifdef _DEBUG
#ifndef IFCPP_OPENMP
				if( csg_failed )
				{
					osg::ref_ptr<osg::Geode> geode = new osg::Geode();
					ConverterOSG carve_converter;
					carve_converter.drawMeshSet( first_operand_poly, geode );
					osg::ref_ptr<osg::MatrixTransform> mt = new osg::MatrixTransform( osg::Matrix::translate(0, 0, failed_geom_offset ) );
					mt->addChild(geode);
					failed_geom_offset += 3;

					carve_converter.drawMeshSet( second_operand, geode );
					m_debug_group_second->addChild(mt);
				}
#endif
#endif
			}
		}

		// now copy processed first operands to result input data
		for( it_first_operands=polyhedrons_first_operand.begin(); it_first_operands!=polyhedrons_first_operand.end(); ++it_first_operands )
		{
			shared_ptr<carve::mesh::MeshSet<3> >& first_operand_poly = (*it_first_operands);
			item_data->polyhedrons.push_back(first_operand_poly);
		}
	}

	if( err.tellp() > 0 )
	{
		throw IfcPPException( err.str().c_str() );
	}
}

void RepresentationConverter::convertIfcBooleanOperand( const shared_ptr<IfcBooleanOperand>& operand, const carve::math::Matrix& pos, shared_ptr<ItemData> item_data )
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
		bool agreement = half_space_solid->m_AgreementFlag;
		
		shared_ptr<carve::input::PolylineSetData> surface_data ( new carve::input::PolylineSetData() );
		convertIfcSurface( base_surface, pos, surface_data );
		std::vector<carve::geom3d::Vector>& points = surface_data->points;

		// If the agreement flag is TRUE, then the subset is the one the normal points away from
		if( !agreement )
		{
			std::reverse( points.begin(), points.end() );
		}
		carve::geom3d::Vector plane_normal = computePolygonNormal( points );

		// TODO: optimize: classify against plane instead of box
		shared_ptr<carve::input::PolyhedronData> box_data( new carve::input::PolyhedronData() );
		item_data->closed_shell_data.push_back(box_data);
		std::vector<int> face_top, face_bottom;
		const int num_vertices = points.size();

		// create geometry
		double box_depth = 10000.f;
		int begin_face_index;
		for( int k=0; k<num_vertices; ++k )
		{
			carve::geom3d::Vector& point = points.at(k);
			carve::geom3d::Vector point_extruded = point - plane_normal*(box_depth);

			box_data->addVertex( point );
			box_data->addVertex( point_extruded );

			face_bottom.push_back( num_vertices*2 - 1 - k*2 );
			face_top.push_back( k*2 );

			begin_face_index = k*2;
			if( k < num_vertices - 1 )
			{
				box_data->addFace( begin_face_index, begin_face_index + 1, begin_face_index + 3, begin_face_index + 2 );
			}
		}
		box_data->addFace( num_vertices*2-2, num_vertices*2-1, 1, 0 );
		box_data->addFace(face_top.begin(), face_top.end());
		box_data->addFace(face_bottom.begin(), face_bottom.end());

		//group->setName("HalfSpaceSolid");
		
		if( dynamic_pointer_cast<IfcBoxedHalfSpace>(half_space_solid) )
		{
			shared_ptr<IfcBoxedHalfSpace> boxed_half_space = dynamic_pointer_cast<IfcBoxedHalfSpace>(half_space_solid);
			shared_ptr<IfcBoundingBox> bbox = boxed_half_space->m_Enclosure;

		}
		else if( dynamic_pointer_cast<IfcPolygonalBoundedHalfSpace>(half_space_solid) )
		{
			shared_ptr<IfcPolygonalBoundedHalfSpace> polygonal_half_space = dynamic_pointer_cast<IfcPolygonalBoundedHalfSpace>(half_space_solid);
			
			shared_ptr<IfcAxis2Placement3D>& primitive_placement = polygonal_half_space->m_Position;

			carve::math::Matrix primitive_placement_matrix( carve::math::Matrix::IDENT() );
			if( primitive_placement )
			{
				PlacementConverter::convertIfcAxis2Placement3D( primitive_placement, primitive_placement_matrix, length_factor );
				primitive_placement_matrix = pos*primitive_placement_matrix;
			}

			shared_ptr<IfcBoundedCurve> bounded_curve = polygonal_half_space->m_PolygonalBoundary;
			std::vector<carve::geom::vector<3> > loops;
			std::vector<carve::geom::vector<3> > segment_start_points;
			convertIfcCurve( bounded_curve, loops, segment_start_points );


			shared_ptr<carve::input::PolyhedronData> box_data( new carve::input::PolyhedronData() );
			item_data->closed_shell_data.push_back(box_data);

			// apply parent position
			for( unsigned int i=0; i<loops.size(); ++i )
			{
				carve::geom::vector<3>& vertex = loops.at(i);
				vertex = primitive_placement_matrix*vertex;
				box_data->addVertex( vertex );
			}

			// TODO: create faces



		}

		return;
	}
	
	shared_ptr<IfcBooleanResult> boolean_result = dynamic_pointer_cast<IfcBooleanResult>(operand);
	if( boolean_result )
	{
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

void RepresentationConverter::convertIfcExtrudedAreaSolid( const shared_ptr<IfcExtrudedAreaSolid>& extruded_area, const carve::math::Matrix& pos, shared_ptr<ItemData> item_data )
{
	if( !extruded_area->m_ExtrudedDirection )
	{
		return;
	}

	if( !extruded_area->m_Depth )
	{
		return;
	}
	double length_factor = m_unit_converter->getLengthInMeterFactor();

	// check if local coordinate system is specified for extrusion
	carve::math::Matrix extrusion_pos_matrix( pos );
	if( extruded_area->m_Position )
	{
		shared_ptr<IfcAxis2Placement3D> extrusion_position = extruded_area->m_Position;
		PlacementConverter::convertIfcAxis2Placement3D( extrusion_position, extrusion_pos_matrix, length_factor );
		extrusion_pos_matrix = pos*extrusion_pos_matrix;
	}

	// direction and length of extrusion
	const double depth = extruded_area->m_Depth->m_value*length_factor;
	carve::geom3d::Vector extrusion_vector;
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
	shared_ptr<ProfileConverter> profile_converter = getProfileConverter(swept_area);

	const std::vector<std::vector<carve::geom::vector<3> > >& paths = profile_converter->getCoordinates();
	shared_ptr<carve::input::PolyhedronData> poly_data( new carve::input::PolyhedronData );
	extrude( paths, extrusion_vector, *(poly_data.get()) );

	std::vector<carve::geom3d::Vector>& points = poly_data->points;
	for( int i=0; i<points.size(); ++i )
	{
		points[i] = extrusion_pos_matrix*points[i];
	}

	item_data->closed_shell_data.push_back(poly_data);
}

void RepresentationConverter::convertIfcRevolvedAreaSolid( const shared_ptr<IfcRevolvedAreaSolid>& revolved_area, const carve::math::Matrix& pos, shared_ptr<ItemData> item_data )
{
	if( !revolved_area->m_SweptArea )
	{
		return;
	}
	double length_factor = m_unit_converter->getLengthInMeterFactor();
	
	// local placement of revolved object
	carve::math::Matrix revolution_pos_matrix( pos );
	if( revolved_area->m_Position )
	{
		shared_ptr<IfcAxis2Placement3D> position = revolved_area->m_Position;
		PlacementConverter::convertIfcAxis2Placement3D( position, revolution_pos_matrix, length_factor );
		revolution_pos_matrix = pos*revolution_pos_matrix;
	}

	// angle and axis
	double angle_factor = m_unit_converter->getAngleInRadianFactor();
	shared_ptr<IfcProfileDef> swept_area_profile = revolved_area->m_SweptArea;
	double revolution_angle = revolved_area->m_Angle->m_value*angle_factor;

	carve::geom3d::Vector axis_location;
	carve::geom3d::Vector axis_direction;
	if(revolved_area->m_Axis)
	{
		shared_ptr<IfcAxis1Placement> axis_placement = revolved_area->m_Axis;

		if( axis_placement->m_Location )
		{
			shared_ptr<IfcCartesianPoint> location_point = axis_placement->m_Location;
			convertIfcCartesianPoint( location_point, axis_location );
		}

		if( axis_placement->m_Axis )
		{
			shared_ptr<IfcDirection> axis = axis_placement->m_Axis;
			axis_direction = carve::geom::VECTOR( axis->m_DirectionRatios[0], axis->m_DirectionRatios[1], axis->m_DirectionRatios[2] );
		}
	}

	// rotation base point is the one with the smallest distance on the rotation axis
	carve::geom3d::Vector origin;
	carve::geom3d::Vector base_point;
	closestPointOnLine( base_point, origin, axis_location, axis_direction );
	base_point *= -1;

	// swept area
	shared_ptr<ProfileConverter> profile_converter = getProfileConverter(swept_area_profile);
	const std::vector<std::vector<carve::geom::vector<3> > >& coords = profile_converter->getCoordinates();

	if( coords.size() == 0 )
	{
		throw IfcPPException("RepresentationConverter::convertIfcRevolvedAreaSolid: num_loops == 0");
	}
	if( coords[0].size() < 3 )
	{
		throw IfcPPException("RepresentationConverter::convertIfcRevolvedAreaSolid: num_polygon_points < 3");
	}

	if( revolution_angle > M_PI*2 ) revolution_angle = M_PI*2;
	if( revolution_angle < -M_PI*2 ) revolution_angle = M_PI*2;

	// TODO: calculate num segments according to length/width/height ratio and overall size of the object
	const int num_segments = 48*(abs(revolution_angle)/(2.0*M_PI));
	double angle = 0.0;
	double d_angle = revolution_angle/num_segments;

	// check if we have to change the direction
	carve::geom3d::Vector polygon_normal = computePolygonNormal( coords[0] );
	carve::geom3d::Vector pt0 = carve::math::Matrix::ROT(d_angle, axis_direction )*(coords[0][0]+base_point);
	if( polygon_normal.z*pt0.z > 0 )
	{
		angle = revolution_angle;
		d_angle = -d_angle;
	}

	shared_ptr<carve::input::PolyhedronData> polyhedron_data( new carve::input::PolyhedronData() );
	item_data->closed_shell_data.push_back(polyhedron_data);

	// create vertices
	carve::math::Matrix m;
	for( int i = 0; i <= num_segments; ++i )
	{
		m = carve::math::Matrix::ROT( angle, -axis_direction );
		for( int j=0; j<coords.size(); ++j )
		{
			const std::vector<carve::geom::vector<3> >& loop = coords[j];
			
			for( int k=0; k<loop.size(); ++k )
			{
				carve::geom3d::Vector vertex= m*(loop[k]+base_point) - base_point;
				polyhedron_data->addVertex( revolution_pos_matrix*vertex );
			}
		}
		angle += d_angle;
	}

	// front cap
	std::vector<int> front_face_loop;
	int num_polygon_points = 0;
	for( int j=0; j<coords.size(); ++j )
	{
		const std::vector<carve::geom::vector<3> >& loop = coords[j];

		for( int k=0; k<loop.size(); ++k )
		{
			front_face_loop.push_back( j*loop.size() + k );
			++num_polygon_points;
		}
	}
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
}

void RepresentationConverter::convertIfcCsgPrimitive3D(	const shared_ptr<IfcCsgPrimitive3D>& csg_primitive,	const carve::math::Matrix& pos, shared_ptr<ItemData> item_data )
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
		
		item_data->closed_shell_data.push_back( polyhedron_data );
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

		item_data->closed_shell_data.push_back( polyhedron_data );
		return;
	}

	shared_ptr<IfcRightCircularCone> right_circular_cone = dynamic_pointer_cast<IfcRightCircularCone>(csg_primitive);
	if( right_circular_cone )
	{
		double height = length_factor;
		double radius = length_factor;
		if( right_circular_cone->m_Height )
		{
			height = right_circular_cone->m_Height->m_value*length_factor;
		}
		if( right_circular_cone->m_BottomRadius )
		{
			radius = right_circular_cone->m_BottomRadius->m_value*length_factor;
		}

		polyhedron_data->addVertex( primitive_placement_matrix*carve::geom::VECTOR(0.0, 0.0, height) ); // top
		polyhedron_data->addVertex( primitive_placement_matrix*carve::geom::VECTOR(0.0, 0.0, 0.0) ); // bottom center

		const int num_segments = 24;
		double angle = 0;
		double d_angle = 2.0*M_PI/double(num_segments);
		for( int i = 0; i < num_segments; ++i )
		{
			polyhedron_data->addVertex( primitive_placement_matrix*carve::geom::VECTOR(sin(angle)*radius, cos(angle)*radius, 0.0) );
			angle += d_angle;
		}

		// outer shape
		for( int i = 0; i < num_segments-1; ++i )
		{
			polyhedron_data->addFace(0, i+3, i+2);
		}
		polyhedron_data->addFace( 0, 2, num_segments+1 );

		// bottom circle
		for( int i = 0; i < num_segments-1; ++i )
		{
			polyhedron_data->addFace(1, i+2, i+3 );
		}
		polyhedron_data->addFace(1, num_segments+1, 2 );

		item_data->closed_shell_data.push_back( polyhedron_data );
		return;
	}

	shared_ptr<IfcRightCircularCylinder> right_circular_cylinder = dynamic_pointer_cast<IfcRightCircularCylinder>(csg_primitive);
	if( right_circular_cylinder )
	{
		// TODO: implement

		//input_data.closed_shell_data.push_back( polyhedron_data );
		return;
	}

	shared_ptr<IfcSphere> sphere = dynamic_pointer_cast<IfcSphere>(csg_primitive);
	if( sphere )
	{
		// TODO: implement

		//input_data.closed_shell_data.push_back( polyhedron_data );
		return;
	}
	throw UnhandledRepresentationException(csg_primitive);
}

void RepresentationConverter::convertIfcCurve( const shared_ptr<IfcCurve>& ifc_curve, std::vector<carve::geom::vector<3> >& loops, std::vector<carve::geom::vector<3> >& segment_start_points )
{
	std::vector<shared_ptr<IfcTrimmingSelect> > trim1_vec;
	std::vector<shared_ptr<IfcTrimmingSelect> > trim2_vec;
	convertIfcCurve( ifc_curve, loops, segment_start_points, trim1_vec, trim2_vec, true );
}


void appendPointsToCurve( std::vector<carve::geom::vector<3> >& points_vec, std::vector<carve::geom::vector<3> >& target_vec )
{
	// sometimes, sense agreement is not given correctly. try to correct sense of segment if necessary
	if( target_vec.size() > 0 && points_vec.size() > 1 )
	{
		carve::geom::vector<3> first_target_point = target_vec.front();
		carve::geom::vector<3> last_target_point = target_vec.back();

		carve::geom::vector<3> first_segment_point = points_vec.front();
		carve::geom::vector<3> last_segment_point = points_vec.back();

		if( (last_target_point-first_segment_point).length() < 0.000001 )
		{
			// segment order is as expected, nothing to do
		}
		else
		{
			if( (last_target_point-last_segment_point).length() < 0.000001 )
			{
				// current segment seems to be in wrong order
				std::reverse( points_vec.begin(), points_vec.end() );
			}
			else
			{
				// maybe the current segment fits to the beginning of the target vector
				if( (first_target_point-first_segment_point).length() < 0.000001 )
				{
					std::reverse( target_vec.begin(), target_vec.end() );
				}
				else
				{
					if( (first_target_point-last_segment_point).length() < 0.000001 )
					{
						std::reverse( target_vec.begin(), target_vec.end() );
						std::reverse( points_vec.begin(), points_vec.end() );
					}
				}
			}
		}
	}

	bool omit_first = false;
	if( target_vec.size() > 0 )
	{
		carve::geom::vector<3> last_point = target_vec.back();
		carve::geom::vector<3> first_point_current_segment = points_vec.front();
		if( (last_point-first_point_current_segment).length() < 0.000001 )
		{
			omit_first = true;
		}
	}

	if( omit_first )
	{
		target_vec.insert( target_vec.end(), points_vec.begin()+1, points_vec.end() );
	}
	else
	{
		target_vec.insert( target_vec.end(), points_vec.begin(), points_vec.end() );
	}
	// TODO: handle all segments separately: std::vector<std::vector<carve::geom::vector<3> > >& target_vec
}

void RepresentationConverter::convertIfcCurve( const shared_ptr<IfcCurve>& ifc_curve, std::vector<carve::geom::vector<3> >& target_vec, std::vector<carve::geom::vector<3> >& segment_start_points,
	std::vector<shared_ptr<IfcTrimmingSelect> >& trim1_vec, std::vector<shared_ptr<IfcTrimmingSelect> >& trim2_vec, bool sense_agreement )
{
	double length_factor = m_unit_converter->getLengthInMeterFactor();
	double plane_angle_factor = m_unit_converter->getAngleInRadianFactor();

	//	ENTITY IfcCurve ABSTRACT SUPERTYPE OF	(ONEOF(IfcBoundedCurve, IfcConic, IfcLine, IfcOffsetCurve2D, IfcOffsetCurve3D, IfcPCurve))
	shared_ptr<IfcBoundedCurve> bounded_curve = dynamic_pointer_cast<IfcBoundedCurve>(ifc_curve);
	if( bounded_curve )
	{
		shared_ptr<IfcCompositeCurve> composite_curve = dynamic_pointer_cast<IfcCompositeCurve>(bounded_curve);
		if( composite_curve )
		{
			// ENTITY IfcBoundedCurve ABSTRACT SUPERTYPE OF	(ONEOF(IfcCompositeCurve, IfcPolyline, IfcTrimmedCurve, IfcBSplineCurve))
			std::vector<shared_ptr<IfcCompositeCurveSegment> > segements = composite_curve->m_Segments;
			std::vector<shared_ptr<IfcCompositeCurveSegment> >::iterator it_segments = segements.begin();
			for(; it_segments!=segements.end(); ++it_segments )
			{
				shared_ptr<IfcCompositeCurveSegment> segement = (*it_segments);
				shared_ptr<IfcCurve> segement_curve = segement->m_ParentCurve;

				std::vector<carve::geom::vector<3> > segment_vec;
				convertIfcCurve( segement_curve, segment_vec, segment_start_points );
				if( segment_vec.size() > 0 )
				{
					appendPointsToCurve( segment_vec, target_vec );
				}
			}
			return;
		}

		shared_ptr<IfcPolyline> poly_line = dynamic_pointer_cast<IfcPolyline>(ifc_curve);
		if( poly_line )
		{
			std::vector<shared_ptr<IfcCartesianPoint> >& points = poly_line->m_Points;
			if( points.size() > 0 )
			{
				convertIfcCartesianPointVector( points, target_vec );
				segment_start_points.push_back( carve::geom::VECTOR( points[0]->m_Coordinates[0]->m_value*length_factor, points[0]->m_Coordinates[1]->m_value*length_factor, 0 ) );
			}
			return;
		}

		shared_ptr<IfcTrimmedCurve> trimmed_curve = dynamic_pointer_cast<IfcTrimmedCurve>(bounded_curve);
		if( trimmed_curve )
		{
			shared_ptr<IfcCurve> basis_curve = trimmed_curve->m_BasisCurve;
			std::vector<carve::geom::vector<3> > basis_curve_points;

			std::vector<shared_ptr<IfcTrimmingSelect> > curve_trim1_vec = trimmed_curve->m_Trim1;
			std::vector<shared_ptr<IfcTrimmingSelect> > curve_trim2_vec = trimmed_curve->m_Trim2;
			bool trimmed_sense_agreement = trimmed_curve->m_SenseAgreement;

			convertIfcCurve( basis_curve, basis_curve_points, segment_start_points, curve_trim1_vec, curve_trim2_vec, trimmed_sense_agreement );
			appendPointsToCurve( basis_curve_points, target_vec );
			return;
		}

		shared_ptr<IfcBSplineCurve> bspline_curve = dynamic_pointer_cast<IfcBSplineCurve>(bounded_curve);
		if( bspline_curve )
		{
			std::vector<shared_ptr<IfcCartesianPoint> >&	points = bspline_curve->m_ControlPointsList;
			// TODO: compute bspline curve
			convertIfcCartesianPointVector( points, target_vec );
			return;
		}
		throw UnhandledRepresentationException(bounded_curve);
	}

	shared_ptr<IfcConic> conic = dynamic_pointer_cast<IfcConic>(ifc_curve);
	if( conic )
	{
		// ENTITY IfcConic ABSTRACT SUPERTYPE OF(ONEOF(IfcCircle, IfcEllipse))
		shared_ptr<IfcAxis2Placement> conic_placement = conic->m_Position;
		carve::math::Matrix conic_position_matrix( carve::math::Matrix::IDENT() );

		shared_ptr<IfcAxis2Placement2D> axis2placement2d = dynamic_pointer_cast<IfcAxis2Placement2D>( conic_placement );
		if( axis2placement2d )
		{
			PlacementConverter::convertIfcAxis2Placement2D( axis2placement2d, conic_position_matrix, length_factor );
		}
		else if( dynamic_pointer_cast<IfcAxis2Placement3D>( conic_placement ) )
		{
			shared_ptr<IfcAxis2Placement3D> axis2placement3d = dynamic_pointer_cast<IfcAxis2Placement3D>( conic_placement );
			PlacementConverter::convertIfcAxis2Placement3D( axis2placement3d, conic_position_matrix, length_factor );
		}

		shared_ptr<IfcCircle> circle = dynamic_pointer_cast<IfcCircle>(conic);
		if( circle )
		{
			double circle_radius = 0.0;
			if( circle->m_Radius )
			{
				circle_radius = circle->m_Radius->m_value*length_factor;
			}

			carve::geom::vector<3> circle_center = conic_position_matrix*carve::geom::VECTOR( 0, 0, 0 );

			double trim_angle1 = 0.0;
			double trim_angle2 = M_PI*2.0;

			// check for trimming begin
			shared_ptr<IfcParameterValue> trim_par1;
			if( trim1_vec.size() > 0 )
			{
				if( findFirstInVector( trim1_vec, trim_par1 ) )
				{
					trim_angle1 = trim_par1->m_value*plane_angle_factor;
				}
				else
				{
					shared_ptr<IfcCartesianPoint> trim_point1;
					if( findFirstInVector( trim1_vec, trim_point1 ) )
					{
						carve::geom::vector<3> trim_point;
						convertIfcCartesianPoint( trim_point1, trim_point );
						trim_angle1 = getAngleOnCircle( circle_center, circle_radius, trim_point );
					}
				}
			}

			if( trim2_vec.size() > 0 )
			{
				// check for trimming end
				shared_ptr<IfcParameterValue> trim_par2;
				if( findFirstInVector( trim2_vec, trim_par2 ) )
				{
					trim_angle2 = trim_par2->m_value*plane_angle_factor;
				}
				else
				{
					shared_ptr<IfcCartesianPoint> ifc_trim_point;
					if( findFirstInVector( trim2_vec, ifc_trim_point ) )
					{
						carve::geom::vector<3> trim_point;
						convertIfcCartesianPoint( ifc_trim_point, trim_point );
						trim_angle2 = getAngleOnCircle( circle_center, circle_radius, trim_point );
					}
				}
			}

			double start_angle = trim_angle1;
			double opening_angle = 0;

			if( sense_agreement )
			{
				if( trim_angle1 < trim_angle2 )
				{
					opening_angle = trim_angle2 - trim_angle1;
				}
				else
				{
					// circle passes 0 anlge
					opening_angle = trim_angle2 - trim_angle1 + 2.0*M_PI;
				}
			}
			else
			{
				if( trim_angle1 > trim_angle2 )
				{
					opening_angle = trim_angle2 - trim_angle1;
				}
				else
				{
					// circle passes 0 anlge
					opening_angle = trim_angle2 - trim_angle1 - 2.0*M_PI;
				}
			}

			if( opening_angle > 0 )
			{
				while( opening_angle > 2.0*M_PI )
				{
					opening_angle -= 2.0*M_PI;
				}
			}
			else
			{
				while( opening_angle < -2.0*M_PI )
				{
					opening_angle += 2.0*M_PI;
				}
			}

			int num_segments = 48*(abs(opening_angle)/(2.0*M_PI));
			if( num_segments < 4 ) num_segments = 4;
			const double circle_center_x = 0.0;
			const double circle_center_y = 0.0;
			std::vector<carve::geom::vector<3> > circle_points;
			ProfileConverter::addFullArc( circle_points, circle_radius, start_angle, opening_angle, circle_center_x, circle_center_y, num_segments );

			if( circle_points.size() > 0 )
			{
				// apply position
				for( unsigned int i=0; i<circle_points.size(); ++i )
				{
					carve::geom3d::Vector& point = circle_points.at(i);
					point = conic_position_matrix * point;
				}

				appendPointsToCurve( circle_points, target_vec );
				segment_start_points.push_back( circle_points.at(0) );
			}

			return;
		}

		shared_ptr<IfcEllipse> ellipse = dynamic_pointer_cast<IfcEllipse>(conic);
		if( ellipse )
		{
			if( ellipse->m_SemiAxis1 )
			{
				if( ellipse->m_SemiAxis2 )
				{
					double xRadius = ellipse->m_SemiAxis1->m_value*length_factor;
					double yRadius = ellipse->m_SemiAxis2->m_value*length_factor;

					double radiusMax = std::max(xRadius, yRadius);
					int segments = (int) (radiusMax*2*M_PI*10);
					if(segments < 16) segments = 16;
					if(segments > 100) segments = 100;

					// todo: implement clipping

					std::vector<carve::geom::vector<3> > circle_points;
					double angle=0;
					for(int i = 0; i < segments; ++i) 
					{
						circle_points.push_back( carve::geom::vector<3>( carve::geom::VECTOR( xRadius * cos(angle), yRadius * sin(angle), 0 ) ) );
						angle += 2*M_PI / segments;
					}

					// apply position
					for( unsigned int i=0; i<circle_points.size(); ++i )
					{
						carve::geom::vector<3>& point = circle_points.at(i);
						point = conic_position_matrix * point;
					}
					appendPointsToCurve( circle_points, target_vec );

					//if( segment_start_points != NULL )
					{
						carve::geom::vector<3> pt0 = circle_points.at(0);
						segment_start_points.push_back( pt0 );
					}
				}
			}
			return;
		}
		throw UnhandledRepresentationException(conic);
	}

	shared_ptr<IfcLine> line = dynamic_pointer_cast<IfcLine>(ifc_curve);
	if( line )
	{
		shared_ptr<IfcCartesianPoint> ifc_line_point = line->m_Pnt;
		carve::geom::vector<3> line_origin;
		convertIfcCartesianPoint( ifc_line_point, line_origin );
		
		// line: lambda(u) = line_point + u*line_direction
		shared_ptr<IfcVector> line_vec = line->m_Dir;
		if( !line_vec )
		{
			return;
		}
		shared_ptr<IfcDirection> ifc_line_direction = line_vec->m_Orientation;

		std::vector<double>& direction_ratios = ifc_line_direction->m_DirectionRatios;
		carve::geom::vector<3> line_direction;
		if( direction_ratios.size() > 1 )
		{
			if( direction_ratios.size() > 2 )
			{
				line_direction = carve::geom::VECTOR( direction_ratios[0], direction_ratios[1], direction_ratios[2] );
			}
			else
			{
				line_direction = carve::geom::VECTOR( direction_ratios[0], direction_ratios[1], 0 );
			}
		}
		line_direction.normalize();

		shared_ptr<IfcLengthMeasure> line_magnitude = line_vec->m_Magnitude;
		double line_magnitude_value = line_magnitude->m_value*length_factor;

		// check for trimming at beginning of line
		double startParameter = 0.0;
		shared_ptr<IfcParameterValue> trim_par1;
		if( findFirstInVector( trim1_vec, trim_par1 ) )
		{
			startParameter = trim_par1->m_value;
			line_origin = line_origin + line_direction*startParameter;
		}
		else
		{
			shared_ptr<IfcCartesianPoint> ifc_trim_point;
			if( findFirstInVector( trim1_vec, ifc_trim_point ) )
			{
				carve::geom::vector<3> trim_point;
				convertIfcCartesianPoint( ifc_trim_point, trim_point );

				carve::geom::vector<3> closest_point_on_line;
				closestPointOnLine( closest_point_on_line, trim_point, line_origin, line_direction );

				if( (closest_point_on_line-trim_point).length() < 0.0001 )
				{
					// trimming point is on the line
					line_origin = trim_point;
				}
			}
		}
		// check for trimming at end of line
		carve::geom::vector<3> line_end;
		shared_ptr<IfcParameterValue> trim_par2;
		if( findFirstInVector( trim2_vec, trim_par2 ) )
		{
			line_magnitude_value = trim_par2->m_value*length_factor;
			line_end = line_origin + line_direction*line_magnitude_value;
		}
		else
		{
			shared_ptr<IfcCartesianPoint> ifc_trim_point;
			if( findFirstInVector( trim2_vec, ifc_trim_point ) )
			{
				carve::geom::vector<3> trim_point;
				convertIfcCartesianPoint( ifc_trim_point, trim_point );

				carve::geom::vector<3> closest_point_on_line;
				closestPointOnLine( closest_point_on_line, trim_point, line_origin, line_direction );

				if( (closest_point_on_line-trim_point).length() < 0.0001 )
				{
					// trimming point is on the line
					line_end = trim_point;
				}
			}
		}

		std::vector<carve::geom::vector<3> > points_vec;
		points_vec.push_back( line_origin );
		points_vec.push_back( line_end );

		appendPointsToCurve( points_vec, target_vec );

		//if( segment_start_points != NULL )
		{
			segment_start_points.push_back( line_origin );
		}
		return;
	}

	shared_ptr<IfcOffsetCurve2D> offset_curve_2d = dynamic_pointer_cast<IfcOffsetCurve2D>(ifc_curve);
	if( offset_curve_2d )
	{
		// TODO: implement
		return;
	}

	shared_ptr<IfcOffsetCurve3D> offset_curve_3d = dynamic_pointer_cast<IfcOffsetCurve3D>(ifc_curve);
	if( offset_curve_3d )
	{
		// TODO: implement
		return;
	}

	shared_ptr<IfcPcurve> pcurve = dynamic_pointer_cast<IfcPcurve>(ifc_curve);
	if( pcurve )
	{
		// TODO: implement
		return;
	}

	throw UnhandledRepresentationException(ifc_curve);
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

void RepresentationConverter::convertIfcSurface( const shared_ptr<IfcSurface>& surface, const carve::math::Matrix& pos, shared_ptr<carve::input::PolylineSetData>& polyline_data )
{
	double length_factor = m_unit_converter->getLengthInMeterFactor();
	//ENTITY IfcSurface ABSTRACT SUPERTYPE OF(ONEOF(IfcBoundedSurface, IfcElementarySurface, IfcSweptSurface))

	shared_ptr<IfcBoundedSurface> bounded_surface = dynamic_pointer_cast<IfcBoundedSurface>(surface);
	if( bounded_surface )
	{
		// ENTITY IfcBoundedSurface ABSTRACT SUPERTYPE OF(ONEOF(IfcBSplineSurface, IfcCurveBoundedPlane, IfcCurveBoundedSurface, IfcRectangularTrimmedSurface))
		if( dynamic_pointer_cast<IfcBSplineSurface>(bounded_surface) )
		{
			if( dynamic_pointer_cast<IfcRationalBSplineSurfaceWithKnots>(bounded_surface) )
			{
				shared_ptr<IfcRationalBSplineSurfaceWithKnots> nurbs_surface = dynamic_pointer_cast<IfcRationalBSplineSurfaceWithKnots>(bounded_surface);
				convertIfcBSplineSurface( nurbs_surface, pos, polyline_data );
			}
		}
		else if( dynamic_pointer_cast<IfcCurveBoundedPlane>(bounded_surface) )
		{
			// ENTITY IfcCurveBoundedPlane SUBTYPE OF IfcBoundedSurface;
			shared_ptr<IfcCurveBoundedPlane> curve_bounded_plane = dynamic_pointer_cast<IfcCurveBoundedPlane>(bounded_surface);
			carve::math::Matrix curve_bounded_plane_matrix( pos );
			shared_ptr<IfcPlane>& basis_surface = curve_bounded_plane->m_BasisSurface;
			if( basis_surface )
			{
				shared_ptr<IfcAxis2Placement3D>& basis_surface_placement = basis_surface->m_Position;
				
				if( basis_surface_placement )
				{
					PlacementConverter::convertIfcAxis2Placement3D( basis_surface_placement, curve_bounded_plane_matrix, length_factor );
					curve_bounded_plane_matrix = pos*curve_bounded_plane_matrix;
				}
			}
			shared_ptr<IfcCurve>& outer_boundary = curve_bounded_plane->m_OuterBoundary;
			if( outer_boundary )
			{
				//convertIfcCurve( outer_boundary, target,  );
				// TODO: implement boundary
			}
			std::vector<shared_ptr<IfcCurve> >& vec_inner_boundaries = curve_bounded_plane->m_InnerBoundaries;
			for( unsigned int i=0; i<vec_inner_boundaries.size(); ++i )
			{
				shared_ptr<IfcCurve>& inner_curve = vec_inner_boundaries[i];
				//convertIfcCurve( outer_boundary)
				// TODO: implement boundary
			}
		}
		else if( dynamic_pointer_cast<IfcCurveBoundedSurface>(bounded_surface) )
		{
			shared_ptr<IfcCurveBoundedSurface> curve_bounded_surface = dynamic_pointer_cast<IfcCurveBoundedSurface>(bounded_surface);
			shared_ptr<IfcSurface>& basis_surface = curve_bounded_surface->m_BasisSurface;
			if( basis_surface )
			{
				convertIfcSurface( basis_surface, pos, polyline_data );
			}

			std::vector<shared_ptr<IfcBoundaryCurve> >& vec_boundaries = curve_bounded_surface->m_Boundaries;
			bool implicit_outer = curve_bounded_surface->m_ImplicitOuter;
			// TODO: implement
		}
		else if( dynamic_pointer_cast<IfcRectangularTrimmedSurface>(bounded_surface) )
		{
			shared_ptr<IfcRectangularTrimmedSurface> rectengular_trimmed_surface = dynamic_pointer_cast<IfcRectangularTrimmedSurface>(bounded_surface);

			shared_ptr<IfcSurface>& basis_surface = rectengular_trimmed_surface->m_BasisSurface;
			if( basis_surface )
			{
				convertIfcSurface( basis_surface, pos, polyline_data );
			}

			shared_ptr<IfcParameterValue>& u1 = rectengular_trimmed_surface->m_U1;
			shared_ptr<IfcParameterValue>& v1 = rectengular_trimmed_surface->m_V1;
			shared_ptr<IfcParameterValue>& u2 = rectengular_trimmed_surface->m_U2;
			shared_ptr<IfcParameterValue>& v2 = rectengular_trimmed_surface->m_V2;
			bool u_sense = rectengular_trimmed_surface->m_Usense;
			bool v_sense = rectengular_trimmed_surface->m_Vsense;
			// TODO: implement
		}
		return;
	}

	shared_ptr<IfcElementarySurface> elementary_surface = dynamic_pointer_cast<IfcElementarySurface>(surface);
	if( elementary_surface )
	{
		//ENTITY IfcElementarySurface	ABSTRACT SUPERTYPE OF(ONEOF(IfcCylindricalSurface, IfcPlane))
		shared_ptr<IfcAxis2Placement3D>& elementary_surface_placement = elementary_surface->m_Position;

		carve::math::Matrix elementary_surface_matrix( pos );
		if( elementary_surface_placement )
		{
			PlacementConverter::convertIfcAxis2Placement3D( elementary_surface_placement, elementary_surface_matrix, length_factor );
			elementary_surface_matrix = pos*elementary_surface_matrix;
		}
	
		shared_ptr<IfcPlane> elementary_surface_plane = dynamic_pointer_cast<IfcPlane>(elementary_surface);
		if( elementary_surface_plane )
		{
			//  1----0     create big rectangular plane
			//  |    |     ^ y
			//  |    |     |
			//  2----3     ---> x
			{
#ifdef _DEBUG
				float plane_span = 10.f;
#else
				float plane_span = 100000.f;
#endif

				polyline_data->beginPolyline();
				polyline_data->addVertex( elementary_surface_matrix*carve::geom::VECTOR( plane_span,  plane_span, 0.0 ));
				polyline_data->addVertex( elementary_surface_matrix*carve::geom::VECTOR(-plane_span,  plane_span, 0.0 ));
				polyline_data->addVertex( elementary_surface_matrix*carve::geom::VECTOR(-plane_span, -plane_span, 0.0 ));
				polyline_data->addVertex( elementary_surface_matrix*carve::geom::VECTOR( plane_span, -plane_span, 0.0 ));

				polyline_data->addPolylineIndex(0);
				polyline_data->addPolylineIndex(1);
				polyline_data->addPolylineIndex(2);
				polyline_data->addPolylineIndex(3);
			}
			return;
		}

		shared_ptr<IfcCylindricalSurface> cylindrical_surface = dynamic_pointer_cast<IfcCylindricalSurface>(elementary_surface);
		if( cylindrical_surface )
		{
			shared_ptr<IfcPositiveLengthMeasure> cylindrical_surface_radius = cylindrical_surface->m_Radius;
			double circle_radius = cylindrical_surface_radius->m_value;

			int num_segments = 48;
			double start_angle = 0.0;
			double opening_angle = M_PI*2.0;
			const double circle_center_x = 0.0;
			const double circle_center_y = 0.0;

			std::vector<carve::geom::vector<3> > circle_points;
			ProfileConverter::addFullArc( circle_points, circle_radius, start_angle, opening_angle, circle_center_x, circle_center_y, num_segments );

			// apply position and insert points
			polyline_data->beginPolyline();
			for( int i=0; i<circle_points.size(); ++i )
			{
				carve::geom::vector<3> point = circle_points.at(i);
				polyline_data->addVertex( elementary_surface_matrix*point );
				polyline_data->addPolylineIndex(i);
			}
			return;
		}

		throw UnhandledRepresentationException(surface);
	}
	
	shared_ptr<IfcSweptSurface> swept_surface = dynamic_pointer_cast<IfcSweptSurface>(surface);
	if( dynamic_pointer_cast<IfcSweptSurface>(surface) )
	{
		// ENTITY IfcSweptSurface	ABSTRACT SUPERTYPE OF(ONEOF(IfcSurfaceOfLinearExtrusion, IfcSurfaceOfRevolution))
		shared_ptr<IfcProfileDef>& swept_surface_profile = swept_surface->m_SweptCurve;
		shared_ptr<IfcAxis2Placement3D>& swept_surface_placement = swept_surface->m_Position;

		carve::math::Matrix swept_surface_matrix( pos );
		if( swept_surface_placement )
		{
			PlacementConverter::convertIfcAxis2Placement3D( swept_surface_placement, swept_surface_matrix, length_factor );
			swept_surface_matrix = pos*swept_surface_matrix;
		}

		shared_ptr<IfcSurfaceOfLinearExtrusion> linear_extrusion = dynamic_pointer_cast<IfcSurfaceOfLinearExtrusion>(swept_surface);
		if( linear_extrusion )
		{
			shared_ptr<IfcDirection>& linear_extrusion_direction = linear_extrusion->m_ExtrudedDirection;
			shared_ptr<IfcLengthMeasure>& linear_extrusion_depth = linear_extrusion->m_Depth;
			// TODO: implement
			return;
		}
		
		shared_ptr<IfcSurfaceOfRevolution> suface_of_revolution = dynamic_pointer_cast<IfcSurfaceOfRevolution>(swept_surface);
		if( suface_of_revolution )
		{
			// TODO: implement
			return;
		}

		throw UnhandledRepresentationException(surface);
	}
	throw UnhandledRepresentationException(surface);
}

void RepresentationConverter::convertIfcFaceList( const std::vector<shared_ptr<IfcFace> >& faces, const carve::math::Matrix& pos, shared_ptr<ItemData> item_data )
{
	shared_ptr<carve::input::PolyhedronData> poly_data( new carve::input::PolyhedronData() );

	// TODO: check if std::unordered_map is faster, hash value is concatenated string of coordinates with a certain precision
	std::map<carve::geom3d::Vector, size_t> vert_idx;
	std::map<carve::geom3d::Vector, size_t>::iterator vert_it;

	enum FaceProjectionPlane
	{
		UNDEFINED,
		XY_PLANE,
		YZ_PLANE,
		XZ_PLANE
	};

	std::vector<shared_ptr<IfcFace> >::const_iterator it_ifc_faces;
	for( it_ifc_faces=faces.begin(); it_ifc_faces!=faces.end(); ++it_ifc_faces )
	{
		const shared_ptr<IfcFace>& face = (*it_ifc_faces);
		std::vector<shared_ptr<IfcFaceBound> >& vec_bounds = face->m_Bounds;

		std::vector<std::vector<carve::geom2d::P2> > face_loops_2d;
		std::vector<std::vector<carve::geom3d::Vector> > face_loops;
		std::vector<std::vector<double> > face_loop_3rd_dim;
		bool face_loop_reversed = false;

		int i_bound = 0;
		FaceProjectionPlane face_plane = UNDEFINED;

		std::vector<shared_ptr<IfcFaceBound> >::iterator it_bounds;
		for( it_bounds=vec_bounds.begin(); it_bounds!=vec_bounds.end(); ++it_bounds, ++i_bound )
		{
			shared_ptr<IfcFaceBound> face_bound = (*it_bounds);

			std::vector<carve::geom2d::P2> path_loop;
			std::vector<double> path_loop_3rd_dim;

			// ENTITY IfcLoop SUPERTYPE OF(ONEOF(IfcEdgeLoop, IfcPolyLoop, IfcVertexLoop))
			shared_ptr<IfcLoop> loop = face_bound->m_Bound;
			if( !loop )
			{
				continue;
			}

			bool orientation = face_bound->m_Orientation;

			const shared_ptr<IfcPolyLoop> poly_loop = dynamic_pointer_cast<IfcPolyLoop>( loop );
			if( poly_loop )
			{
				const std::vector<shared_ptr<IfcCartesianPoint> >&    ifc_points = poly_loop->m_Polygon;
				std::vector<carve::geom3d::Vector> vertices_bound;

				const int num_loop_points = ifc_points.size();
				if( num_loop_points == 3 )
				{
					// TODO: insert triangle and continue
				}
				else if( num_loop_points == 4 )
				{
					// TODO: insert two triangles and continue
				}

				convertIfcCartesianPointVectorSkipDuplicates( ifc_points, vertices_bound );

				// if first and last point have same coordinates, remove last point
				while( vertices_bound.size() > 2 )
				{
					carve::geom3d::Vector& first = vertices_bound.front();
					carve::geom3d::Vector& last = vertices_bound.back();

					if( abs(first.x-last.x) < 0.00000001 )
					{
						if( abs(first.y-last.y) < 0.00000001 )
						{
							if( abs(first.z-last.z) < 0.00000001 )
							{
								vertices_bound.pop_back();
								continue;
							}
						}
					}
					break;
				}


				if( vertices_bound.size() < 3 )
				{
					if( it_bounds == vec_bounds.begin() )
					{
						break;
					}
					else
					{
						continue;
					}
				}

				if( !orientation )
				{
					std::reverse( vertices_bound.begin(), vertices_bound.end() );
				}
				carve::geom3d::Vector normal = computePolygonNormal( vertices_bound );

				if( it_bounds == vec_bounds.begin() )
				{
					double nx = abs(normal.x);
					double ny = abs(normal.y);
					double nz = abs(normal.z);
					if( nz > nx && nz >= ny )
					{
						face_plane = XY_PLANE;
					}
					else if( nx >= ny && nx >= nz )
					{
						face_plane = YZ_PLANE;
					}
					else if( ny > nx && ny >= nz )
					{
						face_plane = XZ_PLANE;
					}
					else
					{
						std::stringstream strs;
						strs << "IfcFace: unable to project to plane: nx" << nx << " ny " << ny << " nz " << nz;
						strs << "IfcPolyLoop id: " << poly_loop->getId() << std::endl;
						for( int i=0; i<vertices_bound.size(); ++i )
						{
							carve::geom3d::Vector& point = vertices_bound.at(i);
							strs << "bound vertex i " << i << "=(" << point.x << "," << point.y << "," << point.z << ")\n";
						}
						throw IfcPPException( strs.str().c_str() );
					}
				}

				// project face into 2d plane
				for( int i=0; i<vertices_bound.size(); ++i )
				{
					carve::geom3d::Vector& point = vertices_bound.at(i);

					if( face_plane == XY_PLANE )
					{
						path_loop.push_back( carve::geom::VECTOR(point.x, point.y ));
						path_loop_3rd_dim.push_back(point.z);
					}
					else if( face_plane == YZ_PLANE )
					{
						path_loop.push_back( carve::geom::VECTOR(point.y, point.z ));
						path_loop_3rd_dim.push_back(point.x);
					}
					else if( face_plane == XZ_PLANE )
					{
						path_loop.push_back( carve::geom::VECTOR(point.x, point.z ));
						path_loop_3rd_dim.push_back(point.y);
					}
				}
				
				// check winding order
				carve::geom3d::Vector normal_2d = computePolygon2DNormal( path_loop );
				if( it_bounds == vec_bounds.begin() )
				{
					if( normal_2d.z < 0 )
					{
						std::reverse( path_loop.begin(), path_loop.end() );
						face_loop_reversed = true;
					}
				}
				else
				{
					if( normal_2d.z > 0 )
					{
						std::reverse( path_loop.begin(), path_loop.end() );
					}
				}
				face_loops_2d.push_back(path_loop);
				face_loop_3rd_dim.push_back(path_loop_3rd_dim);
				continue;
			}

			shared_ptr<IfcEdgeLoop> edge_loop = dynamic_pointer_cast<IfcEdgeLoop>( loop );
			if( edge_loop )
			{
				std::vector<shared_ptr<IfcOrientedEdge> >& edge_list = edge_loop->m_EdgeList;
				std::vector<shared_ptr<IfcOrientedEdge> >::iterator it_edge;
				for( it_edge=edge_list.begin(); it_edge!=edge_list.end(); ++it_edge )
				{
					shared_ptr<IfcOrientedEdge> oriented_edge = (*it_edge);
					shared_ptr<IfcEdge> edge = oriented_edge->m_EdgeElement;

					shared_ptr<IfcVertex> edge_start = edge->m_EdgeStart;
					shared_ptr<IfcVertexPoint> edge_start_point = dynamic_pointer_cast<IfcVertexPoint>(edge_start);
					if( edge_start_point )
					{
						if( edge_start_point->m_VertexGeometry )
						{
							shared_ptr<IfcPoint> edge_start_point_geometry = edge_start_point->m_VertexGeometry;
							shared_ptr<IfcCartesianPoint> ifc_point = dynamic_pointer_cast<IfcCartesianPoint>(edge_start_point_geometry);
							if( !ifc_point )
							{
								// TODO: could be also  IfcPointOnCurve, IfcPointOnSurface
								continue;
							}



							// TODO: implement

						}
					}
					shared_ptr<IfcVertex> edge_end = edge->m_EdgeEnd;
					shared_ptr<IfcVertexPoint> edge_end_point = dynamic_pointer_cast<IfcVertexPoint>(edge_end);
					if( edge_end_point )
					{
						if( edge_end_point->m_VertexGeometry )
						{
							shared_ptr<IfcPoint> edge_point_geometry = edge_end_point->m_VertexGeometry;
							shared_ptr<IfcCartesianPoint> ifc_point = dynamic_pointer_cast<IfcCartesianPoint>(edge_point_geometry);

							if( !ifc_point )
							{
								// TODO: could be also  IfcPointOnCurve, IfcPointOnSurface
								continue;
							}

							// TODO: implement
						}
					}
				}
				face_loops_2d.push_back(path_loop);
				continue;
			}

			shared_ptr<IfcVertexLoop> vertex_loop = dynamic_pointer_cast<IfcVertexLoop>( loop );
			if( vertex_loop )
			{
				continue;

				shared_ptr<IfcVertex> v = vertex_loop->m_LoopVertex;

				shared_ptr<IfcVertexPoint> vp = dynamic_pointer_cast<IfcVertexPoint>(v);
				if( vp )
				{
					if( vp->m_VertexGeometry )
					{
						shared_ptr<IfcPoint> point = vp->m_VertexGeometry;
						shared_ptr<IfcCartesianPoint> cp = dynamic_pointer_cast<IfcCartesianPoint>(point);
						// could be also  IfcPointOnCurve, IfcPointOnSurface
					}
				}
			}
		}

		if( face_loops_2d.size() == 0 )
		{
			continue;
		}

		std::vector<std::pair<size_t, size_t> > result; // first is loop index, second is vertex index in loop
		std::vector<carve::geom2d::P2> merged;
		std::vector<carve::geom3d::Vector> merged_3d;
		std::vector<carve::triangulate::tri_idx> triangulated;

		// TODO: apply this triangulation also in ProfileConverter and ExtrudedAreaSolid

		try
		{
			result = carve::triangulate::incorporateHolesIntoPolygon(face_loops_2d);
			merged.reserve(result.size());
			for( size_t i = 0; i < result.size(); ++i )
			{
				int loop_number = result[i].first;
				int index_in_loop = result[i].second;
				carve::geom2d::P2& loop_point = face_loops_2d[loop_number][index_in_loop];
				merged.push_back( loop_point );

				// restore 3rd dimension
				if( face_loop_reversed )
				{
					index_in_loop = face_loops_2d[loop_number].size() - index_in_loop - 1;
				}

				carve::geom3d::Vector v;
				if( face_plane == XY_PLANE )
				{
					double z = face_loop_3rd_dim[loop_number][index_in_loop];
					v = carve::geom::VECTOR(	loop_point.x,	loop_point.y,	z);
				}
				else if( face_plane == YZ_PLANE )
				{
					double x = face_loop_3rd_dim[loop_number][index_in_loop];
					v = carve::geom::VECTOR(	x,	loop_point.x,	loop_point.y);
				}
				else if( face_plane == XZ_PLANE )
				{
					double y = face_loop_3rd_dim[loop_number][index_in_loop];
					v = carve::geom::VECTOR(	loop_point.x,	y,	loop_point.y);
				}
				merged_3d.push_back( pos * v );
			}
			carve::triangulate::triangulate(merged, triangulated);
			carve::triangulate::improve(merged, triangulated);

		}
		catch(...)
		{
			std::stringstream strs;
			strs << "convertIfcFaceList: carve::triangulate::incorporateHolesIntoPolygon failed " << std::endl;
			strs << "IfcFace id: " << face->getId() << std::endl;

			std::vector<std::vector<carve::geom2d::P2> >::iterator it_face_looops_2d;
			for( it_face_looops_2d = face_loops_2d.begin(); it_face_looops_2d!=face_loops_2d.end(); ++it_face_looops_2d )
			{
				std::vector<carve::geom2d::P2>& single_loop_2d = *it_face_looops_2d;
				std::vector<carve::geom2d::P2>::iterator it_face_loop;
				strs << "loop:\n";
				for( it_face_loop = single_loop_2d.begin(); it_face_loop!=single_loop_2d.end(); ++it_face_loop )
				{
					carve::geom2d::P2& loop_point = *it_face_loop;
					strs << "	bound vertex (" << loop_point.x << "," << loop_point.y << ")\n";
				}
			}
			throw IfcPPException( strs.str().c_str() );
			continue;
		}

		// now insert points to polygon, avoiding points with same coordinates
		std::map<int,int> map_merged_idx;
		for( size_t i = 0; i != merged.size(); ++i )
		{
			carve::geom3d::Vector v = merged_3d[i];
			// TODO: check if this could become a bottleneck with large meshes. try maybe string based map (x,y,z as string key)
			vert_it = vert_idx.find(v);
			if( vert_it == vert_idx.end() )
			{
				poly_data->addVertex(v);
				int vertex_id = poly_data->getVertexCount()-1;
				vert_idx[v] = vertex_id;
				map_merged_idx[i] = vertex_id;
			}
			else
			{
				// vertex already exists in polygon. remember its index for triangles
				map_merged_idx[i]=(*vert_it).second;
			}
		}
		for( size_t i = 0; i != triangulated.size(); ++i )
		{
			carve::triangulate::tri_idx triangle = triangulated[i];
			int a = triangle.a;
			int b = triangle.b;
			int c = triangle.c;

			int vertex_id_a = map_merged_idx[a];
			int vertex_id_b = map_merged_idx[b];
			int vertex_id_c = map_merged_idx[c];

#ifdef _DEBUG
			const carve::poly::Vertex<3>& v_a = poly_data->getVertex(vertex_id_a);
			const carve::poly::Vertex<3>& v_b = poly_data->getVertex(vertex_id_b);

			double dx = v_a.v[0] - v_b.v[0];
			if( abs(dx) < 0.0000001 )
			{
				double dy = v_a.v[1] - v_b.v[1];
				if( abs(dy) < 0.0000001 )
				{
					double dz = v_a.v[2] - v_b.v[2];
					if( abs(dz) < 0.0000001 )
						std::cerr << "abs(dx) < 0.00001 && abs(dy) < 0.00001 && abs(dz) < 0.00001\n";
				}
			}
#endif

			if( face_loop_reversed )
			{
				poly_data->addFace( vertex_id_a, vertex_id_c, vertex_id_b );
			}
			else
			{
				poly_data->addFace( vertex_id_a, vertex_id_b, vertex_id_c );
			}
		}
	}

	// IfcFaceList can be a closed or open shell, so let the calling function decide where to put it
	item_data->open_or_closed_shell_data.push_back( poly_data );
}

void RepresentationConverter::convertIfcPolyline( const shared_ptr<IfcPolyline>& poly_line, std::vector<carve::geom::vector<3> >& loop )
{
	convertIfcCartesianPointVector( poly_line->m_Points, loop );
}

void RepresentationConverter::convertIfcCartesianPoint( const shared_ptr<IfcCartesianPoint>& ifc_point,	carve::geom3d::Vector& point )
{
	double length_factor = m_unit_converter->getLengthInMeterFactor();
	std::vector<shared_ptr<IfcLengthMeasure> >& coords1 = ifc_point->m_Coordinates;
	if( coords1.size() > 2 )
	{
		// round to 0.1 mm
		// TODO: round only when digits are noise
		double x = int(coords1[0]->m_value*length_factor*100000)*0.00001;
		double y = int(coords1[1]->m_value*length_factor*100000)*0.00001;
		double z = int(coords1[2]->m_value*length_factor*100000)*0.00001;
		point = carve::geom::VECTOR( x, y, z );
	}
	else if( coords1.size() > 1 )
	{
		// round to 0.1 mm
		double x = int(coords1[0]->m_value*length_factor*100000)*0.00001;
		double y = int(coords1[1]->m_value*length_factor*100000)*0.00001;
		point = carve::geom::VECTOR( x, y, 0.0 );
	}
}

void RepresentationConverter::convertIfcCartesianPoint( const shared_ptr<IfcCartesianPoint>& ifc_point,	carve::geom3d::Vector& point, double length_factor )
{
	std::vector<shared_ptr<IfcLengthMeasure> >& coords1 = ifc_point->m_Coordinates;
	if( coords1.size() > 2 )
	{
		// round to 0.1 mm
		double x = int(coords1[0]->m_value*length_factor*100000)*0.00001;
		double y = int(coords1[1]->m_value*length_factor*100000)*0.00001;
		double z = int(coords1[2]->m_value*length_factor*100000)*0.00001;
		point = carve::geom::VECTOR( x, y, z );
	}
	else if( coords1.size() > 1 )
	{
		// round to 0.1 mm
		double x = int(coords1[0]->m_value*length_factor*100000)*0.00001;
		double y = int(coords1[1]->m_value*length_factor*100000)*0.00001;
		point = carve::geom::VECTOR( x, y, 0.0 );
	}
}

void RepresentationConverter::convertIfcCartesianPointVector( const std::vector<shared_ptr<IfcCartesianPoint> >& points, std::vector<carve::geom::vector<3> >& loop )
{
	double length_factor = m_unit_converter->getLengthInMeterFactor();
	const unsigned int num_points = points.size();
	for( unsigned int i_point=0; i_point < num_points; ++i_point )
	{
		std::vector<shared_ptr<IfcLengthMeasure> >& coords = points[i_point]->m_Coordinates;

		if( coords.size() > 2  )
		{
			// round to 0.1 mm
			double x = int(coords[0]->m_value*length_factor*100000)*0.00001;
			double y = int(coords[1]->m_value*length_factor*100000)*0.00001;
			double z = int(coords[2]->m_value*length_factor*100000)*0.00001;
			loop.push_back( carve::geom::VECTOR( x, y, z ) );
		}
		else if( coords.size() > 1  )
		{
			// round to 0.1 mm
			double x = int(coords[0]->m_value*length_factor*100000)*0.00001;
			double y = int(coords[1]->m_value*length_factor*100000)*0.00001;
			loop.push_back( carve::geom::VECTOR( x, y, 0.0 ) );
		}
		else
		{
			throw IfcPPException("convertIfcCartesianPointVector: ifc_pt->m_Coordinates.size() != 2");
		}
	}
}

void RepresentationConverter::convertIfcCartesianPointVectorSkipDuplicates( const std::vector<shared_ptr<IfcCartesianPoint> >& ifc_points, std::vector<carve::geom::vector<3> >& loop )
{
	double length_factor = m_unit_converter->getLengthInMeterFactor();
	std::vector<shared_ptr<IfcCartesianPoint> >::const_iterator it_cp;
	int i=0;
	carve::geom3d::Vector vertex_previous;
	for( it_cp=ifc_points.begin(); it_cp!=ifc_points.end(); ++it_cp, ++i )
	{
		shared_ptr<IfcCartesianPoint> cp = (*it_cp);
		const int cp_id = cp->getId();
		double x = 0.0, y = 0.0, z = 0.0;
		std::vector<shared_ptr<IfcLengthMeasure> >& coords = cp->m_Coordinates;


		if( coords.size() > 2 )
		{
			x = int(coords[0]->m_value*length_factor*100000)*0.00001;
			y = int(coords[1]->m_value*length_factor*100000)*0.00001;
			z = int(coords[2]->m_value*length_factor*100000)*0.00001;
		}
		else if( coords.size() > 1 )
		{
			x = int(coords[0]->m_value*length_factor*100000)*0.00001;
			y = int(coords[1]->m_value*length_factor*100000)*0.00001;
		}
		else
		{
			std::stringstream strs;
			strs << "IfcCartesianPoint (#" << cp_id << "): coords.size() < 2";
			throw IfcPPException( strs.str().c_str() );
		}

		carve::geom3d::Vector vertex( carve::geom::VECTOR( x, y, z ) );

		// skip duplicate vertices
		if( it_cp != ifc_points.begin() )
		{
			if( abs(vertex.x-vertex_previous.x) < 0.00000001 )
			{
				if( abs(vertex.y-vertex_previous.y) < 0.00000001 )
				{
					if( abs(vertex.z-vertex_previous.z) < 0.00000001 )
					{
						// TODO: is it better to report degenerated loops, or to just omit them?
						continue;
					}
				}
			}
		}
		loop.push_back( vertex );
		vertex_previous = vertex;
	}
}



// @brief: returns the corresponding angle (radian, 0 is to the right) if the given point lies on the circle. If the point does not lie on the circle, -1 is returned.
double RepresentationConverter::getAngleOnCircle( const carve::geom::vector<3>& circle_center, double circle_radius, const carve::geom::vector<3>& trim_point )
{
	double result_angle = -1.0;
	carve::geom::vector<3> center_trim_point = trim_point-circle_center;
	if( abs(center_trim_point.length() - circle_radius) < 0.0001 )
	{
		carve::geom::vector<3> center_trim_point_direction = center_trim_point;
		center_trim_point_direction.normalize();
		double cos_angle = carve::geom::dot( center_trim_point_direction, carve::geom3d::Vector( carve::geom::VECTOR( 1.0, 0, 0 ) ) );

		if( abs(cos_angle) < 0.0001 )
		{
			if( center_trim_point.y > 0 )
			{
				result_angle = M_PI_2;
			}
			else if( center_trim_point.y < 0 )
			{
				result_angle = M_PI*1.5;
			}
		}
		else
		{
			if( center_trim_point.y > 0 )
			{
				result_angle = acos( cos_angle );
			}
			else if( center_trim_point.y < 0 )
			{
				result_angle = 2.0*M_PI - acos( cos_angle );
			}
		}
	}
	return result_angle;
}

shared_ptr<ProfileConverter> RepresentationConverter::getProfileConverter( shared_ptr<IfcProfileDef>& ifc_profile )
{
	const int profile_id = ifc_profile->getId();
	
	std::map<int,shared_ptr<ProfileConverter> >::iterator it_profile_cache = m_profile_cache.find(profile_id);
	if( it_profile_cache != m_profile_cache.end() )
	{
		return it_profile_cache->second;
	}

	shared_ptr<ProfileConverter> profile_converter = shared_ptr<ProfileConverter>( new ProfileConverter( m_unit_converter ) );
	profile_converter->setProfile( ifc_profile );
#ifdef IFCPP_OPENMP
	omp_set_lock(&m_writelock_profile_cache);
	m_profile_cache[profile_id] = profile_converter;
	omp_unset_lock(&m_writelock_profile_cache);
#else
	m_profile_cache[profile_id] = profile_converter;
#endif
		
	return profile_converter;
}


// osg
void RepresentationConverter::convertIfcCartesianPoint( const shared_ptr<IfcCartesianPoint>& ifc_point,	osg::Vec3d& point, double length_factor )
{
	std::vector<shared_ptr<IfcLengthMeasure> >& coords1 = ifc_point->m_Coordinates;
	if( coords1.size() > 2 )
	{
		point.set( coords1[0]->m_value*length_factor, coords1[1]->m_value*length_factor, coords1[2]->m_value*length_factor );
	}
	else if( coords1.size() > 1 )
	{
		point.set( coords1[0]->m_value*length_factor, coords1[1]->m_value*length_factor, 0.0 );
	}
	else
	{
		point.set(0,0,0);
	}
}

void RepresentationConverter::convertIfcCartesianPointVector( const std::vector<shared_ptr<IfcCartesianPoint> >& points, osg::Vec3dArray* vertices )
{
	double length_factor = m_unit_converter->getLengthInMeterFactor();
	std::vector<shared_ptr<IfcCartesianPoint> >::const_iterator it_cp;
	for( it_cp=points.begin(); it_cp!=points.end(); ++it_cp )
	{
		shared_ptr<IfcCartesianPoint> cp = (*it_cp);

		if( !cp )
		{
			continue;
		}

		std::vector<shared_ptr<IfcLengthMeasure> >& coords = cp->m_Coordinates;
		if( coords.size() > 2 )
		{
			vertices->push_back( osg::Vec3d( coords[0]->m_value*length_factor, coords[1]->m_value*length_factor, coords[2]->m_value*length_factor ) );
		}
		else if( coords.size() > 1 )
		{
			vertices->push_back( osg::Vec3d( coords[0]->m_value*length_factor, coords[1]->m_value*length_factor, 0.0 ) );
		}
	}
}

void RepresentationConverter::convertIfcCartesianPointVector( const std::vector<shared_ptr<IfcCartesianPoint> >& points, osg::Vec3dArray* vertices, double length_factor )
{
	std::vector<shared_ptr<IfcCartesianPoint> >::const_iterator it_cp;
	for( it_cp=points.begin(); it_cp!=points.end(); ++it_cp )
	{
		shared_ptr<IfcCartesianPoint> cp = (*it_cp);

		if( !cp )
		{
			continue;
		}

		std::vector<shared_ptr<IfcLengthMeasure> >& coords = cp->m_Coordinates;
		if( coords.size() > 2 )
		{
			vertices->push_back( osg::Vec3d( coords[0]->m_value*length_factor, coords[1]->m_value*length_factor, coords[2]->m_value*length_factor ) );
		}
		else if( coords.size() > 1 )
		{
			vertices->push_back( osg::Vec3d( coords[0]->m_value*length_factor, coords[1]->m_value*length_factor, 0.0 ) );
		}
	}
}

double RepresentationConverter::getAngleOnCircle( const osg::Vec3d& circle_center, double circle_radius, const osg::Vec3d& trim_point )
{
	double result_angle = -1.0;
	osg::Vec3d center_trim_point = trim_point-circle_center;
	if( abs(center_trim_point.length2() - circle_radius*circle_radius) < 0.01 )
	{
		osg::Vec3d center_trim_point_direction = center_trim_point;
		center_trim_point_direction.normalize();
		double cos_angle = center_trim_point_direction*osg::Vec3d( 1.0, 0, 0 );

		if( abs(cos_angle) < 0.0001 )
		{
			// trim point is vertically up or down from center
			if( center_trim_point.y() > 0 )
			{
				result_angle = M_PI_2;
			}
			else if( center_trim_point.y() < 0 )
			{
				result_angle = M_PI*1.5;
			}
		}
		else
		{
			if( center_trim_point.y() > 0 )
			{
				result_angle = acos( cos_angle );
			}
			else if( center_trim_point.y() < 0 )
			{
				result_angle = 2.0*M_PI - acos( cos_angle );
			}
		}
	}
	return result_angle;
}

void RepresentationConverter::detailedReport( std::stringstream& strs )
{
#ifdef IFCPP_OPENMP
	omp_set_lock(&m_writelock_detailed_report);
	m_detailed_report << strs.str().c_str();
	omp_unset_lock(&m_writelock_detailed_report);
#else
	m_detailed_report << strs.str().c_str();
#endif
}
