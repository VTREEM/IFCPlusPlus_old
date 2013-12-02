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

#include <osgDB/ReadFile>
#include <osg/Texture2D>
#include <osg/ShapeDrawable>
#include <osg/MatrixTransform>
#include <osgText/Text>
#include <osgUtil/SmoothingVisitor>
#include <osg/CullFace>
#include <osg/Material>
#include <osg/Depth>
#include <osg/LineStipple>
#include <osg/PolygonMode>
#include <osg/PolygonOffset>
#include <osg/AnimationPath>
#include <osgGA/CameraManipulator>
#include <osgGA/AnimationPathManipulator>
#include <osgViewer/CompositeViewer>

#define _USE_MATH_DEFINES
#include <math.h>
#include <iostream>
#include <utility>
#include <sstream>

#include "carve/carve.hpp"
#include "carve/geom3d.hpp"
#include "carve/poly.hpp"
#include "carve/polyhedron_base.hpp"
#include "carve/faceloop.hpp"
#include "carve/input.hpp"
#include "carve/csg.hpp"
#include "carve/csg_triangulator.hpp"

#include "ifcpp/model/shared_ptr.h"
#include "ifcpp/model/IfcPPException.h"
#include "ifcpp/IFC4/include/IfcFace.h"
#include "RepresentationConverter.h"
#include "Utility.h"

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void WireFrameModeOn( osg::StateSet* state )
{
	osg::ref_ptr<osg::PolygonMode> polygon_mode = dynamic_cast<osg::PolygonMode*>( state->getAttribute( osg::StateAttribute::POLYGONMODE ));
	if(  !polygon_mode )
	{
		polygon_mode = new osg::PolygonMode();
		state->setAttribute( polygon_mode );	
	}
	polygon_mode->setMode(  osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE );
}
void WireFrameModeOn( osg::Node* node )
{
	if( node == NULL )
		return;

	osg::StateSet* state = node->getOrCreateStateSet();
	WireFrameModeOn( state );
}
void WireFrameModeOn( osg::Drawable* drawable )
{
	if( drawable == NULL )
		return;

	osg::StateSet* state = drawable->getOrCreateStateSet();
	WireFrameModeOn( state );
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void WireFrameModeOff( osg::StateSet* state )
{
	osg::PolygonMode *polygon_mode = dynamic_cast< osg::PolygonMode* >( state->getAttribute( osg::StateAttribute::POLYGONMODE ));

	if(  !polygon_mode )
	{
		polygon_mode = new osg::PolygonMode();
		state->setAttribute( polygon_mode );	
	}
	polygon_mode->setMode(  osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::FILL );
}
void WireFrameModeOff( osg::Node *srisdNode )
{
	if( srisdNode == NULL )
		return;

	osg::StateSet *state = srisdNode->getOrCreateStateSet();
	WireFrameModeOff( state );
}
void WireFrameModeOff( osg::Drawable* drawable )
{
	if( drawable == NULL )
		return;

	osg::StateSet *state = drawable->getOrCreateStateSet();
	WireFrameModeOff( state );
}

void HiddenLineModeOn( osg::Group* node )
{
	return;
	osg::ref_ptr<osg::StateSet> ss = node->getOrCreateStateSet(); 
	ss->setAttributeAndModes( new osg::ColorMask(false,false,false,false),osg::StateAttribute::ON); 
	ss->setBinName("RenderBin"); 
	ss->setBinNumber(1); 
	ss->setRenderBinDetails(1,"RenderBin");

	osg::ref_ptr<osg::Group> node_lines = new osg::Group();
	// TODO: create lines
	
	ss =node_lines->getOrCreateStateSet(); 
	ss->setBinName("RenderBin"); 
	ss->setBinNumber(2); 
	ss->setRenderBinDetails(2,"RenderBin"); 
	ss->setMode(GL_POLYGON_OFFSET_FILL,osg::StateAttribute::ON); 
	osg::ref_ptr<osg::PolygonOffset> po = new osg::PolygonOffset(10.0f,10.0f); 
	ss->setAttributeAndModes(po,osg::StateAttribute::ON); 

}
void HiddenLineModeOff( osg::Group* node )
{

}

void cullFrontBack( bool front, bool back, osg::StateSet* stateset )
{
	if( front )
	{
		if( back )
		{
			// cull back and front
			osg::ref_ptr<osg::CullFace> cull = new osg::CullFace( osg::CullFace::FRONT_AND_BACK );
			stateset->setAttributeAndModes( cull.get(), osg::StateAttribute::ON );
		}
		else
		{
			// cull back face off
			osg::ref_ptr<osg::CullFace> cull_back_off = new osg::CullFace( osg::CullFace::BACK );
			stateset->setAttributeAndModes( cull_back_off.get(), osg::StateAttribute::OFF );

			// cull front face on
			osg::ref_ptr<osg::CullFace> cull_front_on = new osg::CullFace( osg::CullFace::FRONT );
			stateset->setAttributeAndModes( cull_front_on.get(), osg::StateAttribute::ON );
		}
	}
	else
	{
		if( back )
		{
			// cull front face off
			osg::ref_ptr<osg::CullFace> cull_front_off = new osg::CullFace( osg::CullFace::FRONT );
			stateset->setAttributeAndModes( cull_front_off.get(), osg::StateAttribute::OFF );

			// cull back face on
			osg::ref_ptr<osg::CullFace> cull_back_on = new osg::CullFace( osg::CullFace::BACK );
			stateset->setAttributeAndModes( cull_back_on.get(), osg::StateAttribute::ON );

		}
		else
		{
			// cull back and front off
			osg::ref_ptr<osg::CullFace> cull = new osg::CullFace( osg::CullFace::FRONT_AND_BACK );
			stateset->setAttributeAndModes( cull.get(), osg::StateAttribute::OFF );
		}
	}
}

//#define COORDINATE_AXES_NO_COLORS

osg::ref_ptr<osg::Geode> createCoordinateAxes()
{
	osg::ref_ptr<osg::Geode> geode = new osg::Geode();
	osg::ref_ptr<osg::StateSet> stateset = geode->getOrCreateStateSet();
	stateset->setMode( GL_LIGHTING, osg::StateAttribute::OFF );
	float alpha = 0.5f;

	// positive axes
	{
		osg::ref_ptr<osg::Geometry> geom	= new osg::Geometry();
		geode->addDrawable( geom );

		osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array();
		vertices->push_back( osg::Vec3f( 0.0, 0.0, 0.0 ) );
		vertices->push_back( osg::Vec3f( 500.0, 0.0, 0.0 ) );

		vertices->push_back( osg::Vec3f( 0.0, 0.0, 0.0 ) );
		vertices->push_back( osg::Vec3f( 0.0, 500.0, 0.0 ) );

		vertices->push_back( osg::Vec3f( 0.0, 0.0, 0.0 ) );
		vertices->push_back( osg::Vec3f( 0.0, 0.0, 500.0 ) );

#ifndef COORDINATE_AXES_NO_COLORS
		osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array();
		colors->push_back( osg::Vec4f( 1.f,		0.f,	0.f, alpha ) );
		colors->push_back( osg::Vec4f( 1.f,		0.f,	0.f, alpha ) );
		colors->push_back( osg::Vec4f( 0.f,		0.8f,	0.f, alpha ) );
		colors->push_back( osg::Vec4f( 0.f,		0.8f,	0.f, alpha ) );
		colors->push_back( osg::Vec4f( 0.f,		0.f,	1.f, alpha ) );
		colors->push_back( osg::Vec4f( 0.f,		0.f,	1.f, alpha ) );

		geom->setColorArray( colors );
		geom->setColorBinding( osg::Geometry::BIND_PER_VERTEX );
#endif
		geom->setVertexArray( vertices );
		geom->addPrimitiveSet( new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, 6 ) );
	}

	// positive axes
	{
		osg::ref_ptr<osg::Geometry> geom	= new osg::Geometry();
		geode->addDrawable( geom );

		osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array();
		vertices->push_back( osg::Vec3f( 0.0, 0.0, 0.0 ) );
		vertices->push_back( osg::Vec3f( -500.0, 0.0, 0.0 ) );

		vertices->push_back( osg::Vec3f( 0.0, 0.0, 0.0 ) );
		vertices->push_back( osg::Vec3f( 0.0, -500.0, 0.0 ) );

		vertices->push_back( osg::Vec3f( 0.0, 0.0, 0.0 ) );
		vertices->push_back( osg::Vec3f( 0.0, 0.0, -500.0 ) );

#ifndef COORDINATE_AXES_NO_COLORS
		osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array();
		colors->push_back( osg::Vec4f( 1.f, 0.f, 0.f, alpha ) );
		colors->push_back( osg::Vec4f( 1.f, 0.f, 0.f, alpha) );
		colors->push_back( osg::Vec4f( 0.f, 1.f, 0.f, alpha ) );
		colors->push_back( osg::Vec4f( 0.f, 1.f, 0.f, alpha ) );
		colors->push_back( osg::Vec4f( 0.f, 0.f, 1.f, alpha ) );
		colors->push_back( osg::Vec4f( 0.f, 0.f, 1.f, alpha ) );

		geom->setColorArray( colors );
		geom->setColorBinding( osg::Geometry::BIND_PER_VERTEX );
#endif
		geom->setVertexArray( vertices );
		geom->addPrimitiveSet( new osg::DrawArrays( osg::PrimitiveSet::LINE_STRIP, 0, 6 ) );

		// make negative axed dotted
		osg::ref_ptr<osg::StateSet> stateset_negative = geom->getOrCreateStateSet();
		osg::ref_ptr<osg::LineStipple> linestipple = new osg::LineStipple();
		linestipple->setFactor( 2 );
		linestipple->setPattern( 0xAAAA );
		stateset_negative->setAttributeAndModes( linestipple, osg::StateAttribute::ON );
	}

	// x axis label
	bool add_x_label = false;
	if( add_x_label )
	{
		osg::ref_ptr<osgText::Text> label_x = new  osgText::Text();
		label_x->setFont( "ARIAL.TTF" );
		label_x->setAlignment( osgText::Text::RIGHT_TOP );
		label_x->setAxisAlignment( osgText::Text::SCREEN );
		label_x->setColor( osg::Vec4( 0.8, 0.0, 0.0, 1.0 ) );
		label_x->setCharacterSize( 0.5f );
		label_x->setText( "x" );
		label_x->setPosition( osg::Vec3( 1, 0, 0 ) );
		label_x->setEnableDepthWrites(false);
		geode->addDrawable( label_x );
	}
	
	return geode;
}

osg::ref_ptr<osg::Geode> createCoordinateGrid()
{
	osg::ref_ptr<osg::Geode> geode = new osg::Geode();
	osg::ref_ptr<osg::StateSet> stateset = geode->getOrCreateStateSet();
	stateset->setMode( GL_LIGHTING, osg::StateAttribute::OFF );

	{
		osg::ref_ptr<osg::Geometry> geom	= new osg::Geometry();
		geode->addDrawable( geom );

		osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array();

		for( int i=0; i<=20; ++i )
		{
			vertices->push_back( osg::Vec3f( -10, -10+i, 0.0 ) );
			vertices->push_back( osg::Vec3f(  10, -10+i, 0.0 ) );

			vertices->push_back( osg::Vec3f( -10+i, -10, 0.0 ) );
			vertices->push_back( osg::Vec3f( -10+i, 10, 0.0 ) );
		}
		
		osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array();
		colors->push_back( osg::Vec4f( 0.7f,	0.7f,	0.7f, 0.5f ) );
		geom->setColorArray( colors );
		geom->setColorBinding( osg::Geometry::BIND_OVERALL );

		geom->setVertexArray( vertices );
		geom->addPrimitiveSet( new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, vertices->size() ) );
	}

	return geode;
}

osg::ref_ptr<osg::Group> createCoordinateAxesArrows()
{
	float cone_tip = 1.2f;
	float cone_base = 1.f;
	float cone_radius = 0.06f;
	float cone_height = cone_tip - cone_base;
	osg::ref_ptr<osg::Group> group = new osg::Group();

	{
		// x
		osg::ref_ptr<osg::ShapeDrawable> cone_drawable = new osg::ShapeDrawable( new osg::Cone( osg::Vec3f( 0.f, 0.f, 0.f ), cone_radius, cone_height ) );
		osg::ref_ptr<osg::ShapeDrawable> cyl_drawable = new osg::ShapeDrawable( new osg::Cylinder( osg::Vec3f( 0.f, 0.f, -cone_base*0.5 ), cone_radius*0.2, cone_base ) );
		cone_drawable->setColor( osg::Vec4f(0.8f, 0.0f, 0.0f, 0.7f) );
		cyl_drawable->setColor( osg::Vec4f(0.8f, 0.0f, 0.0f, 0.7f) );
		osg::ref_ptr<osg::MatrixTransform> mt1 = new osg::MatrixTransform( osg::Matrix::rotate( M_PI_2, osg::Vec3d(0,1,0) )*osg::Matrix::translate(1, 0, 0) );
		osg::ref_ptr<osg::Geode> geode = new osg::Geode();
		geode->addDrawable(cone_drawable);
		geode->addDrawable(cyl_drawable);
		mt1->addChild(geode);
		group->addChild( mt1 );
	}

	{
		// y
		osg::ref_ptr<osg::Cone> cone = new osg::Cone( osg::Vec3f( 0.f, 0.f, 0.f ), cone_radius, cone_height );
		
		osg::ref_ptr<osg::ShapeDrawable> cone_drawable = new osg::ShapeDrawable( new osg::Cone( osg::Vec3f( 0.f, 0.f, 0.f ), cone_radius, cone_height ) );
		osg::ref_ptr<osg::ShapeDrawable> cyl_drawable = new osg::ShapeDrawable( new osg::Cylinder( osg::Vec3f( 0.f, 0.f, cone_base*0.5 ), cone_radius*0.2, cone_base ) );
		cone_drawable->setColor( osg::Vec4f(0.f, 0.7f, 0.f, 0.7f) );
		cyl_drawable->setColor( osg::Vec4f(0.f, 0.7f, 0.f, 0.7f) );
		
		osg::ref_ptr<osg::MatrixTransform> mt1 = new osg::MatrixTransform( osg::Matrix::rotate( -M_PI_2, osg::Vec3d(1,0,0) )*osg::Matrix::translate(0, 1, 0) );
		osg::ref_ptr<osg::MatrixTransform> mt2 = new osg::MatrixTransform( osg::Matrix::rotate( -M_PI_2, osg::Vec3d(1,0,0) )*osg::Matrix::translate(0, 1+cone_height, 0) );
		osg::ref_ptr<osg::MatrixTransform> mt_cyl = new osg::MatrixTransform( osg::Matrix::rotate( -M_PI_2, osg::Vec3d(1,0,0) ) );
		osg::ref_ptr<osg::Geode> geode = new osg::Geode();
		geode->addDrawable(cone_drawable);
		
		osg::ref_ptr<osg::Geode> geode_cyl = new osg::Geode();
		geode_cyl->addDrawable(cyl_drawable);
				
		mt1->addChild(geode);
		mt2->addChild(geode);
		mt_cyl->addChild(geode_cyl);
		group->addChild( mt1 );
		group->addChild( mt2 );
		group->addChild( mt_cyl );
	}

	{
		// z
		osg::ref_ptr<osg::ShapeDrawable> cone_drawable = new osg::ShapeDrawable( new osg::Cone( osg::Vec3f( 0.f, 0.f, 0.f ), cone_radius, cone_height ) );
		osg::ref_ptr<osg::ShapeDrawable> cyl_drawable = new osg::ShapeDrawable( new osg::Cylinder( osg::Vec3f( 0.f, 0.f, cone_base*0.5 ), cone_radius*0.2, cone_base ) );
		cone_drawable->setColor( osg::Vec4f(0.0f, 0.0f, 0.8f, 0.7f) );
		cyl_drawable->setColor( osg::Vec4f(0.0f, 0.0f, 0.8f, 0.7f) );

		osg::ref_ptr<osg::MatrixTransform> mt1 = new osg::MatrixTransform( osg::Matrix::translate(0, 0, 1) );
		osg::ref_ptr<osg::MatrixTransform> mt2 = new osg::MatrixTransform( osg::Matrix::translate(0, 0, 1+cone_height) );
		osg::ref_ptr<osg::MatrixTransform> mt3 = new osg::MatrixTransform( osg::Matrix::translate(0, 0, 1+cone_height*2) );

		osg::ref_ptr<osg::Geode> geode = new osg::Geode();
		geode->addDrawable(cone_drawable);
		osg::ref_ptr<osg::Geode> geode_cyl = new osg::Geode();
		geode_cyl->addDrawable(cyl_drawable);
		mt1->addChild(geode);
		mt2->addChild(geode);
		mt3->addChild(geode);
		group->addChild( mt1 );
		group->addChild( mt2 );
		group->addChild( mt3 );
		group->addChild( geode_cyl );
	}

	//group->getOrCreateStateSet()->setMode( GL_LIGHTING, osg::StateAttribute::ON );	
	return group;
}

osg::ref_ptr<osg::Geode> createQuarterCircles()
{
	osg::ref_ptr<osg::DrawElementsUInt> circle_x = new osg::DrawElementsUInt( osg::PrimitiveSet::TRIANGLE_FAN, 13 );
	osg::ref_ptr<osg::DrawElementsUInt> circle_y = new osg::DrawElementsUInt( osg::PrimitiveSet::TRIANGLE_FAN, 13 );
	osg::ref_ptr<osg::DrawElementsUInt> circle_z = new osg::DrawElementsUInt( osg::PrimitiveSet::TRIANGLE_FAN, 13 );
	osg::ref_ptr<osg::Vec3Array> vertices_circle = new osg::Vec3Array(39);
	osg::ref_ptr<osg::Vec3Array> normals_circle = new osg::Vec3Array(39);

	normals_circle->at(0).set(0.f,1.f,0.f);
	normals_circle->at(13).set(1.f,0.f,0.f);
	normals_circle->at(26).set(0.f,0.f,1.f);
	float angle = 0.f, angle_delta = 3.14156f/(2.f*11.f);
	circle_x->at(0)=0;
	circle_y->at(0)=13;
	circle_z->at(0)=26;
	for( int i=1; i<=12; ++i )
	{
		vertices_circle->at(i).set(		sinf(angle),	0.f,			cosf(angle));
		vertices_circle->at(i+13).set(	0.f,			cosf(angle),	sinf(angle));
		vertices_circle->at(i+26).set(	cosf(angle),	sinf(angle),	0.f);
		normals_circle->at(i).set(		0.f,			1.f,			0.f);
		normals_circle->at(i+13).set(	1.f,			0.f,			0.f);
		normals_circle->at(i+26).set(	0.f,			0.f,			1.f);
		circle_x->at(i)=i;
		circle_y->at(i)=i+13;
		circle_z->at(i)=i+26;
		angle += angle_delta;
	}

	osg::ref_ptr<osg::Geometry> geom_circle = new osg::Geometry();
	geom_circle->setVertexArray(vertices_circle);
	geom_circle->setNormalArray(normals_circle);
	geom_circle->setNormalBinding(osg::Geometry::BIND_PER_VERTEX);

	osg::ref_ptr<osg::Vec4Array> colors_circle = new osg::Vec4Array(1);
	colors_circle->at(0).set(0.5, 0.5, 0.5, 0.3);
	geom_circle->setColorArray(colors_circle);
	geom_circle->setColorBinding( osg::Geometry::BIND_OVERALL );

	geom_circle->addPrimitiveSet(circle_x);
	geom_circle->addPrimitiveSet(circle_y);
	geom_circle->addPrimitiveSet(circle_z);
	osg::ref_ptr<osg::Geode> geode_circle = new osg::Geode();
	osg::ref_ptr<osg::StateSet> stateset_circles = geode_circle->getOrCreateStateSet();
	stateset_circles->setMode( GL_LIGHTING, osg::StateAttribute::ON );
	stateset_circles->setMode( GL_BLEND, osg::StateAttribute::ON );
	stateset_circles->setRenderingHint( osg::StateSet::TRANSPARENT_BIN );
	stateset_circles->setMode( GL_DEPTH_TEST, osg::StateAttribute::ON );
	osg::ref_ptr<osg::Depth> depth = new osg::Depth();
	depth->setWriteMask( false );
	stateset_circles->setAttributeAndModes( depth, osg::StateAttribute::ON );

	geode_circle->addDrawable(geom_circle);
	return geode_circle;
}

osg::Vec3d computePolygonNormal( const osg::Vec3dArray* polygon )
{
	const int num_points = polygon->size();
	osg::Vec3d polygon_normal(0, 0, 0);
	for( int k=0; k<num_points; ++k )
	{
		const osg::Vec3d& vertex_current = polygon->at(k);
		const osg::Vec3d& vertex_next = polygon->at((k+1)%num_points);
		polygon_normal._v[0] += (vertex_current.y()-vertex_next.y() )*(vertex_current.z()+vertex_next.z() );
		polygon_normal._v[1] += (vertex_current.z()-vertex_next.z() )*(vertex_current.x()+vertex_next.x() );
		polygon_normal._v[2] += (vertex_current.x()-vertex_next.x() )*(vertex_current.y()+vertex_next.y() );
	}
	polygon_normal.normalize();
	return polygon_normal;
}
osg::Vec3f computePolygonNormal( const osg::Vec3Array* polygon )
{
	const int num_points = polygon->size();
	osg::Vec3f polygon_normal(0, 0, 0);
	for( int k=0; k<num_points; ++k )
	{
		const osg::Vec3f& vertex_current = polygon->at(k);
		const osg::Vec3f& vertex_next = polygon->at((k+1)%num_points);
		polygon_normal._v[0] += (vertex_current.y()-vertex_next.y() )*(vertex_current.z()+vertex_next.z() );
		polygon_normal._v[1] += (vertex_current.z()-vertex_next.z() )*(vertex_current.x()+vertex_next.x() );
		polygon_normal._v[2] += (vertex_current.x()-vertex_next.x() )*(vertex_current.y()+vertex_next.y() );
	}
	polygon_normal.normalize();
	return polygon_normal;
}

carve::geom::vector<3> computePolygonNormal( const std::vector<carve::geom::vector<3> >& polygon )
{
	const int num_points = polygon.size();
	carve::geom::vector<3> polygon_normal( carve::geom::VECTOR(0, 0, 0) );
	for( int k=0; k<num_points; ++k )
	{
		const carve::geom::vector<3>& vertex_current = polygon.at(k);
		const carve::geom::vector<3>& vertex_next = polygon.at((k+1)%num_points);
		polygon_normal[0] += (vertex_current.y-vertex_next.y )*(vertex_current.z+vertex_next.z );
		polygon_normal[1] += (vertex_current.z-vertex_next.z )*(vertex_current.x+vertex_next.x );
		polygon_normal[2] += (vertex_current.x-vertex_next.x )*(vertex_current.y+vertex_next.y );
	}
	polygon_normal.normalize();
	return polygon_normal;
}
carve::geom::vector<3> computePolygon2DNormal( const std::vector<carve::geom::vector<2> >& polygon )
{
	const int num_points = polygon.size();
	carve::geom::vector<3> polygon_normal( carve::geom::VECTOR(0, 0, 0) );
	for( int k=0; k<num_points; ++k )
	{
		const carve::geom::vector<2>& vertex_current = polygon.at(k);
		const carve::geom::vector<2>& vertex_next = polygon.at((k+1)%num_points);
		//polygon_normal[0] += (vertex_current.y-vertex_next.y )*(0);
		//polygon_normal[1] += (0)*(vertex_current.x+vertex_next.x );
		polygon_normal[2] += (vertex_current.x-vertex_next.x )*(vertex_current.y+vertex_next.y );
	}
	polygon_normal.normalize();
	return polygon_normal;
}

void extrude( const std::vector<std::vector<carve::geom::vector<3> > >& paths, const carve::geom::vector<3> dir, carve::input::PolyhedronData& poly_data )
{
	std::vector<std::vector<carve::geom::vector<3> > >::const_iterator it_paths;
	std::vector<carve::geom::vector<3> >::const_iterator it_loop;
	for( it_paths=paths.begin(); it_paths!=paths.end(); ++it_paths )
	{
		const std::vector<carve::geom::vector<3> >& path = (*it_paths);
		const unsigned int num_points_in_loop = path.size();

		if( num_points_in_loop < 2 )
		{
			std::cout << "extrude: num_points_in_loop < 2"  << std::endl;
			continue;
		}

		// check if path has correct winding direction
		carve::geom::vector<3> normal = computePolygonNormal( path );

		bool reverse = false;
		if( it_paths == paths.begin() )
		{
			// polygon normal and extrusion vector should point into OPPOSITE halfspaces
			if( (normal.z*dir.z) > 0 )
			{
				reverse = true;
			}
		}
		else
		{
			// polygon normal and extrusion vector should point into SAME halfspaces
			if( (normal.z*dir.z) < 0 )
			{
				reverse = true;
			}
		}

		// TODO: incorporate holes into polygon

		std::vector<unsigned int> top_loop, bottom_loop;
		top_loop.reserve(num_points_in_loop);
		bottom_loop.reserve(num_points_in_loop);

		std::set<std::pair<size_t, size_t> > edges;

		for( it_loop=path.begin(); it_loop!=path.end(); ++it_loop )
		{
			const carve::geom3d::Vector& loop_point = (*it_loop);
			poly_data.addVertex( carve::geom::VECTOR(loop_point.x, loop_point.y, loop_point.z) + dir );
			int vertex_id = poly_data.getVertexCount()-1;
			top_loop.push_back( vertex_id );

			poly_data.addVertex( carve::geom::VECTOR(loop_point.x, loop_point.y, loop_point.z) );
			vertex_id = poly_data.getVertexCount()-1;
			bottom_loop.push_back( vertex_id );
		}
		
		if( reverse )
		{
			std::reverse( top_loop.begin(), top_loop.end() );
			std::reverse( bottom_loop.begin(), bottom_loop.end() );
		}

		poly_data.addFace(top_loop.rbegin(), top_loop.rend());
		poly_data.addFace(bottom_loop.begin(), bottom_loop.end());

		if( top_loop.size() < path.size() )
		{
			throw std::exception( "top_loop.size() < path.size()" );
		}

		for( size_t i = 0; i < path.size()-1; ++i )
		{
			edges.insert(std::make_pair(top_loop[i+1], top_loop[i]));
		}
		edges.insert(std::make_pair(top_loop[0], top_loop[num_points_in_loop-1]));

		for( size_t i = 0; i < path.size()-1; ++i )
		{
			if( edges.find(std::make_pair(top_loop[i], top_loop[i+1])) == edges.end() )
			{
				poly_data.addFace(top_loop[i], top_loop[i+1], bottom_loop[i+1], bottom_loop[i]);
			}
		}
		if( edges.find(std::make_pair(top_loop[num_points_in_loop-1], top_loop[0])) == edges.end() )
		{
			poly_data.addFace(top_loop[num_points_in_loop-1], top_loop[0], bottom_loop[0], bottom_loop[num_points_in_loop-1]);
		}
	}
}

void computeInverse( const carve::math::Matrix& matrix_a, carve::math::Matrix& inv ) 
{
	int i, j;	// col, row
	int s;		// step
	int prow;	// pivot
	int err_flag = 0;
	double factor;
	const double eps = 0.01;
	double max;
	int pivot = 1;
	double a[4][8];

	a[0][0] = matrix_a._11;
	a[0][1] = matrix_a._12;
	a[0][2] = matrix_a._13;
	a[0][3] = matrix_a._14;

	a[1][0] = matrix_a._21;
	a[1][1] = matrix_a._22;
	a[1][2] = matrix_a._23;
	a[1][3] = matrix_a._24;

	a[2][0] = matrix_a._31;
	a[2][1] = matrix_a._32;
	a[2][2] = matrix_a._33;
	a[2][3] = matrix_a._34;

	a[3][0] = matrix_a._41;
	a[3][1] = matrix_a._42;
	a[3][2] = matrix_a._43;
	a[3][3] = matrix_a._44;

	// append identity at the right
	for( i = 0; i < 4; ++i )
	{
		for( j = 0; j < 4; ++j )
		{
			a[i][4+j] = 0.0;
			if( i == j )
			{
				a[i][4+j] = 1.0;
			}
		}
	}

	s = 0;
	do
	{
		max = fabs(a[s][s]);
		if( pivot )
		{
			prow = s;
			for( i = s+1; i < 4; ++i )
			{
				if( fabs(a[i][s]) > max)
				{
					max = fabs(a[i][s]);
					prow = i;
				}
			}
		}
		err_flag = max < eps;

		if( err_flag ) break;

		if( pivot )
		{
			if( prow != s )
			{
				// change rows
				double temp;
				for( j = s ; j < 2*4; ++j )
				{
					temp = a[s][j];
					a[s][j] = a[prow][j];
					a[prow][j]= temp;
				}
			}
		}

		// elimination: divide by pivot coefficient f = a[s][s]
		factor = a[s][s];
		for( j = s; j < 2*4; ++j )
		{
			a[s][j] = a[s][j] / factor;
		}

		for( i = 0; i < 4; ++i )
		{
			if( i != s )
			{
				factor = -a[i][s];
				for( j = s; j < 2*4 ; ++j )
				{
					a[i][j] += factor*a[s][j];
				}
			}
		}
		++s;
	}
	while( s < 4 ) ;

	if( err_flag )
	{
		throw IfcPPException("cannot compute inverse of matrix");
	}
	
	inv._11 = a[0][4];
	inv._12 = a[0][5];
	inv._13 = a[0][6];
	inv._14 = a[0][7];
	
	inv._21 = a[1][4];
	inv._22 = a[1][5];
	inv._23 = a[1][6];
	inv._24 = a[1][7];

	inv._31 = a[2][4];
	inv._32 = a[2][5];
	inv._33 = a[2][6];
	inv._34 = a[2][7];

	inv._41 = a[3][4];
	inv._42 = a[3][5];
	inv._43 = a[3][6];
	inv._44 = a[3][7];
}

void closestPointOnLine( carve::geom::vector<3>& closest, const carve::geom::vector<3>& point, const carve::geom::vector<3>& line_origin, const carve::geom::vector<3>& line_direction )
{
	double denom = point.x*line_direction.x + point.y*line_direction.y + point.z*line_direction.z - line_direction.x*line_origin.x - line_direction.y*line_origin.y - line_direction.z*line_origin.z;
	double numer = line_direction.x*line_direction.x + line_direction.y*line_direction.y + line_direction.z*line_direction.z;
	if(numer == 0)
	{
		throw IfcPPException("Line is degenerated: the line's direction vector is a null vector!");
	}
	double lambda = denom/numer;
	closest = carve::geom::VECTOR(line_origin.x+lambda*line_direction.x, line_origin.y+lambda*line_direction.y, line_origin.z+lambda*line_direction.z);
}


void closestPointOnLine( osg::Vec3d& closest, const osg::Vec3d& point, const osg::Vec3d& line_origin, const osg::Vec3d& line_direction )
{
	double denom = point.x()*line_direction.x() + point.y()*line_direction.y() + point.z()*line_direction.z() - line_direction.x()*line_origin.x() - line_direction.y()*line_origin.y() - line_direction.z()*line_origin.z();
	double numer = line_direction.x()*line_direction.x() + line_direction.y()*line_direction.y() + line_direction.z()*line_direction.z();
	if(numer == 0)
	{
		throw IfcPPException("Line is degenerated: the line's direction vector is a null vector!");
	}
	double lambda = denom/numer;
	closest.set(line_origin.x()+lambda*line_direction.x(), line_origin.y()+lambda*line_direction.y(), line_origin.z()+lambda*line_direction.z());
}

//void closestPointOnLineSegment( osg::Vec3d& closest, osg::Vec3d& point, osg::Vec3d& line_origin, osg::Vec3d& line_end )
bool isPointOnLineSegment( double& target_lambda, const osg::Vec3d& point, const osg::Vec3d& line_origin, const osg::Vec3d& line_end )
{
	const osg::Vec3d line_direction = line_end - line_origin;
	const double denom = point.x()*line_direction.x() + point.y()*line_direction.y() + point.z()*line_direction.z() - line_direction.x()*line_origin.x() - line_direction.y()*line_origin.y() - line_direction.z()*line_origin.z();
	const double numer = line_direction.x()*line_direction.x() + line_direction.y()*line_direction.y() + line_direction.z()*line_direction.z();
	if(numer == 0)
	{
		throw IfcPPException("Line is degenerated: the line's direction vector is a null vector!");
	}
	const double lambda = denom/numer;
	osg::Vec3d closest( line_origin.x()+lambda*line_direction.x(), line_origin.y()+lambda*line_direction.y(), line_origin.z()+lambda*line_direction.z() );
	const double length2 = (closest-point).length2();
	if(  length2 > 0.0001 )
	{
		// point is not on line
		return false;
	}
	if( lambda < -0.0001 )
	{
		// point is not on line
		return false;
	}
	if( lambda > 1.0001 )
	{
		// point is not on line
		return false;
	}
	target_lambda = lambda;
	return true;
}

bool LineToLineIntersectionHelper(carve::geom::vector<2>& v1, carve::geom::vector<2>& v2, carve::geom::vector<2>& v3, carve::geom::vector<2>& v4, double & r, double & s)
{
	double d;
	// check if lines are parallel
	carve::geom::vector<2> vertex1to2 = v2 - v1;
	carve::geom::vector<2> vertex3to4 = v4 - v3;
	if( vertex1to2.y / vertex1to2.x != vertex3to4.y / vertex3to4.x )
	{
		d = vertex1to2.x*vertex3to4.y - vertex1to2.y*vertex3to4.x;
		if( d != 0 )
		{
			carve::geom::vector<2> vertex3to1 = v1 - v3;
			r = (vertex3to1.y*vertex3to4.x - vertex3to1.x*vertex3to4.y) / d;
			s = (vertex3to1.y*vertex1to2.x - vertex3to1.x*vertex1to2.y) / d;
			return true;
		}
	}
	return false;

}

bool LineSegmentToLineIntersection(carve::geom::vector<2>& v1, carve::geom::vector<2>& v2, carve::geom::vector<2>& v3, carve::geom::vector<2>& v4, std::vector<carve::geom::vector<2> >& result )
{
	double r, s;
	if( LineToLineIntersectionHelper(v1, v2, v3, v4, r, s) )
	{
		if (r >= 0 && r <= 1)
		{
			result.push_back(v1 + (v2 - v1) * r);
			return true;
		}
	}
	return false;
}

bool LineSegmentToLineSegmentIntersection(carve::geom::vector<2>& v1, carve::geom::vector<2>& v2, carve::geom::vector<2>& v3, carve::geom::vector<2>& v4, std::vector<carve::geom::vector<2> >& result )
{
	double r, s;
	if( LineToLineIntersectionHelper(v1, v2, v3, v4, r, s) )
	{
		if (r >= 0 && r <= 1)
		{
			if (s >= 0 && s <= 1)
			{
				result.push_back(v1 + (v2 - v1) * r);
				return true;
			}
		}
	}
	return false;
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


void makeLookAt(const carve::geom::vector<3>& eye,const carve::geom::vector<3>& center,const carve::geom::vector<3>& up, carve::math::Matrix& m )
{
	carve::geom::vector<3> f(center-eye);
    f.normalize();
    carve::geom::vector<3> s = cross(f,up);
    s.normalize();
    carve::geom::vector<3> u = cross(s, f);
    u.normalize();

	//m.m[0][0] = s[0];
	//m.m[0][1] = u[0];
	//m.m[0][2] = -f[0];
	//m.m[0][3] = 0.0;
	//m.m[1][0] = s[1];
	//m.m[1][1] = u[1];
	//m.m[1][2] = -f[1];
	//m.m[1][3] = 0.0;
	//m.m[2][0] = s[2];
	//m.m[2][1] = u[2];
	//m.m[2][2] = -f[2];
	//m.m[2][3] = 0.0;
	//m.m[3][0] = 0.0;
	//m.m[3][1] = 0.0;
	//m.m[3][2] = 0.0;
	//m.m[3][3] = 1.0;

	m._11 = s[0];
	m._12 = u[0];
	m._13 = -f[0];
	m._14 = 0.0;
	m._21 = s[1];
	m._22 = u[1];
	m._23 = -f[1];
	m._24 = 0.0;
	m._31 = s[2];
	m._32 = u[2];
	m._33 = -f[2];
	m._34 = 0.0;
	m._41 = 0.0;
	m._42 = 0.0;
	m._43 = 0.0;
	m._44 = 1.0;

	for (unsigned i = 0; i < 3; ++i)
    {
        double tmp = -eye[i];
        if (tmp == 0)
            continue;
        m.m[3][0] += tmp*m.m[i][0];
        m.m[3][1] += tmp*m.m[i][1];
        m.m[3][2] += tmp*m.m[i][2];
        m.m[3][3] += tmp*m.m[i][3];
    }
}

void makeRotate( const carve::geom::vector<3>& from, const carve::geom::vector<3>& to, carve::math::Quaternion quat )
{

    // This routine takes any vector as argument but normalized
    // vectors are necessary, if only for computing the dot product.
    // Too bad the API is that generic, it leads to performance loss.
    // Even in the case the 2 vectors are not normalized but same length,
    // the sqrt could be shared, but we have no way to know beforehand
    // at this point, while the caller may know.
    // So, we have to test... in the hope of saving at least a sqrt
    carve::geom::vector<3> sourceVector = from;
    carve::geom::vector<3> targetVector = to;

    double fromLen2 = from.length2();
    double fromLen;
    // normalize only when necessary, epsilon test
    if ((fromLen2 < 1.0-1e-7) || (fromLen2 > 1.0+1e-7)) {
        fromLen = sqrt(fromLen2);
        sourceVector /= fromLen;
    } else fromLen = 1.0;

    double toLen2 = to.length2();
    // normalize only when necessary, epsilon test
    if ((toLen2 < 1.0-1e-7) || (toLen2 > 1.0+1e-7)) {
        double toLen;
        // re-use fromLen for case of mapping 2 vectors of the same length
        if ((toLen2 > fromLen2-1e-7) && (toLen2 < fromLen2+1e-7)) {
            toLen = fromLen;
        }
        else toLen = sqrt(toLen2);
        targetVector /= toLen;
    }


    // Now let's get into the real stuff
    // Use "dot product plus one" as test as it can be re-used later on
    //double dotProdPlus1 = 1.0 + sourceVector * targetVector;
	double dotProdPlus1 = 1.0 + dot( sourceVector, targetVector );

    // Check for degenerate case of full u-turn. Use epsilon for detection
    if (dotProdPlus1 < 1e-7) {

        // Get an orthogonal vector of the given vector
        // in a plane with maximum vector coordinates.
        // Then use it as quaternion axis with pi angle
        // Trick is to realize one value at least is >0.6 for a normalized vector.
        if (fabs(sourceVector.x) < 0.6) {
            const double norm = sqrt(1.0 - sourceVector.x * sourceVector.x);
			quat.x = 0.0;
            quat.y = sourceVector.z / norm;
            quat.z = -sourceVector.y / norm;
            quat.w = 0.0;
        } else if (fabs(sourceVector.y) < 0.6) {
            const double norm = sqrt(1.0 - sourceVector.y * sourceVector.y);
            quat.x = -sourceVector.z / norm;
            quat.y = 0.0;
            quat.z = sourceVector.x / norm;
            quat.w = 0.0;
        } else {
            const double norm = sqrt(1.0 - sourceVector.z * sourceVector.z);
            quat.x = sourceVector.y / norm;
            quat.y = -sourceVector.x / norm;
            quat.z = 0.0;
            quat.w = 0.0;
        }
    }

    else {
        // Find the shortest angle quaternion that transforms normalized vectors
        // into one other. Formula is still valid when vectors are colinear
        const double s = sqrt(0.5 * dotProdPlus1);
        //const carve::geom::vector<3> tmp = sourceVector ^ targetVector / (2.0*s);
		const carve::geom::vector<3> tmp = cross(sourceVector , targetVector) / (2.0*s);
        quat.x = tmp.x;
        quat.y = tmp.y;
        quat.z = tmp.z;
        quat.w = s;
    }
}

void getMatrixRotate( const carve::math::Matrix mat, carve::math::Quaternion q )
{
    double s;
    double tq[4];
    int    i, j;

    // Use tq to store the largest trace
	tq[0] = 1 + mat._11 + mat._22 + mat._33;
    tq[1] = 1 + mat._11 - mat._22 - mat._33;
    tq[2] = 1 - mat._11 + mat._22 - mat._33;
    tq[3] = 1 - mat._11 - mat._22 + mat._33;

    // Find the maximum (could also use stacked if's later)
    j = 0;
    for(i=1;i<4;i++) j = (tq[i]>tq[j])? i : j;

    // check the diagonal
    if (j==0)
    {
        /* perform instant calculation */
		q.w = tq[0];
        q.x = mat._23 - mat._32;
        q.y = mat._31 - mat._13;
        q.z = mat._12 - mat._21;
    }
    else if (j==1)
    {
        q.w = mat._23 - mat._32;
        q.x = tq[1];
        q.y = mat._12 + mat._21;
        q.z = mat._31 + mat._13;
    }
    else if (j==2)
    {
        q.w = mat._31 - mat._13;
        q.x = mat._12 + mat._21;
        q.y = tq[2];
        q.z = mat._23 + mat._32;
    }
    else /* if (j==3) */
    {
        q.w = mat._12 - mat._21;
        q.x = mat._31 + mat._13;
        q.y = mat._23 + mat._32;
        q.z = tq[3];
    }

    s = sqrt(0.25/tq[j]);
    q.w *= s;
    q.x *= s;
    q.y *= s;
    q.z *= s;
}


bool bisectingPlane( osg::Vec3d& n, const osg::Vec3d& v1, const osg::Vec3d& v2, const osg::Vec3d& v3)
{
	bool valid = false;
	osg::Vec3d v21 = v2 - v1;
	osg::Vec3d v32 = v3 - v2;
	double len21 = v21.length();
	double len32 = v32.length();

	if( len21 <= GEOM_TOLERANCE * len32)
	{
		if( len32 == 0.0)
		{
			// all three points lie ontop of one-another
			n.set( 0.0, 0.0, 0.0 );
			valid = false;
		}
		else
		{
			// return a normalized copy of v32 as bisector
			len32 = 1.0 / len32;
			n = v32*len32;
			valid = true;
		}

	}
	else
	{
		valid = true;
		if( len32 <= GEOM_TOLERANCE * len21)
		{
			// return v21 as bisector
			v21.normalize();
			n = v21;
		}
		else
		{
			v21.normalize();
			v32.normalize();

			double dot_product = v32*v21;
			double dot_product_abs = abs( dot_product );
			
			if( dot_product_abs > (1.0+GEOM_TOLERANCE) || dot_product_abs < (1.0-GEOM_TOLERANCE) )
			{
				n = (v32 + v21)*dot_product - v32 - v21;
				n.normalize();
			}
			else
			{
				// dot == 1 or -1, points are colinear
				n = -v21;
			}
		}
	}
	return valid;
}

bool bisectingPlane( carve::geom::vector<3>& n, const carve::geom::vector<3>& v1, 
											 const carve::geom::vector<3>& v2, const carve::geom::vector<3>& v3)
{
	bool valid = false;
	carve::geom::vector<3> v21 = v2 - v1;
	carve::geom::vector<3> v32 = v3 - v2;
	double len21 = v21.length();
	double len32 = v32.length();

	if( len21 <= GEOM_TOLERANCE * len32)
	{
		if( len32 == 0.0)
		{
			// all three points lie ontop of one-another
			n = carve::geom::VECTOR( 0.0, 0.0, 0.0 );
			valid = false;
		}
		else
		{
			// return a normalized copy of v32 as bisector
			len32 = 1.0 / len32;
			n = v32*len32;
			valid = true;
		}

	}
	else
	{
		valid = true;
		if( len32 <= GEOM_TOLERANCE * len21)
		{
			// return v21 as bisector
			v21.normalize();
			n = v21;
		}
		else
		{
			v21.normalize();
			v32.normalize();

			double dot_product = dot( v32, v21 );
			double dot_product_abs = abs( dot_product );

			if( dot_product_abs > (1.0+GEOM_TOLERANCE) || dot_product_abs < (1.0-GEOM_TOLERANCE) )
			{
				n = (v32 + v21)*dot_product - v32 - v21;
				n.normalize();
			}
			else
			{
				// dot == 1 or -1, points are colinear
				n = -v21;
			}
		}
	}
	return valid;
}

void convertPlane2Matrix( const carve::geom::vector<3>& plane_normal, const carve::geom::vector<3>& plane_position, 
						 const carve::geom::vector<3>& local_z, carve::math::Matrix& resulting_matrix )
{
	carve::geom::vector<3> local_x( plane_normal );
	local_x.normalize();
	carve::geom::vector<3> local_z_new( local_z );

	carve::geom::vector<3> local_y = cross( local_x, local_z_new );
	local_z_new = cross( local_y, local_x );
	local_z_new.normalize();
	local_y.normalize();

	resulting_matrix._11 = local_x.x;
	resulting_matrix._12 = local_x.y;
	resulting_matrix._13 = local_x.z,
	resulting_matrix._14 = 0;
	resulting_matrix._21 = local_y.x;
	resulting_matrix._22 = local_y.y;
	resulting_matrix._23 =	local_y.z;
	resulting_matrix._24 =	0;
	resulting_matrix._31 = local_z_new.x;
	resulting_matrix._32 = local_z_new.y;
	resulting_matrix._33 = local_z_new.z;
	resulting_matrix._34 = 0;
	resulting_matrix._41 = plane_position.x;
	resulting_matrix._42 = plane_position.y;
	resulting_matrix._43 = plane_position.z;
	resulting_matrix._44 = 1;
}
