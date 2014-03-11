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

#include <fstream>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Group>
#include <osg/MatrixTransform>
#include <osgUtil/Tessellator>
#include <osgUtil/Optimizer>
#include <osgText/Text>

#include "carve/carve.hpp"
#include "carve/csg.hpp"
#include "carve/csg_triangulator.hpp"
#include "carve/faceloop.hpp"
#include "carve/geom3d.hpp"
#include "carve/input.hpp"
#include "carve/polyhedron_base.hpp"
#include "carve/poly.hpp"
#include "carve/poly.hpp"
#include "carve/triangulator.hpp"
#include "carve/math.hpp"
#include "common/geometry.hpp"

#include "ifcpp/model/IfcPPException.h"
#include "ifcpp/model/UnitConverter.h"
#include "ifcpp/IFC4/include/IfcSphere.h"
#include "ifcpp/IFC4/include/IfcPositiveLengthMeasure.h"
#include "ifcpp/IFC4/include/IfcUShapeProfileDef.h"
#include "ifcpp/IFC4/include/IfcNonNegativeLengthMeasure.h"
#include "ifcpp/IFC4/include/IfcPlaneAngleMeasure.h"
#include "ifcpp/IFC4/include/IfcExtrudedAreaSolid.h"
#include "ifcpp/IFC4/include/IfcDirection.h"

#include "GeomUtils.h"
#include "ProfileConverter.h"
#include "RepresentationConverter.h"
#include "SolidModelConverter.h"
#include "ConverterOSG.h"

//#define DEBUG_DRAW_NORMALS

inline void drawTriangles( osg::Vec3Array* vertices_triangles, osg::Vec3Array* normals_triangles, bool add_color_array, osg::Geode* geode )
{
	osg::Geometry* geometry = new osg::Geometry();
	geometry->setVertexArray( vertices_triangles );

	geometry->setNormalArray( normals_triangles );
	geometry->setNormalBinding( osg::Geometry::BIND_PER_VERTEX );

	if( add_color_array )
	{
		osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array();
		colors->resize( vertices_triangles->size(), osg::Vec4f( 0.6f, 0.6f, 0.6f, 0.1f ) );

		geometry->setColorArray( colors );
		geometry->setColorBinding( osg::Geometry::BIND_PER_VERTEX );
	}

	geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::TRIANGLES,0,vertices_triangles->size()));
	geode->addDrawable( geometry );
}

inline void drawQuads( osg::Vec3Array* vertices, osg::Vec3Array* normals, bool add_color_array, osg::Geode* geode )
{
	osg::Geometry* geometry = new osg::Geometry();
	geometry->setVertexArray( vertices );

	geometry->setNormalArray( normals );
	geometry->setNormalBinding( osg::Geometry::BIND_PER_VERTEX );

	if( add_color_array )
	{
		osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array();
		colors->resize( vertices->size(), osg::Vec4f( 0.6f, 0.6f, 0.6f, 0.1f ) );

		geometry->setColorArray( colors );
		geometry->setColorBinding( osg::Geometry::BIND_PER_VERTEX );
	}

	geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::QUADS,0,vertices->size()));
	geode->addDrawable( geometry );
}

ConverterOSG::ConverterOSG()
{
}

ConverterOSG::~ConverterOSG()
{

}

void ConverterOSG::drawFace( const carve::mesh::Face<3>* face, osg::Geode* geode, bool add_color_array )
{
	std::vector<carve::geom::vector<3> > face_vertices;
	face_vertices.resize(face->nVertices());
	carve::mesh::Edge<3> *e = face->edge;
	const size_t num_vertices = face->nVertices();
	for( size_t i = 0; i < num_vertices; ++i )
	{
		face_vertices[i] = e->v1()->v;
		e = e->next;
	}

	if( num_vertices < 4 )
	{
		std::cout << "drawFace is meant only for num vertices > 4" << std::endl;
	}

	carve::geom::vector<3> * vertex_vec;
	osg::Vec3Array* vertices = new osg::Vec3Array();
	osg::ref_ptr<osg::DrawElementsUInt> triangles = new osg::DrawElementsUInt( osg::PrimitiveSet::POLYGON, 0 );

	for( size_t i = 0; i < num_vertices; ++i )
	{
		vertex_vec = &face_vertices[num_vertices-i-1];
		vertices->push_back(osg::Vec3f(vertex_vec->x, vertex_vec->y, vertex_vec->z));
		triangles->push_back( i );
	}

	osg::Vec3f poly_normal = GeomUtils::computePolygonNormal( vertices );
	osg::Vec3Array* normals = new osg::Vec3Array();
	normals->resize( num_vertices, poly_normal );


	osg::Geometry* geometry = new osg::Geometry();
	geometry->setVertexArray( vertices );
	geometry->setNormalArray( normals );

	geometry->setNormalBinding( osg::Geometry::BIND_PER_VERTEX );
	geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POLYGON,0,vertices->size()));

	if( add_color_array )
	{
		osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array();
		colors->resize( vertices->size(), osg::Vec4f( 0.6f, 0.6f, 0.6f, 0.1f ) );

		geometry->setColorArray( colors );
		geometry->setColorBinding( osg::Geometry::BIND_PER_VERTEX );
	}

	if( num_vertices > 4 )
	{
		// TODO: check if polygon is convex with Gift wrapping algorithm

		osg::ref_ptr<osgUtil::Tessellator> tesselator = new osgUtil::Tessellator();
		tesselator->setTessellationType(osgUtil::Tessellator::TESS_TYPE_POLYGONS);
		//tesselator->setWindingType( osgUtil::Tessellator::TESS_WINDING_ODD );
		tesselator->retessellatePolygons(*geometry);
	}
	geode->addDrawable(geometry);


#ifdef DEBUG_DRAW_NORMALS
	osg::Vec3Array* vertices_normals = new osg::Vec3Array();
	for( size_t i = 0; i < num_vertices; ++i )
	{
		vertex_vec = &face_vertices[num_vertices-i-1];
		vertices_normals->push_back(osg::Vec3f(vertex_vec->x, vertex_vec->y, vertex_vec->z));
		vertices_normals->push_back(osg::Vec3f(vertex_vec->x, vertex_vec->y, vertex_vec->z) + poly_normal );
	}

	osg::Vec4Array* colors_normals = new osg::Vec4Array();
	colors_normals->resize( num_vertices*2, osg::Vec4f( 0.4f, 0.7f, 0.4f, 1.f ) );

	osg::Geometry* geometry_normals = new osg::Geometry();
	geometry_normals->setVertexArray( vertices_normals );
	geometry_normals->setColorArray( colors_normals );
	geometry_normals->setColorBinding( osg::Geometry::BIND_PER_VERTEX );
	geometry_normals->getOrCreateStateSet()->setMode( GL_LIGHTING, osg::StateAttribute::OFF );

	geometry_normals->setNormalBinding( osg::Geometry::BIND_OFF );
	geometry_normals->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES,0,vertices_normals->size()));
	geode->addDrawable(geometry_normals);
#endif
}

void ConverterOSG::drawMesh( const carve::mesh::Mesh<3>* mesh, osg::Geode* geode, bool add_color_array )
{
	osg::ref_ptr<osg::Vec3Array> vertices_tri = new osg::Vec3Array();
	osg::ref_ptr<osg::Vec3Array> normals_tri = new osg::Vec3Array();

	osg::ref_ptr<osg::Vec3Array> vertices_quad = new osg::Vec3Array();
	osg::ref_ptr<osg::Vec3Array> normals_quad = new osg::Vec3Array();

	std::vector<carve::mesh::Vertex<3> *> v;
	const carve::geom::vector<3>* v0;
	const carve::geom::vector<3>* v1;
	const carve::geom::vector<3>* v2;
	const carve::geom::vector<3>* v3;

	const size_t num_faces = mesh->faces.size();
	for( size_t i = 0; i != num_faces; ++i )
	{
		carve::mesh::Face<3>* f = mesh->faces[i];
		if(f->nVertices() > 4)
		{
			drawFace(f, geode );
			continue;
		}
		osg::Vec3 normal(f->plane.N.v[0],f->plane.N.v[1],f->plane.N.v[2]);
		f->getVertices(v);

		if( f->nVertices() == 3 )
		{
			v0 = &(v[0]->v);
			v1 = &(v[1]->v);
			v2 = &(v[2]->v);
			normals_tri->push_back( normal );
			normals_tri->push_back( normal );
			normals_tri->push_back( normal );

			vertices_tri->push_back( osg::Vec3(v0->x, v0->y, v0->z ) );
			vertices_tri->push_back( osg::Vec3(v1->x, v1->y, v1->z ) );
			vertices_tri->push_back( osg::Vec3(v2->x, v2->y, v2->z ) );
		}
		else if( f->nVertices() == 4 )
		{
			v0 = &(v[0]->v);
			v1 = &(v[1]->v);
			v2 = &(v[2]->v);
			v3 = &(v[3]->v);

			normals_quad->push_back( normal );
			normals_quad->push_back( normal );
			normals_quad->push_back( normal );
			normals_quad->push_back( normal );

			vertices_quad->push_back( osg::Vec3(v0->x, v0->y, v0->z ) );
			vertices_quad->push_back( osg::Vec3(v1->x, v1->y, v1->z ) );
			vertices_quad->push_back( osg::Vec3(v2->x, v2->y, v2->z ) );
			vertices_quad->push_back( osg::Vec3(v3->x, v3->y, v3->z ) );
		}
	}

	if( vertices_tri->size() > 0 )
	{
		drawTriangles( vertices_tri, normals_tri, add_color_array, geode );
	}

	if( vertices_quad->size() > 0 )
	{
		drawQuads( vertices_quad, normals_quad, add_color_array, geode );
	}
}


void dumpMeshset( shared_ptr<carve::mesh::MeshSet<3> >& meshset, const int poly_num, std::stringstream& strs_out )
{
	strs_out << "shared_ptr<carve::input::PolyhedronData> poly_data" << poly_num << "( new carve::input::PolyhedronData() );" << std::endl;
	carve::poly::Polyhedron * poly = carve::polyhedronFromMesh(meshset.get(), -1);

	
	const size_t num_vertices = poly->vertices.size();
	for( int i=0; i<num_vertices; ++i )
	{
		carve::poly::Vertex<3> vertex = poly->vertices[i];
		strs_out << "poly_data" << poly_num << "->addVertex( carve::geom::VECTOR(" << vertex.v.x << ", " << vertex.v.y << ", " << vertex.v.z << " ) );" << std::endl;
	}
		
	for( int i=0; i<poly->faces.size(); ++i )
	{
		carve::poly::Face<3> f = poly->faces[i];
		strs_out << "poly_data" << poly_num << "->addFace( ";
		for( int j=0; j< f.nVertices(); ++j )
		{
			if( j > 0 )
			{
				strs_out << ", ";
			}
			strs_out << poly->vertexToIndex(f.vertex(j));
		}
		strs_out << ");" << std::endl;
	}
}

#ifdef _DEBUG
int dump_count = 1;
void ConverterOSG::dumpMeshsets( shared_ptr<carve::mesh::MeshSet<3> >& first_operand_meshset, 
								shared_ptr<carve::mesh::MeshSet<3> >& second_operand_meshset, 
								shared_ptr<carve::mesh::MeshSet<3> >& result_meshset )
{
	std::stringstream cpp_input;

	if( first_operand_meshset )		dumpMeshset( first_operand_meshset, 1, cpp_input );
	if( second_operand_meshset )	dumpMeshset( second_operand_meshset, 2, cpp_input );
	if( result_meshset )			dumpMeshset( result_meshset, 3, cpp_input );

	std::stringstream file_name;
	file_name << "dump_csg_failure" << dump_count << ".txt";
	std::ofstream ofs( file_name.str().c_str(), std::ofstream::out);
	ofs << cpp_input.str().c_str();
	ofs.close();

	++dump_count;
}
#endif

void ConverterOSG::drawMeshSet( const shared_ptr<carve::mesh::MeshSet<3> >& meshset, osg::Geode* geode, bool add_color_array )
{
	if( !meshset )
	{
		return;
	}

	for( size_t i = 0; i < meshset->meshes.size(); ++i )
	{
		drawMesh( meshset->meshes[i], geode, add_color_array );
	}
}

void ConverterOSG::drawPolyhedron( const shared_ptr<carve::poly::Polyhedron>& polyhedron, osg::Geode* geode, bool add_color_array )
{
	shared_ptr<carve::mesh::MeshSet<3> > mesh_set( carve::meshFromPolyhedron(polyhedron.get(), -1) );
	for( size_t i = 0; i < mesh_set->meshes.size(); ++i )
	{
		drawMesh(mesh_set->meshes[i], geode, add_color_array );
	}
}

void ConverterOSG::drawVertexNumbers( const shared_ptr<carve::input::PolyhedronData> poly, const osg::Vec4f& color, osg::Geode* geode )
{
	if( poly )
	{
		const size_t num_vertices = poly->points.size();
		for( int i=0; i<num_vertices; ++i )
		{
			carve::poly::Vertex<3> vertex = poly->points[i];
			osgText::Text* txt = new osgText::Text;
			txt->setFont("fonts/arial.ttf");
			txt->setColor( color );
			txt->setCharacterSize( 0.1f );
			txt->setAutoRotateToScreen( true );
			txt->setPosition( osg::Vec3( vertex.v.x , vertex.v.y , vertex.v.z ) );

			std::stringstream strs;
			strs << i;
			txt->setText( strs.str().c_str() );
			geode->addDrawable(txt);
		}
	}
}

void ConverterOSG::drawOpenMesh(	const shared_ptr<carve::input::PolyhedronData>& poly_data, osg::Geode* geode, bool add_color_array )
{
	const std::vector<carve::geom::vector<3> >& vec_points = poly_data->points;
	const std::vector<int>& face_indices = poly_data->faceIndices;

	osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array();
	osg::ref_ptr<osg::Vec3Array> normals = new osg::Vec3Array();
	osg::ref_ptr<osg::DrawElementsUInt> triangles = new osg::DrawElementsUInt( osg::PrimitiveSet::TRIANGLES, 0 );

	osg::ref_ptr<osg::Vec3Array> normals_quad = new osg::Vec3Array();
	osg::ref_ptr<osg::DrawElementsUInt> quads = new osg::DrawElementsUInt( osg::PrimitiveSet::QUADS, 0 );

	std::vector<carve::geom::vector<3> >::const_iterator it_points;
	for( it_points = vec_points.begin(); it_points != vec_points.end(); ++it_points )
	{
		const carve::geom::vector<3> & vec = (*it_points);
		vertices->push_back( osg::Vec3f( vec.x, vec.y, vec.z ) );
	}

	std::vector<int>::const_iterator it_face_indices;
	for( it_face_indices = face_indices.begin(); it_face_indices != face_indices.end(); ++it_face_indices )
	{
		int num_indices = (*it_face_indices);
		if( num_indices == 3 )
		{
			++it_face_indices;
			int vi0 = (*it_face_indices);
			triangles->push_back( vi0 );
			osg::Vec3f& v0 = vertices->at( vi0 );

			++it_face_indices;
			int vi1 = (*it_face_indices);
			triangles->push_back( vi1 );
			osg::Vec3f& v1 = vertices->at( vi1 );

			++it_face_indices;
			int vi2 = (*it_face_indices);
			triangles->push_back( vi2 );
			osg::Vec3f& v2 = vertices->at( vi2 );

			// TODO: get edged from connectivity and compute intermediate normal if angle between faces is small
			osg::Vec3f normal( (v1-v0)^(v2-v0) );
			normal.normalize();
			normals->push_back(normal);

		}
		else if( num_indices == 4 )
		{
			++it_face_indices;
			int vi0 = (*it_face_indices);
			quads->push_back( vi0 );
			osg::Vec3f& v0 = vertices->at( vi0 );

			++it_face_indices;
			int vi1 = (*it_face_indices);
			quads->push_back( vi1 );
			osg::Vec3f& v1 = vertices->at( vi1 );

			++it_face_indices;
			int vi2 = (*it_face_indices);
			quads->push_back( vi2 );
			osg::Vec3f& v2 = vertices->at( vi2 );

			++it_face_indices;
			int vi3 = (*it_face_indices);
			quads->push_back( vi3 );

			osg::Vec3f normal( (v1-v0)^(v2-v0) );
			normal.normalize();
			normals->push_back(normal);

		}
		else
		{
			for( int i=0; i<num_indices; ++i )
			{
				(++it_face_indices);

			}
		}
	}

	osg::Geometry* geometry = new osg::Geometry();
	geometry->setVertexArray( vertices );

	geometry->setNormalArray( normals );
	geometry->setNormalBinding( osg::Geometry::BIND_PER_VERTEX );

	if( add_color_array )
	{
		osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array();
		colors->resize( vertices->size(), osg::Vec4f( 0.6f, 0.6f, 0.6f, 0.1f ) );

		geometry->setColorArray( colors );
		geometry->setColorBinding( osg::Geometry::BIND_PER_VERTEX );
	}

	if( triangles->size() > 0 )
	{
		geometry->addPrimitiveSet( triangles );
	}
	if( quads->size() > 0 )
	{
		geometry->addPrimitiveSet( quads );
	}
	geode->addDrawable( geometry );
}

void ConverterOSG::drawPolyline( const shared_ptr<carve::input::PolylineSetData>& polyline_data, osg::Geode* geode, bool add_color_array )
{
	osg::Vec3Array* vertices = new osg::Vec3Array();
	carve::line::PolylineSet* polyline_set = polyline_data->create(carve::input::opts());

	if( polyline_set->vertices.size() < 2 )
	{
		std::cout << "polyline_set->vertices.size() < 2" << std::endl;
		return;
	}

	carve::line::PolylineSet::const_line_iter it;
	for( it = polyline_set->lines.begin(); it != polyline_set->lines.end(); ++it )
	{
		carve::line::Polyline* pline = *it;
		size_t vertex_count = pline->vertexCount();

		for( size_t vertex_i = 0; vertex_i < vertex_count; ++vertex_i )
		{
			if( vertex_i >= polyline_set->vertices.size() )
			{
				std::cout << "drawPolyline: vertex_i >= polyline_set->vertices.size()" << std::endl;
				continue;
			}
			const carve::line::Vertex* v = pline->vertex( vertex_i );
			vertices->push_back( osg::Vec3d( v->v[0], v->v[1], v->v[2] ) );

		}
	}

	osg::Geometry* geometry = new osg::Geometry();
	geometry->setVertexArray( vertices );
	geometry->addPrimitiveSet( new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP, 0, vertices->size()) );

	if( add_color_array )
	{
		osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array();
		colors->resize( vertices->size(), osg::Vec4f( 0.6f, 0.6f, 0.6f, 0.1f ) );

		geometry->setColorArray( colors );
		geometry->setColorBinding( osg::Geometry::BIND_PER_VERTEX );
	}

	geode->addDrawable(geometry);
}

//#define THOROUGH_MESHSET_CHECK

bool ConverterOSG::checkMeshSet( const shared_ptr<carve::mesh::MeshSet<3> >& mesh_set, std::stringstream& err_poly, int entity_id )
{
	// check opening polyhedron
	if( !mesh_set )
	{
#ifdef _DEBUG
		err_poly << "MeshSet of entity #" << entity_id << " not valid" << std::endl;
#endif
		return false;
	}
	if( mesh_set->meshes.size() == 0 )
	{
#ifdef _DEBUG
		err_poly << "MeshSet of entity #" << entity_id << " has no meshes" << std::endl;
#endif
		return false;
	}

	std::stringstream err;
	bool meshes_closed = true;
	for (size_t i = 0; i < mesh_set->meshes.size(); ++i)
	{
		carve::mesh::Mesh<3>* mesh_i = mesh_set->meshes[i];

		if( mesh_i->isNegative() )
		{
			mesh_i->invert();
			if( mesh_i->isNegative() )
			{
				mesh_i->recalc();
				mesh_i->calcOrientation();
				if( mesh_i->isNegative() )
				{
					err << "mesh_set->meshes[" << i << "]->isNegative() " << std::endl;
				}
			}
		}

		if( !mesh_i->isClosed() )
		{
			meshes_closed = false;
		}

#ifdef THOROUGH_MESHSET_CHECK

		std::vector<carve::mesh::Face<3>* >& vec_faces = mesh_i->faces;
		for( int j=0; j<vec_faces.size(); ++j )
		{
			carve::mesh::Face<3>* face = vec_faces[j];


			carve::geom::vector<3>& face_normal = face->plane.N;
			//carve::geom::vector<3>& face_centroid = face->centroid;
			carve::geom::vector<3>& point_on_face = face->edge->v1()->v;
			carve::geom::linesegment<3> face_linesegment( point_on_face, point_on_face + face_normal*10000 );
			int intersect_face_count = 0;
			int intersect_vertex_count = 0;
			int intersect_edge_count = 0;

			for( int j_other=0; j_other<vec_faces.size(); ++j_other )
			{
				if( j == j_other )
				{
					// don't intersect face with itself
					continue;
				}
				carve::mesh::Face<3>* other_face = vec_faces[j_other];

				carve::geom::vector<3> intersection_point;
				carve::IntersectionClass intersection_result = other_face->lineSegmentIntersection( face_linesegment, intersection_point );

				double intersection_distance = DBL_MAX;

				if( intersection_result > 0 )
				{
					if( intersection_result == carve::INTERSECT_FACE )
					{
						++intersect_face_count;
					}
					else if( intersection_result == carve::INTERSECT_VERTEX )
					{
						++intersect_vertex_count;
					}
					else if( intersection_result == carve::INTERSECT_EDGE )
					{
						++intersect_edge_count;
					}
				}
			}
			if( intersect_face_count > 0 )
			{
				if( intersect_face_count%2 == 1 )
				{
					if( intersect_vertex_count == 0 )
					{
						err_poly << "face normal pointing inside" << std::endl;
						return false;
					}
				}
			}
		}
#endif
	}


	if( !meshes_closed )
	{
#ifdef _DEBUG
		err << "mesh_set not closed" << std::endl;
#endif
	}


	if( err.tellp() > 0 )
	{
		err_poly << "MeshSet of entity #" << entity_id << " has problems:" << std::endl;
		err_poly << err.str().c_str();
		return false;
	}
	return true;
}

double ConverterOSG::computeSurfaceAreaOfGroup( const osg::Group* grp )
{
	double surface_area = 0.0;
	int num_children = grp->getNumChildren();
	for( int i=0; i<num_children; ++i )
	{
		const osg::Node* node = grp->getChild(i);
		const osg::Group* child_group = dynamic_cast<const osg::Group*>(node);
		if( child_group )
		{
			surface_area += computeSurfaceAreaOfGroup( child_group );
			continue;
		}
		const osg::Geode* child_geode = dynamic_cast<const osg::Geode*>(node);
		if( child_geode )
		{
			const osg::Geode::DrawableList& drawable_list = child_geode->getDrawableList();
			osg::Geode::DrawableList::const_iterator it_drawables;
			for( it_drawables=drawable_list.begin(); it_drawables!=drawable_list.end(); ++it_drawables )
			{
				osg::Drawable* drawable = (*it_drawables);
				const osg::Geometry* child_gemetry = dynamic_cast<const osg::Geometry*>(drawable);
				if( !child_gemetry )
				{
					std::cout << "!child_gemetry" << std::endl;
					return 0;
				}
				const osg::Array* vertices_array = child_gemetry->getVertexArray();
				const osg::Vec3Array* vertices_float = dynamic_cast<const osg::Vec3Array*>(vertices_array);

				if( !vertices_float )
				{
					std::cout << "!vertices_float" << std::endl; 
					return 0;
				}

				const osg::Geometry::PrimitiveSetList& primitive_sets = child_gemetry->getPrimitiveSetList();
				osg::Geometry::PrimitiveSetList::const_iterator it_primitives;
				for( it_primitives=primitive_sets.begin(); it_primitives!=primitive_sets.end(); ++it_primitives )
				{
					const osg::PrimitiveSet* p_set = (*it_primitives);

					const int num_elements = p_set->getNumIndices();
					if( num_elements < 3 )
					{
						std::cout << "num_elements < 3" << std::endl; 
						continue;
					}

					if( p_set->getMode() == osg::PrimitiveSet::QUADS )
					{
						for( int k=0; k<num_elements-3; k+=4 )
						{
							const osg::Vec3& v0 = vertices_float->at( p_set->index(k) );
							const osg::Vec3& v1 = vertices_float->at( p_set->index(k+1) );
							const osg::Vec3& v2 = vertices_float->at( p_set->index(k+2) );
							const osg::Vec3& v3 = vertices_float->at( p_set->index(k+3) );

#ifdef _DEBUG
							if( (v0 -v1).length2() < 0.00001 )
							{
								continue;
							}
							if( (v1 -v2).length2() < 0.00001 )
							{
								continue;
							}
							if( (v2 -v3).length2() < 0.00001 )
							{
								continue;
							}
							if( (v3 -v0).length2() < 0.00001 )
							{
								continue;
							}
#endif
							osg::Vec3d v0_v1 = v1 - v0;
							osg::Vec3d v0_v3 = v3 - v0;
							osg::Vec3d v2_v1 = v1 - v2;
							osg::Vec3d v2_v3 = v3 - v2;
							osg::Vec3d cross_vec = v0_v1 ^ v0_v3;
							surface_area += cross_vec.length()*0.5;

							cross_vec = v2_v1 ^ v2_v3;
							surface_area += cross_vec.length()*0.5;
						}
					}
					else if( p_set->getMode() == osg::PrimitiveSet::TRIANGLES )
					{
						for( int k=0; k<num_elements-2; k+=3 )
						{
							const osg::Vec3& v0 = vertices_float->at( p_set->index(k) );
							const osg::Vec3& v1 = vertices_float->at( p_set->index(k+1) );
							const osg::Vec3& v2 = vertices_float->at( p_set->index(k+2) );

							osg::Vec3d v0_v1 = v1 - v0;
							osg::Vec3d v0_v2 = v2 - v0;

							osg::Vec3d cross_vec = v0_v1 ^ v0_v2;
							surface_area += cross_vec.length()*0.5;
						}
					}
					else if( p_set->getMode() == osg::PrimitiveSet::TRIANGLE_STRIP )
					{
						for( int k=0; k<num_elements-3; ++k )
						{
							const osg::Vec3& v0 = vertices_float->at( p_set->index(k) );
							const osg::Vec3& v1 = vertices_float->at( p_set->index(k+1) );
							const osg::Vec3& v2 = vertices_float->at( p_set->index(k+2) );

							osg::Vec3d v0_v1 = v1 - v0;
							osg::Vec3d v0_v2 = v2 - v0;

							osg::Vec3d cross_vec = v0_v1 ^ v0_v2;
							surface_area += cross_vec.length()*0.5;
						}
					}
					else if( p_set->getMode() == osg::PrimitiveSet::TRIANGLE_FAN )
					{
						const osg::Vec3& v0 = vertices_float->at( p_set->index(0) );
						for( int k=0; k<num_elements-2; ++k )
						{
							const osg::Vec3& v1 = vertices_float->at( p_set->index(k+1) );
							const osg::Vec3& v2 = vertices_float->at( p_set->index(k+2) );

							osg::Vec3d v0_v1 = v1 - v0;
							osg::Vec3d v0_v2 = v2 - v0;

							osg::Vec3d cross_vec = v0_v1 ^ v0_v2;
							surface_area += cross_vec.length()*0.5;
						}
					}
					else
					{
						std::cout << "other primitive set mode" << std::endl;
						return 0;
					}
				}
			}
		}
	}
	return surface_area;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define POINTS 10
void ConverterOSG::createTest( osg::Group* group, osg::Group* root )
{
	{
shared_ptr<carve::input::PolyhedronData> poly_data1( new carve::input::PolyhedronData() );
poly_data1->addVertex( carve::geom::VECTOR(0.365, 7.275, 2.94 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.365, 11.74, 2.94 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.365, 9.1476, 6.0684 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.365, 7.275, 7.06408 ) );
poly_data1->addVertex( carve::geom::VECTOR(0, 11.74, 2.94 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.365, 11.74, 4.69 ) );
poly_data1->addVertex( carve::geom::VECTOR(-5.55112e-017, 7.43613, 6.97841 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.365, 7.27499, 7.06409 ) );
poly_data1->addVertex( carve::geom::VECTOR(0, 7.275, 2.94 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.118546, 11.74, 4.69 ) );
poly_data1->addVertex( carve::geom::VECTOR(0, 9.64725, 5.80274 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.365, 7.125, 8.24843 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.365, 7.125, 2.94 ) );
poly_data1->addVertex( carve::geom::VECTOR(0, 11.74, 4.69 ) );
poly_data1->addVertex( carve::geom::VECTOR(1.13257e-005, 7.27499, 7.06409 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.365, 7.27499, 8.32818 ) );
poly_data1->addVertex( carve::geom::VECTOR(0, 4.365, 2.94 ) );
poly_data1->addVertex( carve::geom::VECTOR(-1.11022e-016, 7.275, 7.06408 ) );
poly_data1->addVertex( carve::geom::VECTOR(0, 7.27499, 7.06409 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.365, 7.27499, 8.32818 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.365, 6.125, 5.2 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.365, 4.365, 6.78091 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.365, 5.50101, 5.2 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.365, 6.125, 4.33163 ) );
poly_data1->addVertex( carve::geom::VECTOR(0, 6.125, 3.69 ) );
poly_data1->addVertex( carve::geom::VECTOR(0, 6.125, 4.45789 ) );
poly_data1->addVertex( carve::geom::VECTOR(0, 4.365, 6.78091 ) );
poly_data1->addVertex( carve::geom::VECTOR(0, 5.56275, 5.2 ) );
poly_data1->addVertex( carve::geom::VECTOR(0, 5.24, 5.2 ) );
poly_data1->addVertex( carve::geom::VECTOR(0, 5.24, 3.69 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.365, 4.365, 2.94 ) );
poly_data1->addVertex( carve::geom::VECTOR(0, 7.27499, 7.06409 ) );
poly_data1->addVertex( carve::geom::VECTOR(1.69407e-021, 7.27499, 7.06409 ) );
poly_data1->addVertex( carve::geom::VECTOR(0, 7.27499, 8.32818 ) );
poly_data1->addVertex( carve::geom::VECTOR(-5.55112e-017, 3.79863, 6.47977 ) );
poly_data1->addVertex( carve::geom::VECTOR(0, 6.125, 5.2 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.365, 4.215, 2.94 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.365, 4.215, 6.70116 ) );
poly_data1->addVertex( carve::geom::VECTOR(-2.77556e-017, 2.77556e-017, 4.46 ) );
poly_data1->addVertex( carve::geom::VECTOR(0, 0, 2.94 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.365, -4.44089e-016, 4.46 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.365, 0, 2.94 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.365, 6.125, 3.69 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.365, 5.24, 3.69 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.365, 5.24, 5.2 ) );
poly_data1->addFace( 0, 1, 2);
poly_data1->addFace( 3, 0, 2);
poly_data1->addFace( 0, 4, 1);
poly_data1->addFace( 2, 1, 5);
poly_data1->addFace( 2, 6, 3);
poly_data1->addFace( 0, 3, 7);
poly_data1->addFace( 0, 8, 4);
poly_data1->addFace( 1, 4, 9);
poly_data1->addFace( 2, 5, 10);
poly_data1->addFace( 5, 1, 9);
poly_data1->addFace( 10, 6, 2);
poly_data1->addFace( 3, 6, 7);
poly_data1->addFace( 7, 11, 0);
poly_data1->addFace( 12, 8, 0);
poly_data1->addFace( 4, 8, 10);
poly_data1->addFace( 9, 4, 13);
poly_data1->addFace( 10, 5, 9);
poly_data1->addFace( 6, 10, 8);
poly_data1->addFace( 14, 7, 6);
poly_data1->addFace( 7, 15, 11);
poly_data1->addFace( 11, 12, 0);
poly_data1->addFace( 8, 12, 16);
poly_data1->addFace( 13, 4, 10);
poly_data1->addFace( 9, 13, 10);
poly_data1->addFace( 8, 17, 6);
poly_data1->addFace( 6, 17, 14);
poly_data1->addFace( 18, 7, 14);
poly_data1->addFace( 15, 7, 18);
poly_data1->addFace( 19, 11, 15);
poly_data1->addFace( 20, 21, 22);
poly_data1->addFace( 11, 21, 20);
poly_data1->addFace( 12, 11, 20);
poly_data1->addFace( 12, 20, 23);
poly_data1->addFace( 24, 25, 8);
poly_data1->addFace( 24, 8, 16);
poly_data1->addFace( 26, 27, 28);
poly_data1->addFace( 16, 26, 28);
poly_data1->addFace( 29, 24, 16);
poly_data1->addFace( 16, 28, 29);
poly_data1->addFace( 12, 30, 16);
poly_data1->addFace( 17, 8, 31);
poly_data1->addFace( 14, 17, 31);
poly_data1->addFace( 14, 32, 18);
poly_data1->addFace( 15, 18, 33);
poly_data1->addFace( 33, 19, 15);
poly_data1->addFace( 11, 19, 34);
poly_data1->addFace( 30, 12, 23);
poly_data1->addFace( 22, 21, 30);
poly_data1->addFace( 30, 23, 22);
poly_data1->addFace( 34, 21, 11);
poly_data1->addFace( 35, 26, 31);
poly_data1->addFace( 35, 8, 25);
poly_data1->addFace( 35, 27, 26);
poly_data1->addFace( 31, 8, 35);
poly_data1->addFace( 34, 26, 16);
poly_data1->addFace( 16, 30, 36);
poly_data1->addFace( 14, 31, 32);
poly_data1->addFace( 26, 18, 32);
poly_data1->addFace( 26, 33, 18);
poly_data1->addFace( 19, 33, 26);
poly_data1->addFace( 26, 34, 19);
poly_data1->addFace( 37, 30, 21);
poly_data1->addFace( 22, 23, 20);
poly_data1->addFace( 34, 37, 21);
poly_data1->addFace( 32, 31, 26);
poly_data1->addFace( 16, 38, 34);
poly_data1->addFace( 16, 36, 39);
poly_data1->addFace( 37, 36, 30);
poly_data1->addFace( 34, 40, 37);
poly_data1->addFace( 38, 16, 39);
poly_data1->addFace( 34, 38, 40);
poly_data1->addFace( 39, 36, 41);
poly_data1->addFace( 36, 37, 40);
poly_data1->addFace( 38, 39, 41);
poly_data1->addFace( 41, 40, 38);
poly_data1->addFace( 40, 41, 36);
poly_data1->addFace( 24, 42, 23);
poly_data1->addFace( 25, 24, 23);
poly_data1->addFace( 20, 35, 25);
poly_data1->addFace( 23, 20, 25);
poly_data1->addFace( 24, 29, 43);
poly_data1->addFace( 43, 42, 24);
poly_data1->addFace( 22, 44, 28);
poly_data1->addFace( 22, 28, 27);
poly_data1->addFace( 27, 35, 20);
poly_data1->addFace( 20, 22, 27);
poly_data1->addFace( 23, 42, 43);
poly_data1->addFace( 43, 20, 23);
poly_data1->addFace( 29, 28, 44);
poly_data1->addFace( 44, 43, 29);
poly_data1->addFace( 22, 20, 43);
poly_data1->addFace( 43, 44, 22);
shared_ptr<carve::input::PolyhedronData> poly_data2( new carve::input::PolyhedronData() );
poly_data2->addVertex( carve::geom::VECTOR(0, 9.875, 3.69 ) );
poly_data2->addVertex( carve::geom::VECTOR(0, 9.875, 5.2 ) );
poly_data2->addVertex( carve::geom::VECTOR(0, 8.99, 5.2 ) );
poly_data2->addVertex( carve::geom::VECTOR(0, 8.99, 3.69 ) );
poly_data2->addVertex( carve::geom::VECTOR(0.365, 9.875, 3.69 ) );
poly_data2->addVertex( carve::geom::VECTOR(0.365, 9.875, 5.2 ) );
poly_data2->addVertex( carve::geom::VECTOR(0.365, 8.99, 5.2 ) );
poly_data2->addVertex( carve::geom::VECTOR(0.365, 8.99, 3.69 ) );
poly_data2->addFace( 0, 1, 5, 4);
poly_data2->addFace( 1, 2, 6, 5);
poly_data2->addFace( 2, 3, 7, 6);
poly_data2->addFace( 3, 0, 4, 7);
poly_data2->addFace( 3, 1, 0);
poly_data2->addFace( 7, 4, 5);
poly_data2->addFace( 1, 3, 2);
poly_data2->addFace( 5, 6, 7);
		shared_ptr<carve::mesh::MeshSet<3> > meshset1( poly_data1->createMesh(carve::input::opts()) );
		shared_ptr<carve::mesh::MeshSet<3> > meshset2( poly_data2->createMesh(carve::input::opts()) );

		carve::csg::CSG csg;
		csg.hooks.registerHook(new carve::csg::CarveTriangulatorWithImprovement(), carve::csg::CSG::Hooks::PROCESS_OUTPUT_FACE_BIT);
		shared_ptr<carve::mesh::MeshSet<3> > result_meshset;
		try
		{
			result_meshset = shared_ptr<carve::mesh::MeshSet<3> >( csg.compute( meshset1.get(), meshset2.get(), carve::csg::CSG::A_MINUS_B, NULL, carve::csg::CSG::CLASSIFY_NORMAL ) );
		}
		catch(...)
		{
			std::cout << "csg op failed" << std::endl;

			osg::Geode* geode1 = new osg::Geode();
			group->addChild( geode1 );
			drawMeshSet( meshset1, geode1 );
			

			osg::Geode* geode2 = new osg::Geode();
			group->addChild( geode2 );
			drawMeshSet( meshset2, geode2 );

			osg::Geode* geode_txt = new osg::Geode();
			root->addChild( geode_txt );
			drawVertexNumbers( poly_data2, osg::Vec4( 1, 0, 0, 1 ), geode_txt );
			drawVertexNumbers( poly_data1, osg::Vec4( 0, 1, 0, 1 ), geode_txt );
		}
		
		osg::Geode* geode = new osg::Geode();
		group->addChild( geode );
		drawMeshSet( result_meshset, geode );
		return;

		
	}

	if( false )
	{
		//create a tetrahedron
		std::vector<carve::mesh::MeshSet<3>::vertex_t> tet_verts;
		std::vector<carve::mesh::MeshSet<3>::face_t *> tet_faces;
		std::vector<carve::mesh::MeshSet<3>::vertex_t *> corners;

		tet_verts.push_back(carve::mesh::MeshSet<3>::vertex_t(carve::geom::VECTOR(0.0, 0.0, 0.0)));
		tet_verts.push_back(carve::mesh::MeshSet<3>::vertex_t(carve::geom::VECTOR(1.0, 0.0, 0.0)));
		tet_verts.push_back(carve::mesh::MeshSet<3>::vertex_t(carve::geom::VECTOR(0.0, 1.0, 0.0)));
		tet_verts.push_back(carve::mesh::MeshSet<3>::vertex_t(carve::geom::VECTOR(0.0, 0.0, 1.0)));

		corners.push_back(&tet_verts[0]);
		corners.push_back(&tet_verts[2]);
		corners.push_back(&tet_verts[1]);
		tet_faces.push_back(new carve::mesh::MeshSet<3>::face_t(corners.begin(), corners.end()));

		corners.clear();
		corners.push_back(&tet_verts[0]);
		corners.push_back(&tet_verts[1]);
		corners.push_back(&tet_verts[3]);
		tet_faces.push_back(new carve::mesh::MeshSet<3>::face_t(corners.begin(), corners.end()));

		corners.clear();
		corners.push_back(&tet_verts[0]);
		corners.push_back(&tet_verts[3]);
		corners.push_back(&tet_verts[2]);
		tet_faces.push_back(new carve::mesh::MeshSet<3>::face_t(corners.begin(), corners.end()));

		corners.clear();
		corners.push_back(&tet_verts[1]);
		corners.push_back(&tet_verts[2]);
		corners.push_back(&tet_verts[3]);
		tet_faces.push_back(new carve::mesh::MeshSet<3>::face_t(corners.begin(), corners.end()));

		shared_ptr<carve::mesh::MeshSet<3> > tetrahedron( new carve::mesh::MeshSet<3>(tet_faces) );

		osg::Geode* geode = new osg::Geode();
		drawMeshSet( tetrahedron, geode );
		group->addChild(geode);


		//create a triangle
		std::vector<carve::mesh::MeshSet<3>::vertex_t> tri_verts;
		std::vector<carve::mesh::MeshSet<3>::face_t *> tri_faces;

		//Vertices
		//crashes if last coordinate set to 1e-8, but ok for 1e-7
		tri_verts.push_back(carve::mesh::MeshSet<3>::vertex_t(carve::geom::VECTOR(-0.3, 0.0, 1e-8)));
		tri_verts.push_back(carve::mesh::MeshSet<3>::vertex_t(carve::geom::VECTOR(1.0, 0.0, 1.1e-8)));
		tri_verts.push_back(carve::mesh::MeshSet<3>::vertex_t(carve::geom::VECTOR(-0.3, 1.0, 1.1e-8)));

		//Face
		corners.clear();
		corners.push_back(&tri_verts[0]);
		corners.push_back(&tri_verts[2]);
		corners.push_back(&tri_verts[1]);
		tri_faces.push_back(new carve::mesh::MeshSet<3>::face_t(corners.begin(), corners.end()));

		//  corners.clear();
		//  corners.push_back(&tri_verts[0]);
		//  corners.push_back(&tri_verts[1]);
		//  corners.push_back(&tri_verts[2]);
		//  tri_faces.push_back(new carve::mesh::MeshSet<3>::face_t(corners));

		shared_ptr<carve::mesh::MeshSet<3> > triangle( new carve::mesh::MeshSet<3>(tri_faces) );

		drawMeshSet( triangle, geode );



		//cut triangle with tetrahedron.
		shared_ptr<carve::mesh::MeshSet<3> > is_poly( carve::csg::CSG().compute( tetrahedron.get(), triangle.get(), carve::csg::CSG::INTERSECTION) );

		drawMeshSet( is_poly, geode );


	}


	shared_ptr<IfcSphere> sphere( new IfcSphere() );
	sphere->m_Radius = shared_ptr<IfcPositiveLengthMeasure>( new IfcPositiveLengthMeasure() );
	sphere->m_Radius->m_value = 0.75;

	shared_ptr<GeometrySettings> geom_settings( new GeometrySettings() );
	geom_settings->m_num_vertices_per_circle = 50;
	shared_ptr<UnitConverter> unit_converter( new UnitConverter() );
	shared_ptr<RepresentationConverter> representation_converter( new RepresentationConverter( geom_settings, unit_converter ) );

	shared_ptr<ItemData> item_data( new ItemData() );
	representation_converter->getSolidConverter()->convertIfcCsgPrimitive3D( sphere, carve::math::Matrix::TRANS(1.0, 1.0, 1.0), item_data );
	item_data->createMeshSetsFromClosedPolyhedrons();


	if( item_data->item_meshsets.size() != 1 )
	{
		return;
	}

	shared_ptr<carve::mesh::MeshSet<3> > sphere_meshset = item_data->item_meshsets[0];
	osg::Geode* geode_sphere = new osg::Geode();
	//drawMeshSet( sphere_meshset, geode_sphere );
	group->addChild(geode_sphere);
	

	shared_ptr<IfcUShapeProfileDef> u_profile( new IfcUShapeProfileDef() );
	u_profile->m_Depth = shared_ptr<IfcPositiveLengthMeasure>( new IfcPositiveLengthMeasure() );
	u_profile->m_Depth->m_value = 0.24;
	u_profile->m_FlangeWidth = shared_ptr<IfcPositiveLengthMeasure>( new IfcPositiveLengthMeasure() );
	u_profile->m_FlangeWidth->m_value = 0.09;
	
	u_profile->m_WebThickness = shared_ptr<IfcPositiveLengthMeasure>( new IfcPositiveLengthMeasure() );
	u_profile->m_WebThickness->m_value = 0.007;
	
	u_profile->m_FlangeThickness = shared_ptr<IfcPositiveLengthMeasure>( new IfcPositiveLengthMeasure() );
	u_profile->m_FlangeThickness->m_value = 0.0125;

	u_profile->m_FilletRadius = shared_ptr<IfcNonNegativeLengthMeasure>( new IfcNonNegativeLengthMeasure() );
	u_profile->m_FilletRadius->m_value = 0.015;
	
	u_profile->m_EdgeRadius = shared_ptr<IfcNonNegativeLengthMeasure>( new IfcNonNegativeLengthMeasure() );
	u_profile->m_EdgeRadius->m_value = 0.01;
	
	u_profile->m_FlangeSlope = shared_ptr<IfcPlaneAngleMeasure>( new IfcPlaneAngleMeasure() );
	u_profile->m_FlangeSlope->m_value = 0.0;

	shared_ptr<IfcExtrudedAreaSolid> solid( new IfcExtrudedAreaSolid() );
	solid->m_SweptArea = u_profile;
	solid->m_ExtrudedDirection = shared_ptr<IfcDirection>( new IfcDirection() );
	solid->m_ExtrudedDirection->m_DirectionRatios.push_back( 0 );
	solid->m_ExtrudedDirection->m_DirectionRatios.push_back( 0 );
	solid->m_ExtrudedDirection->m_DirectionRatios.push_back( 1 );
	solid->m_Depth = shared_ptr<IfcPositiveLengthMeasure>( new IfcPositiveLengthMeasure() );
	solid->m_Depth->m_value = 3.0;

	shared_ptr<ItemData> item_data2( new ItemData() );
	representation_converter->getSolidConverter()->convertIfcExtrudedAreaSolid( solid, carve::math::Matrix::TRANS(0.7, 0.0, 0.0), item_data2 );
	item_data2->createMeshSetsFromClosedPolyhedrons();


	if( item_data2->item_meshsets.size() != 1 )
	{
		return;
	}

	shared_ptr<carve::mesh::MeshSet<3> > solid_meshset = item_data2->item_meshsets[0];
	osg::Geode* geode_solid = new osg::Geode();

	shared_ptr<carve::mesh::MeshSet<3> > cut_box1( makeCube( carve::math::Matrix::TRANS(0.3, 0.0, 0.0)*carve::math::Matrix::ROT( 0.8, 1, 0, 0 ) ) );
	shared_ptr<carve::mesh::MeshSet<3> > cut_box2( makeCube( carve::math::Matrix::TRANS(0.8, 0.0, 2.0)*carve::math::Matrix::ROT( 0.8, 1, 0, 0 )*carve::math::Matrix::SCALE(0.3,0.3,0.3) ) );
	//drawMeshSet( cut_box2, geode_solid );

	carve::csg::CSG csg6;
	csg6.hooks.registerHook(new carve::csg::CarveTriangulatorWithImprovement, carve::csg::CSG::Hooks::PROCESS_OUTPUT_FACE_BIT);
	shared_ptr<carve::mesh::MeshSet<3> > solid_result( csg6.compute(solid_meshset.get(), cut_box1.get(), carve::csg::CSG::A_MINUS_B) );

	carve::csg::CSG csg7;
	csg7.hooks.registerHook(new carve::csg::CarveTriangulatorWithImprovement, carve::csg::CSG::Hooks::PROCESS_OUTPUT_FACE_BIT);
	solid_result = shared_ptr<carve::mesh::MeshSet<3> >( csg7.compute(solid_result.get(), cut_box2.get(), carve::csg::CSG::A_MINUS_B) );

	drawMeshSet( solid_result, geode_solid );
	group->addChild(geode_solid);


	int slices = 16;
	double rad = 1.5;
	double height = 2.0;
	shared_ptr<carve::mesh::MeshSet<3> > opening( makeCone( slices, rad*0.5, height*2, carve::math::Matrix::TRANS(0.0, 0.0, 0.0)) );
	shared_ptr<carve::mesh::MeshSet<3> > subtract_from( makeCube( carve::math::Matrix::TRANS(0.3, 0.0, 0.0)) );
	shared_ptr<carve::mesh::MeshSet<3> > another_opening( makeCone( slices, rad*0.5, height*2, carve::math::Matrix::TRANS(0.5, 0.5, -0.5)) );
	shared_ptr<carve::mesh::MeshSet<3> > cut_box( makeCube( carve::math::Matrix::TRANS(0.3, 0.0, 0.0)*carve::math::Matrix::ROT( 0.8, 1, 0, 0 ) ) );

	carve::csg::CSG csg;
	csg.hooks.registerHook(new carve::csg::CarveTriangulatorWithImprovement, carve::csg::CSG::Hooks::PROCESS_OUTPUT_FACE_BIT);
	shared_ptr<carve::mesh::MeshSet<3> > result( csg.compute(subtract_from.get(), opening.get(), carve::csg::CSG::A_MINUS_B) );

	carve::csg::CSG csg2;
	csg2.hooks.registerHook(new carve::csg::CarveTriangulatorWithImprovement, carve::csg::CSG::Hooks::PROCESS_OUTPUT_FACE_BIT);
	result = shared_ptr<carve::mesh::MeshSet<3> >( csg2.compute(result.get(), another_opening.get(), carve::csg::CSG::A_MINUS_B) );

	carve::csg::CSG csg3;
	csg3.hooks.registerHook(new carve::csg::CarveTriangulatorWithImprovement, carve::csg::CSG::Hooks::PROCESS_OUTPUT_FACE_BIT);
	result = shared_ptr<carve::mesh::MeshSet<3> >( csg3.compute(result.get(), sphere_meshset.get(), carve::csg::CSG::UNION) );

	carve::csg::CSG csg4;
	csg4.hooks.registerHook(new carve::csg::CarveTriangulatorWithImprovement, carve::csg::CSG::Hooks::PROCESS_OUTPUT_FACE_BIT);
	result = shared_ptr<carve::mesh::MeshSet<3> >( csg4.compute(result.get(), cut_box.get(), carve::csg::CSG::INTERSECTION) );

	carve::csg::CSG csg5;
	csg5.hooks.registerHook(new carve::csg::CarveTriangulatorWithImprovement, carve::csg::CSG::Hooks::PROCESS_OUTPUT_FACE_BIT);
	result = shared_ptr<carve::mesh::MeshSet<3> >( csg5.compute(result.get(), solid_meshset.get(), carve::csg::CSG::A_MINUS_B) );

	{
		osg::Geode* geode = new osg::Geode();

		drawMeshSet( shared_ptr<carve::mesh::MeshSet<3> >( opening ), geode );

		osg::MatrixTransform* mt = new osg::MatrixTransform( osg::Matrix::translate( 10, 10, 0 ) );
		mt->addChild(geode);
		group->addChild(mt);
	}

	{
		osg::Geode* geode = new osg::Geode();
		drawMeshSet( shared_ptr<carve::mesh::MeshSet<3> >( subtract_from ), geode );

		osg::MatrixTransform* mt = new osg::MatrixTransform( osg::Matrix::translate( -10, 10, 0 ) );
		mt->addChild(geode);
		group->addChild(mt);
	}
	{
		osg::Geode* geode = new osg::Geode();
		drawMeshSet( shared_ptr<carve::mesh::MeshSet<3> >( result ), geode );
		group->addChild(geode);
	}
}

void ConverterOSG::createTest2(osg::Group* group)
{
	int slices = 16;
	double rad = 1.5;
	double height = 2.0;
	carve::mesh::MeshSet<3> *cone = makeCone( slices, rad, height, carve::math::Matrix::TRANS(0.0, 0.0, 0.0));

	{
		osg::Geode* geode = new osg::Geode();
		//drawPolyhedron( cone, geode );

		osg::MatrixTransform* mt = new osg::MatrixTransform( osg::Matrix::translate( 10, 10, 0 ) );
		mt->addChild(geode);
		group->addChild(mt);
	}

	height = 1.5;
	carve::mesh::MeshSet<3> *cyl = makeCylinder( slices, rad, height, carve::math::Matrix::TRANS(1.0, 0.0, 0.0));

	std::list<carve::mesh::MeshSet<3> *> a_sliced, b_sliced;
	carve::csg::V2Set shared_edges;
	carve::csg::CSG csg;

	csg.slice(cone, cyl, a_sliced, b_sliced, &shared_edges);
	std::cerr << "result: " << a_sliced.size() << " connected components from a" << std::endl;
	std::cerr << "      : " << b_sliced.size() << " connected components from b" << std::endl;
	std::cerr << "      : " << shared_edges.size() << " edges in the line of intersection" << std::endl;


	std::list<carve::mesh::MeshSet<3> *>::iterator it;
	double x = 10;
	for( it=b_sliced.begin(); it!= b_sliced.end(); ++it )
	{
		if( it == b_sliced.begin() )
		{
			//continue;
		}

		carve::mesh::MeshSet<3>* meshset = (*it);

		osg::Geode* geode = new osg::Geode();
		osg::MatrixTransform* mt = new osg::MatrixTransform( osg::Matrix::translate( 0, 0, 0 ) );
		x += 3;
		mt->addChild(geode);
		group->addChild(mt);

		drawMeshSet( shared_ptr<carve::mesh::MeshSet<3> >( meshset ), geode );

	}
}

void ConverterOSG::createTest4(osg::Group* group)
{
	if( false )
	{
		std::vector<carve::triangulate::tri_idx> triangulated;
		std::vector<carve::geom2d::P2> merged;
		merged.push_back( carve::geom::VECTOR( 0.21, 0.21 ) );
		merged.push_back( carve::geom::VECTOR( 2.06, -1.02 ) );
		merged.push_back( carve::geom::VECTOR( 4.24, -1.45 ) );
		merged.push_back( carve::geom::VECTOR( 6.42, -1.02 ) );
		merged.push_back( carve::geom::VECTOR( 8.27,  0.21 ) );
		merged.push_back( carve::geom::VECTOR( 8.45,  0.0 ) );
		merged.push_back( carve::geom::VECTOR( 6.53, -1.30 ) );
		merged.push_back( carve::geom::VECTOR( 4.24, -1.75 ) );
		merged.push_back( carve::geom::VECTOR( 1.94, -1.30 ) );
		merged.push_back( carve::geom::VECTOR( 0.0, 0.0 ) );
		merged.push_back( carve::geom::VECTOR( 0.21, 0.21 ) );
		//ProfileConverter::deleteLastPointIfEqualToFirst( merged );
		carve::geom::vector<3> normal = GeomUtils::computePolygon2DNormal( merged );
		if( normal.z < 0 )
		{
			std::reverse( merged.begin(), merged.end() );
		}

		carve::triangulate::triangulate(merged, triangulated);
		carve::triangulate::improve(merged, triangulated);

		carve::input::PolyhedronData poly_data;
		for( int i=0; i<merged.size(); ++i )
		{
			poly_data.addVertex( carve::geom::VECTOR( merged[i].x, merged[i].y, 0 ) );
		}
		for( size_t i = 0; i != triangulated.size(); ++i )
		{
			carve::triangulate::tri_idx triangle = triangulated[i];
			int a = triangle.a;
			int b = triangle.b;
			int c = triangle.c;

			poly_data.addFace( a, b, c );
		}

		osg::Geode* geode = new osg::Geode();
		group->addChild( geode );

		shared_ptr<carve::mesh::MeshSet<3> > meshset( poly_data.createMesh(carve::input::opts()) );
		drawMeshSet( meshset, geode );
	}

	{

		std::vector<carve::geom::vector<2> > merged;
		merged.push_back( carve::geom::VECTOR( 0.21, 0.21 ) );
		merged.push_back( carve::geom::VECTOR( 2.06, -1.02 ) );
		merged.push_back( carve::geom::VECTOR( 4.24, -1.45 ) );
		merged.push_back( carve::geom::VECTOR( 6.42, -1.02 ) );
		merged.push_back( carve::geom::VECTOR( 8.27,  0.21 ) );
		merged.push_back( carve::geom::VECTOR( 8.45,  0.0 ) );
		merged.push_back( carve::geom::VECTOR( 6.53, -1.30 ) );
		merged.push_back( carve::geom::VECTOR( 4.24, -1.75 ) );
		merged.push_back( carve::geom::VECTOR( 1.94, -1.30 ) );
		merged.push_back( carve::geom::VECTOR( 0.0,  0.0 ) );
		merged.push_back( carve::geom::VECTOR( 0.21, 0.21 ) );

		std::vector<std::vector<carve::geom::vector<2> > > face_loops;
		face_loops.push_back( merged );

		carve::geom::vector<3> extrusion_vector( carve::geom::VECTOR( 0.0, 0.0,	-2.78 ) );
		shared_ptr<carve::input::PolyhedronData> poly_data( new carve::input::PolyhedronData() );
		std::stringstream err;
		GeomUtils::extrude( face_loops, extrusion_vector, poly_data, err );

		osg::Geode* geode = new osg::Geode();
		group->addChild( geode );

		shared_ptr<carve::mesh::MeshSet<3> > meshset( poly_data->createMesh(carve::input::opts()) );

		checkMeshSet( meshset, err, -1 );
		drawMeshSet( meshset, geode );

	}

	carve::input::PolyhedronData poly_data;

	{
		poly_data.addVertex( carve::geom::VECTOR(1, 0, 0 ));
		poly_data.addVertex( carve::geom::VECTOR(0, 0, 0 ));
		poly_data.addVertex( carve::geom::VECTOR(0, 0.1, 0 ));
		poly_data.addVertex( carve::geom::VECTOR(1, 0.1, 0 ));


		poly_data.addVertex( carve::geom::VECTOR(0, 0, 1 ));
		poly_data.addVertex( carve::geom::VECTOR(1, 0, 1 ));
		poly_data.addVertex( carve::geom::VECTOR(1, 0.1, 1 ));
		poly_data.addVertex( carve::geom::VECTOR(0, 0.1, 1 ));

		poly_data.addFace(4,5,6);
		poly_data.addFace(6,7,4);
		poly_data.addFace(1,2,3);
		poly_data.addFace(3,0,1);
		poly_data.addFace(6,5,0);
		poly_data.addFace(0,3,6);
		poly_data.addFace(1,4,7);
		poly_data.addFace(7,2,1);
		poly_data.addFace(7,6,3);
		poly_data.addFace(3,2,7);
		poly_data.addFace(5,4,1);
		poly_data.addFace(1,0,5);
	}


	carve::input::PolyhedronData poly_opening_data;
	{

		poly_opening_data.addVertex( carve::geom::VECTOR(0.9,	0,	0 ));
		poly_opening_data.addVertex( carve::geom::VECTOR(0.3,	0,	0 ));
		poly_opening_data.addVertex( carve::geom::VECTOR(0.3,	0.1, 0 ));
		poly_opening_data.addVertex( carve::geom::VECTOR(0.9,	0.1, 0 ));


		poly_opening_data.addVertex( carve::geom::VECTOR(0.3, 0, 1 ));
		poly_opening_data.addVertex( carve::geom::VECTOR(0.9, 0, 1 ));
		poly_opening_data.addVertex( carve::geom::VECTOR(0.9, 0.1, 1 ));
		poly_opening_data.addVertex( carve::geom::VECTOR(0.3, 0.1, 1 ));

		poly_opening_data.addFace(4,5,6);
		poly_opening_data.addFace(6,7,4);
		poly_opening_data.addFace(1,2,3);
		poly_opening_data.addFace(3,0,1);
		poly_opening_data.addFace(6,5,0);
		poly_opening_data.addFace(0,3,6);
		poly_opening_data.addFace(1,4,7);
		poly_opening_data.addFace(7,2,1);
		poly_opening_data.addFace(7,6,3);
		poly_opening_data.addFace(3,2,7);
		poly_opening_data.addFace(5,4,1);
		poly_opening_data.addFace(1,0,5);
	}

	shared_ptr<carve::poly::Polyhedron> poly( poly_data.create(carve::input::opts()) );
	shared_ptr<carve::poly::Polyhedron> poly_opening( poly_opening_data.create(carve::input::opts()) );

	osg::Geode* geode = new osg::Geode();
	osg::MatrixTransform* mt = new osg::MatrixTransform( osg::Matrix::translate( 0, 0, 0 ) );
	mt->addChild(geode);
	group->addChild(mt);

	drawPolyhedron( poly, geode );
	drawPolyhedron( poly_opening, geode );

	carve::csg::CSG csg;
	csg.hooks.registerHook(new carve::csg::CarveTriangulator, carve::csg::CSG::Hooks::PROCESS_OUTPUT_FACE_BIT);
	{
		osg::Geode* geode = new osg::Geode();
		osg::MatrixTransform* mt = new osg::MatrixTransform( osg::Matrix::translate( 0, 2, 0 ) );
		mt->addChild(geode);
		group->addChild(mt);

		shared_ptr<carve::mesh::MeshSet<3> > result( csg.compute( poly_data.createMesh(carve::input::opts()), poly_opening_data.createMesh(carve::input::opts()), carve::csg::CSG::A_MINUS_B) );
		drawMeshSet( result, geode );
	}
}

