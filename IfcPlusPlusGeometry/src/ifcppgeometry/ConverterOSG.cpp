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

void ConverterOSG::drawMeshSet( const shared_ptr<carve::mesh::MeshSet<3> >& meshset, osg::Geode* geode, bool add_color_array )
{
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
		err_poly << "MeshSet of entity #" << entity_id << " not valid" << std::endl;
		return false;
	}

	std::stringstream err;
	if( mesh_set->meshes.size() == 0 )
	{
		err << "mesh_set->meshes.size() == 0" << std::endl;
	}

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
void ConverterOSG::createTest( osg::Group* group )
{
	{
		shared_ptr<IfcSphere> sphere( new IfcSphere() );
		sphere->m_Radius = shared_ptr<IfcPositiveLengthMeasure>( new IfcPositiveLengthMeasure() );
		sphere->m_Radius->m_value = 0.75;

		shared_ptr<ItemData> item_data( new ItemData() );
		
		shared_ptr<GeometrySettings> geom_settings( new GeometrySettings() );
		geom_settings->m_num_vertices_per_circle = 50;
		shared_ptr<UnitConverter> unit_converter( new UnitConverter() );
		shared_ptr<RepresentationConverter> representation_converter( new RepresentationConverter( geom_settings, unit_converter ) );

		representation_converter->getSolidConverter()->convertIfcCsgPrimitive3D( sphere, carve::math::Matrix::TRANS(2.0, 3.0, 4.0), item_data );
		item_data->createMeshSetsFromClosedPolyhedrons();

		for( int i=0; i<item_data->item_meshsets.size(); ++i )
		{
			osg::Geode* geode = new osg::Geode();
			
			drawMeshSet( item_data->item_meshsets[i], geode );
			group->addChild(geode);
		}
		return;
	}

	osg::Geode* geode = new osg::Geode();
	int slices = 16;
	double rad = 1.5;
	double height = 2.0;
	carve::mesh::MeshSet<3>* opening = makeCone( slices, rad*0.5, height*2, carve::math::Matrix::TRANS(0.0, 0.0, 0.0));
	carve::mesh::MeshSet<3>* subtract_from = makeCube( carve::math::Matrix::TRANS(0.3, 0.0, 0.0));

	carve::csg::CSG csg;
	csg.hooks.registerHook(new carve::csg::CarveTriangulator, carve::csg::CSG::Hooks::PROCESS_OUTPUT_FACE_BIT);
	shared_ptr<carve::mesh::MeshSet<3> > result( csg.compute(subtract_from, opening, carve::csg::CSG::A_MINUS_B) );

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

	group->addChild(geode);

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

