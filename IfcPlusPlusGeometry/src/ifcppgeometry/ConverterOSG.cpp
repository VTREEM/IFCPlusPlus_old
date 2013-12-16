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
#include "common/geometry.hpp"

#include "ifcpp/model/IfcPPException.h"
#include "Utility.h"
#include "ProfileConverter.h"
#include "ConverterOSG.h"

//#define DEBUG_DRAW_NORMALS

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
		throw IfcPPException( "ConverterOSG::drawFace is meant only for num vertices > 4" );
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

	osg::Vec3f poly_normal = computePolygonNormal( vertices );
	osg::Vec3Array* normals = new osg::Vec3Array();
	normals->resize( num_vertices, poly_normal );


	osg::Geometry* geometry = new osg::Geometry();
	geometry->setVertexArray( vertices );
	geometry->setNormalArray( normals );

	geometry->setNormalBinding( osg::Geometry::BIND_PER_VERTEX );
	geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POLYGON,0,vertices->size()));
	//
	//new osg::DrawElementsUInt( osg::PrimitiveSet::POLYGON,0,vertices->size()));
	

	//geometry->addPrimitiveSet( triangles );

	if( add_color_array )
	{
		osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array();
		colors->resize( vertices->size(), osg::Vec4f( 0.6f, 0.6f, 0.6f, 0.1f ) );
		
		geometry->setColorArray( colors );
		geometry->setColorBinding( osg::Geometry::BIND_PER_VERTEX );
	}


	osg::ref_ptr<osgUtil::Tessellator> tesselator = new osgUtil::Tessellator();
	tesselator->setTessellationType(osgUtil::Tessellator::TESS_TYPE_POLYGONS);
	tesselator->setWindingType( osgUtil::Tessellator::TESS_WINDING_ODD );
	tesselator->retessellatePolygons(*geometry);
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
		osg::Geometry* geometry = new osg::Geometry();
		geometry->setVertexArray( vertices_tri );
		
		geometry->setNormalArray( normals_tri );
		geometry->setNormalBinding( osg::Geometry::BIND_PER_VERTEX );

		if( add_color_array )
		{
			osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array();
			colors->resize( vertices_tri->size(), osg::Vec4f( 0.6f, 0.6f, 0.6f, 0.1f ) );

			geometry->setColorArray( colors );
			geometry->setColorBinding( osg::Geometry::BIND_PER_VERTEX );
		}
		
		geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::TRIANGLES,0,vertices_tri->size()));
		geode->addDrawable( geometry );
	}

	if( vertices_quad->size() > 0 )
	{
		osg::Geometry* geometry = new osg::Geometry();
		geometry->setVertexArray( vertices_quad );

		geometry->setNormalArray( normals_quad );
		geometry->setNormalBinding( osg::Geometry::BIND_PER_VERTEX );

		if( add_color_array )
		{
			osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array();
			colors->resize( vertices_quad->size(), osg::Vec4f( 0.6f, 0.6f, 0.6f, 0.1f ) );

			geometry->setColorArray( colors );
			geometry->setColorBinding( osg::Geometry::BIND_PER_VERTEX );
		}

		geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::QUADS,0,vertices_quad->size()));
		geode->addDrawable( geometry );
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
	carve::input::Options carve_options;
	carve::line::PolylineSet* polyline_set = polyline_data->create(carve_options);

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

bool ConverterOSG::checkMeshSet( shared_ptr<carve::mesh::MeshSet<3> >& mesh_set, std::stringstream& err_poly, int entity_id )
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
	}


	if( !meshes_closed )
	{
#ifdef _DEBUG
		std::cout << "mesh_set not closed" << std::endl;
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

void ConverterOSG::convertOsgGroup( const osg::Group* src, carve::input::PolyhedronData& target )
{
	int num_children = src->getNumChildren();
	for( int i=0; i<num_children; ++i )
	{
		const osg::Node* node = src->getChild(i);
		const osg::Group* child_group = dynamic_cast<const osg::Group*>(node);
		if( child_group )
		{
			convertOsgGroup( child_group, target );
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
				if( child_gemetry )
				{
					const osg::Array* vertices_array = child_gemetry->getVertexArray();
					const osg::Vec3dArray* vertices_d = dynamic_cast<const osg::Vec3dArray*>(vertices_array);
					
					const int vertex_offset = target.getVertexCount();

					if( vertices_d )
					{
						const int num_vertices = vertices_d->size();
						for( int i_vertex=0; i_vertex<num_vertices; ++i_vertex )
						{
							const osg::Vec3d& vertex = vertices_d->at(i_vertex);
							target.addVertex(carve::geom::VECTOR(vertex.x(), vertex.y(), vertex.z() ) );

						}
					}
					else
					{
						const osg::Vec3Array* vertices_float = dynamic_cast<const osg::Vec3Array*>(vertices_array);
						if( vertices_float )
						{
							const int num_vertices = vertices_float->size();
							for( int i_vertex=0; i_vertex<num_vertices; ++i_vertex )
							{
								const osg::Vec3& vertex = vertices_float->at(i_vertex);
								target.addVertex(carve::geom::VECTOR(vertex.x(), vertex.y(), vertex.z() ) );
							}
						}
						else
						{
							continue;
						}
					}

					const osg::Geometry::PrimitiveSetList& primitive_sets = child_gemetry->getPrimitiveSetList();
					osg::Geometry::PrimitiveSetList::const_iterator it_primitives;
					for( it_primitives=primitive_sets.begin(); it_primitives!=primitive_sets.end(); ++it_primitives )
					{
						const osg::PrimitiveSet* p_set = (*it_primitives);

						const osg::DrawElementsUInt* elements = dynamic_cast<const osg::DrawElementsUInt*>(p_set);
						if( elements )
						{
							if( elements->getMode() == osg::PrimitiveSet::QUADS )
							{

								const osg::DrawElementsUInt& ele_ref = (*elements);
								const int num_elements = elements->size();
								if( elements->size() > 3 )
								{
									for( int k=0; k<num_elements-3; k+=4 )
									{
										int p0 = ele_ref[k] + vertex_offset;
										int p1 = ele_ref[k+1] + vertex_offset;
										int p2 = ele_ref[k+2] + vertex_offset;
										int p3 = ele_ref[k+3] + vertex_offset;

										carve::geom::vector<3> v0 = target.getVertex(p0);
										carve::geom::vector<3> v1 = target.getVertex(p1);
										carve::geom::vector<3> v2 = target.getVertex(p2);
										carve::geom::vector<3> v3 = target.getVertex(p3);
										osg::Vec3 pt0(v0.v[0],v0.v[1],v0.v[2]);
										osg::Vec3 pt1(v1.v[0],v1.v[1],v1.v[2]);
										osg::Vec3 pt2(v2.v[0],v2.v[1],v2.v[2]);
										osg::Vec3 pt3(v3.v[0],v3.v[1],v3.v[2]);

										if( (pt0 -pt1).length2() < 0.00001 )
										{
											continue;
										}
										if( (pt1 -pt2).length2() < 0.00001 )
										{
											continue;
										}
										if( (pt2 -pt3).length2() < 0.00001 )
										{
											continue;
										}
										if( (pt3 -pt0).length2() < 0.00001 )
										{
											continue;
										}
										std::vector<int> face_verts;
										face_verts.push_back(p0);
										face_verts.push_back(p1);
										face_verts.push_back(p2);
										face_verts.push_back(p3);
										target.addFace(face_verts.begin(), face_verts.end());

									}
								}
							

							}
							else if( elements->getMode() == osg::PrimitiveSet::TRIANGLES )
							{
								const osg::DrawElementsUInt& ele_ref = (*elements);
								const int num_elements = elements->size();
								if( elements->size() > 3 )
								{
									for( int k=0; k<num_elements-2; k+=3 )
									{
										int p0 = ele_ref[k] + vertex_offset;
										int p1 = ele_ref[k+1] + vertex_offset;
										int p2 = ele_ref[k+2] + vertex_offset;
										
										std::vector<int> face_verts;
										face_verts.push_back(p0);
										face_verts.push_back(p1);
										face_verts.push_back(p2);
										target.addFace(face_verts.begin(), face_verts.end());

										carve::geom::vector<3>  v0 = target.getVertex(p0);
										carve::geom::vector<3>  v1 = target.getVertex(p1);
										carve::geom::vector<3>  v2 = target.getVertex(p2);
										osg::Vec3 pt0(v0.v[0],v0.v[1],v0.v[2]);
										osg::Vec3 pt1(v1.v[0],v1.v[1],v1.v[2]);
										osg::Vec3 pt2(v2.v[0],v2.v[1],v2.v[2]);
										
										if( (pt0 -pt1).length2() < 0.00001 )
										{
											continue;
										}
										if( (pt1 -pt2).length2() < 0.00001 )
										{
											continue;
										}
										if( (pt2 -pt0).length2() < 0.00001 )
										{
											continue;
										}
									}
								}

							}
							else if( elements->getMode() == osg::PrimitiveSet::TRIANGLE_STRIP )
							{
								const osg::DrawElementsUInt& ele_ref = (*elements);
								const int num_elements = elements->size();
								if( elements->size() > 3 )
								{
									for( int k=0; k<num_elements-3; k+=2 )
									{
										int p0 = ele_ref[k] + vertex_offset;
										int p1 = ele_ref[k+1] + vertex_offset;
										int p2 = ele_ref[k+2] + vertex_offset;
										int p3 = ele_ref[k+3] + vertex_offset;
										
										carve::geom::vector<3>  v0 = target.getVertex(p0);
										carve::geom::vector<3>  v1 = target.getVertex(p1);
										carve::geom::vector<3>  v2 = target.getVertex(p2);
										carve::geom::vector<3>  v3 = target.getVertex(p3);
										osg::Vec3 pt0(v0.v[0],v0.v[1],v0.v[2]);
										osg::Vec3 pt1(v1.v[0],v1.v[1],v1.v[2]);
										osg::Vec3 pt2(v2.v[0],v2.v[1],v2.v[2]);
										osg::Vec3 pt3(v3.v[0],v3.v[1],v3.v[2]);

										if( (pt0 -pt1).length2() < 0.00001 )
										{
											continue;
										}
										if( (pt1 -pt2).length2() < 0.00001 )
										{
											continue;
										}
										if( (pt2 -pt3).length2() < 0.00001 )
										{
											continue;
										}
										if( (pt3 -pt0).length2() < 0.00001 )
										{
											continue;
										}
										std::vector<int> face_verts;
										face_verts.push_back(p0);
										face_verts.push_back(p1);
										face_verts.push_back(p2);
										target.addFace(face_verts.begin(), face_verts.end());


										std::vector<int> face_verts1;
										face_verts1.push_back(p1);
										face_verts1.push_back(p3);
										face_verts1.push_back(p2);
										target.addFace(face_verts1.begin(), face_verts1.end());
									}
								}
							}
						}
					}
				}
			}
		}
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define POINTS 10
void ConverterOSG::createTest( osg::Group* group )
{
	osg::Geode* geode = new osg::Geode();
	int slices = 16;
	double rad = 1.5;
	double height = 2.0;
	carve::mesh::MeshSet<3>* opening = makeCone( slices, rad*0.5, height*2, carve::math::Matrix::TRANS(0.0, 0.0, 0.0));
	carve::mesh::MeshSet<3>* subtract_from = makeCube( carve::math::Matrix::TRANS(0.3, 0.0, 0.0));

	std::map<const carve::poly::Polyhedron *, int> counter;
	carve::csg::CSG csg1;

	csg1.hooks.registerHook(new carve::csg::CarveTriangulator, carve::csg::CSG::Hooks::PROCESS_OUTPUT_FACE_BIT);
	shared_ptr<carve::mesh::MeshSet<3> > result( csg1.compute(subtract_from, opening, carve::csg::CSG::A_MINUS_B, NULL, carve::csg::CSG::CLASSIFY_EDGE) );

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
		carve::geom::vector<3> normal = computePolygon2DNormal( merged );
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

		carve::input::Options carve_options;
		shared_ptr<carve::mesh::MeshSet<3> > meshset( poly_data.createMesh(carve_options) );
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
		extrude( face_loops, extrusion_vector, poly_data, err );

		osg::Geode* geode = new osg::Geode();
		group->addChild( geode );

		carve::input::Options carve_options;
		shared_ptr<carve::mesh::MeshSet<3> > meshset( poly_data->createMesh(carve_options) );

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

	carve::input::Options carve_options;
	shared_ptr<carve::poly::Polyhedron> poly( poly_data.create(carve_options) );
	shared_ptr<carve::poly::Polyhedron> poly_opening( poly_opening_data.create(carve_options) );

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

		shared_ptr<carve::mesh::MeshSet<3> > result( csg.compute( poly_data.createMesh(carve_options), poly_opening_data.createMesh(carve_options), carve::csg::CSG::A_MINUS_B, NULL, carve::csg::CSG::CLASSIFY_EDGE) );
		drawMeshSet( result, geode );
	}
}

