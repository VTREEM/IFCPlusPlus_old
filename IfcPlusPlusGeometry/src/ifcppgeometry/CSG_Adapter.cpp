
#include <ifcpp/model/IfcPPException.h>
#include <ifcpp/model/IfcPPOpenMP.h>

#include "DebugViewerCallback.h"
#include "ConverterOSG.h"
#include "GeometryInputData.h"
#include "CSG_Adapter.h"

void collectAdjacentFacesSameNormal( const carve::mesh::Face<3>* face, const carve::geom::vector<3>& normal,
									std::vector<carve::mesh::Face<3>* >& vec_faces, 
									std::vector<carve::mesh::Edge<3>* >& vec_perimeter_edges)
{
	carve::mesh::Edge<3>* edge = face->edge;
	do
	{
		carve::mesh::Edge<3>* edge_reverse = edge->rev;
		carve::mesh::Face<3>* adjacent_face = edge_reverse->face;
		const carve::geom::vector<3>& adjacent_face_normal = adjacent_face->plane.N;
		const double cos_angle = dot( adjacent_face_normal, normal );
		if( abs( cos_angle + 1.0 ) < 0.000000001 || abs( cos_angle - 1.0 ) < 0.000000001 )
		{
			bool already_in_vec = false;
			for( int i=0; i<vec_faces.size(); ++i )
			{
				carve::mesh::Face<3>* f = vec_faces[i];
				if( f == adjacent_face )
				{
					already_in_vec = true;
					break;
				}
			}
			if( !already_in_vec )
			{
				vec_faces.push_back( adjacent_face );
				collectAdjacentFacesSameNormal( adjacent_face, normal, vec_faces, vec_perimeter_edges );
			}
		}
		else
		{
			vec_perimeter_edges.push_back( edge );
		}

		edge = edge->next;
	}while( edge != face->edge );
}


void collectFacesOppositeOrientation( const carve::geom::vector<3>& normal, const std::vector<carve::mesh::Face<3>* >& vec_faces_in, std::vector<carve::mesh::Face<3>* >& vec_opposite_faces )
{
	for( int i=0; i<vec_faces_in.size(); ++i )
	{
		carve::mesh::Face<3>* face = vec_faces_in[i];
		const carve::geom::vector<3>& face_normal = face->plane.N;
		const double cos_angle = dot( face_normal, normal );
		//if( abs( cos_angle + 1.0 ) < 0.000000001 || abs( cos_angle - 1.0 ) < 0.000000001 )
		if( abs( cos_angle + 1.0 ) < 0.000001 )
		{
			vec_opposite_faces.push_back( face );
		}
		if( abs( cos_angle - 1.0 ) > 0.0001 )
		{
			int wait=0;
		}
	}
}

void collectFinFaces( const carve::mesh::Face<3>* face, const carve::geom::vector<3>& normal, std::vector<carve::mesh::Face<3>* >& vec_faces, std::vector<carve::mesh::Edge<3>*>& vec_perimeter )
{
	carve::mesh::Edge<3>* edge = face->edge;
	do
	{
		carve::mesh::Edge<3>* edge_reverse = edge->rev;
		carve::mesh::Face<3>* adjacent_face = edge_reverse->face;
		const carve::geom::vector<3>& adjacent_face_normal = adjacent_face->plane.N;
		const double cos_angle = dot( adjacent_face_normal, normal );
		if( abs( cos_angle + 1.0 ) < 0.0000001 || abs( cos_angle - 1.0 ) < 0.0000001 )
		{
			bool already_in_vec = false;
			for( int i=0; i<vec_faces.size(); ++i )
			{
				carve::mesh::Face<3>* f = vec_faces[i];
				if( f == adjacent_face )
				{
					already_in_vec = true;
					break;
				}
			}
			if( !already_in_vec )
			{
				vec_faces.push_back( adjacent_face );
				collectAdjacentFacesSameNormal( adjacent_face, normal, vec_faces, vec_perimeter );
			}
		}
		else
		{
			vec_perimeter.push_back( edge );
		}

		edge = edge->next;
	}while( edge != face->edge );
}


void mergeCoplanarFaces( shared_ptr<carve::mesh::MeshSet<3> >& meshset )
{

	// TODO: merge coplanar faces
	//            v3
	//           /|\
	//          / |  \
	//         /  |    \
	//        /f2 |  f1  \
	//       /    v2------v1
	//      /    /|      /
	//     /   /  |  f0 /  
	//    /  /    |    /
	//   / /      |   /
	//  //   f3   |  /
	// v4---------v0

	// start with f0: check if adjacent face f1 has same normal
	// check if one of the edges of f1 is aligned with one of the edges of f0
	// if so, check if adjacent faces of aligned edges have same normal too
	// check if f0 and f3 have same normal, if so create triangle v1-v3-v4, omit f0,f1,f2,f3
	// else, create new triangles v0-v1-v2 and v3-v4-v0, omit f0,f1,f2,f3
}

void splitFace(carve::mesh::MeshSet<3>::face_t *face,
                        const carve::csg::V2Set &edges,
                        std::list<std::vector<carve::mesh::MeshSet<3>::vertex_t *> > &face_loops,
                        std::list<std::vector<carve::mesh::MeshSet<3>::vertex_t *> > &hole_loops,
                        const carve::csg::VertexIntersections & /* vi */);

int intersection_test_count = 0;
#ifdef IFCPP_OPENMP
	Mutex writelock_count;
#endif


void removeFins( shared_ptr<carve::mesh::MeshSet<3> >& meshset )
{
	if( !meshset )
	{
		return;
	}

	PolyInputCache3D poly_cache;
	double volume_check = 0;

	for( size_t i_mesh = 0; i_mesh < meshset->meshes.size(); ++i_mesh )
	{
		carve::mesh::Mesh<3>* mesh = meshset->meshes[i_mesh];
		volume_check += mesh->volume();
		const std::vector<carve::mesh::Face<3>* >& vec_faces = mesh->faces;
		std::map<double, std::vector<carve::mesh::Face<3>* > > map_face_distances;
		//std::map<carve::mesh::Face<3>*, std::vector<carve::geom::vector<3> > > map_face_subtract;
	
		for( size_t i2 = 0; i2 < vec_faces.size(); ++i2 )
		{
			carve::mesh::Face<3>* face = vec_faces[i2];
			const carve::geom::vector<3>& face_normal = face->plane.N;

			// check if there is a fin
			carve::mesh::Edge<3>* edge = face->edge;
			do
			{
				//const carve::geom::vector<3>& v = edge->vert->v;
				carve::mesh::Edge<3>* edge_reverse = edge->rev;
				carve::mesh::Face<3>* face_reverse = edge_reverse->face;
				const carve::geom::vector<3>& face_reverse_normal = face->plane.N;

				const double cos_angle = dot( face_normal, face_reverse_normal );
				if( abs( cos_angle + 1.0 ) < 0.000001 )
				{
					// fin detected
					// check if all vertices are same

					std::vector<carve::mesh::Face<3>* > vec_ad_faces;
					std::vector<carve::mesh::Edge<3>* > vec_perimeter_edges;
					collectAdjacentFacesSameNormal( face,  face_normal, vec_ad_faces, vec_perimeter_edges );

				}

				edge = edge->next;
						
			} while( edge != face->edge );

			//const double face_distance_origin = round(face->plane.d/round_distance)*round_distance;
			
			
			
			
		}
	}
}

#define DISTANCE_ROUND_UP 1000.0
#define DISTANCE_ROUND_DOWN	0.001

void removeThinCSGRemainings( shared_ptr<carve::mesh::MeshSet<3> >& meshset )
{
	if( !meshset )
	{
		return;
	}

	int count_begin = 0;
	{
#ifdef IFCPP_OPENMP
		ScopedLock lock( writelock_count );
#endif
		count_begin = intersection_test_count;
	}
	//std::vector<PolyInputCache3D> vec_poly_cache_subtact_fins;
	//std::map<carve::mesh::Face<3>*, std::vector<carve::geom::vector<3>* > > map_additional_face_openings;
	//std::map<int,int> map_merged_idx;

	int num_vertices1 = meshset->vertex_storage.size();
	PolyInputCache3D poly_cache;
	double volume_check = 0;

	const carve::geom::aabb<3>& meshset_aabb = meshset->getAABB();
	double round_distance = std::min( meshset_aabb.extent.x, std::min( meshset_aabb.extent.y, meshset_aabb.extent.z ) )*0.1;

	for( size_t i_mesh = 0; i_mesh < meshset->meshes.size(); ++i_mesh )
	{
		carve::mesh::Mesh<3>* mesh = meshset->meshes[i_mesh];
		volume_check += mesh->volume();
		const std::vector<carve::mesh::Face<3>* >& vec_faces = mesh->faces;
		std::map<double, std::vector<carve::mesh::Face<3>* > > map_face_distances;
		//std::map<carve::mesh::Face<3>*, std::vector<carve::geom::vector<3> > > map_face_subtract;
		std::set<carve::mesh::Face<3>* > set_unhandled_faces;

		for( size_t i2 = 0; i2 < vec_faces.size(); ++i2 )
		{
			carve::mesh::Face<3>* face = vec_faces[i2];
			//const double face_distance_origin = round(face->plane.d/round_distance)*round_distance;
			const double face_distance_origin = abs(round(face->plane.d*DISTANCE_ROUND_UP)*DISTANCE_ROUND_DOWN);
			std::vector<carve::mesh::Face<3>* >& vec_faces_same_distance = map_face_distances.insert( std::make_pair( face_distance_origin, std::vector<carve::mesh::Face<3>* >() ) ).first->second;
			vec_faces_same_distance.push_back( face );
			set_unhandled_faces.insert( face );
		}

		std::vector<carve::mesh::Vertex<3>*[3] > vec_additional_triangles;
		
		//for( size_t i2 = 0; i2 < vec_faces.size(); ++i2 )
		for( std::map<double, std::vector<carve::mesh::Face<3>* > >::iterator it = map_face_distances.begin(); it != map_face_distances.end(); ++it )
		{
			//carve::mesh::Face<3>* face = vec_faces[i2];
			std::vector<carve::mesh::Face<3>* >& vec_same_distance_faces = it->second;

			if( vec_same_distance_faces.size() == 1 )
			{
				continue;
			}

			//std::map<double, std::vector<carve::mesh::Face<3>* > > map_same_orientation_faces;
			

			for( size_t i_same_distance = 0; i_same_distance < vec_same_distance_faces.size(); ++i_same_distance )
			{
				//set_faces_done.insert( face );
				carve::mesh::Face<3>* face = vec_same_distance_faces[i_same_distance];
				bool face_processed = false;
				//const carve::geom::aabb<3>& face_aabb = face->getAABB();
				const carve::geom::vector<3>& face_normal = face->plane.N;

				// within set of same distance, find the faces that have opposite orientation
				std::vector<carve::mesh::Face<3>* > vec_opposite_orientation_faces;
				collectFacesOppositeOrientation( face_normal, vec_same_distance_faces, vec_opposite_orientation_faces );

				if( vec_opposite_orientation_faces.size() < 1 )
				{
					continue;
				}

				std::vector<carve::mesh::Vertex<3>* > face_vertices;
				face->getVertices( face_vertices );

				//if( vec_same_distance_faces.size() == 11 )
				//{
				//	set_unhandled_faces.erase( face );
				//	continue;

				//}

				if( face_vertices.size() != 3 )
				{
					std::cout << "not a triangle" << std::endl;
					continue;
				}

				std::vector<carve::geom::vector<2> > verts2d;
				face->getProjectedVertices( verts2d );
				bool omit_face = false;

				carve::mesh::Vertex<3>* face_vertex0 = face_vertices[0];
				carve::mesh::Vertex<3>* face_vertex1 = face_vertices[1];
				carve::mesh::Vertex<3>* face_vertex2 = face_vertices[2];

				std::vector<carve::mesh::Face<3>* > vec_ad_faces;
				std::vector<carve::mesh::Edge<3>* > vec_perimeter_edges;
				collectAdjacentFacesSameNormal( face,  face_normal, vec_ad_faces, vec_perimeter_edges );


				for( size_t i3 = 0; i3 < vec_opposite_orientation_faces.size(); ++i3 )
				{
					carve::mesh::Face<3>* opposite_face = vec_opposite_orientation_faces[i3];
					if( opposite_face  == face )
					{
						// shouldn't happen, but still check
						continue;
					}
					const carve::geom::vector<3>& opposite_face_normal = opposite_face->plane.N;


					//std::list<std::vector<carve::mesh::MeshSet<3>::vertex_t *> > face_loops;
					//std::list<std::vector<carve::mesh::MeshSet<3>::vertex_t *> > hole_loops;

					//carve::csg::V2Set face_split_edges;

					//// for (size_t j = 0; j < divided_base_loop[i].size() - 1; ++j) {
					////face_split_edges.insert(std::make_pair(divided_base_loop[i][j], divided_base_loop[i][j+1]));

					//splitFace(face, face_edges, face_loops, hole_loops, vertex_intersections);

					//std::vector<carve::mesh::Face<3>* >& vec_faces_same_distance = map_face_distances.insert( std::make_pair( face_distance_origin, std::vector<carve::mesh::Face<3>* >() ) ).first->second;
					//vec_faces_same_distance.push_back( face );

					//const double face_distance_origin = round(face->plane.d*DISTANCE_ROUND_UP)*DISTANCE_ROUND_DOWN;
					const double face_distance_origin = face->plane.d;
					//const double opposite_face_distance_origin = round(opposite_face->plane.d*DISTANCE_ROUND_UP)*DISTANCE_ROUND_DOWN;
					const double opposite_face_distance_origin = opposite_face->plane.d;
					if( abs(face_distance_origin+opposite_face_distance_origin) > 0.0001 )
					{
						// faces are not in same plane, but in opposite planes
						continue;
					}
			
#ifdef _DEBUG
					const double opposite_direction = dot( opposite_face_normal, face_normal );
					if( abs( opposite_direction + 1.0 ) > 0.000000001  )
					{
						std::cout << "orientation check failed" << std::endl;
					}
#endif

					bool create_subtract_poly = false;
					if( abs(face_distance_origin+opposite_face_distance_origin) > carve::EPSILON )
					{
						create_subtract_poly = true;
					}

					//if( !face_processed )
					//{
					//	set_unhandled_faces.erase( face );
					//	face_processed = true;
					//}


					// split triangles

					// collect ajacent faces with same normal
					std::vector<carve::mesh::Face<3>* > vec_opposite_plane_faces;
					std::vector<carve::mesh::Edge<3>* > vec_opposite_perimeter;
					collectAdjacentFacesSameNormal( opposite_face,  face_normal, vec_opposite_plane_faces, vec_opposite_perimeter );

					if( create_subtract_poly )
					{
						// project vec_opposite_plane_faces to face->plane -> poly1 
						// project vec_ad_faces to opposite_face->plane -> poly2
						// carve::meshset* subtract_mesh = carve::csg::compute( poly1, poly2, carve::csg::CSG::INTERSECTION, CLASSIFY_EDGE )
						// vec_subtract_meshsets.push_back( subtract_mesh );
						// foreach( face_opposite in vec_opposite_plane_faces) set_unhandled_faces.erase( face_opposite );
						// foreach( ad_face in vec_ad_faces) set_unhandled_faces.erase( ad_face );
						// std::vector<carve::mesh::meshset<3>* > vec_subtract_meshsets
						// meshset = carve::csg::compute( meshset, subtract_mesh, carve::csg::CSG::A_MINUS_B, CLASSIFY_EDGE )



					}
					else
					{
						// meshset mesh_adjacent, mesh_opposite
						// carve::meshset* subtract_mesh = carve::csg::compute( mesh_adjacent, mesh_opposite, carve::csg::CSG::INTERSECTION, CLASSIFY_EDGE )
						// vec_subtract_meshsets.push_back( subtract_mesh );
						// foreach( face_opposite in vec_opposite_plane_faces) set_unhandled_faces.erase( face_opposite );
						// foreach( ad_face in vec_ad_faces) set_unhandled_faces.erase( ad_face );
						// std::vector<carve::mesh::meshset<3>* > vec_subtract_meshsets
						// meshset = carve::csg::compute( meshset, subtract_mesh, carve::csg::CSG::A_MINUS_B, CLASSIFY_EDGE )
					}

					/*
					// project and compute triangle intersection
					std::vector<carve::geom::vector<2> > verts2d_opposite;
					opposite_face->getProjectedVertices( verts2d_opposite );

					std::vector<carve::geom::vector<2> > intersection_result_points;
					//carve::geom::vector<2> intersection_result_p1;
					//carve::geom::vector<2> intersection_result_p2;
					//set_faces_done.insert( same_plane_face );

					{
					#ifdef IFCPP_OPENMP
						ScopedLock lock( writelock_count );
					#endif
						++intersection_test_count;
					}
						//enum LineIntersectionClass {  COLINEAR        = -1, NO_INTERSECTION = 0, INTERSECTION_LL = 1, INTERSECTION_PL = 2,  INTERSECTION_LP = 3, INTERSECTION_PP = 4 };
						//                                                            const P2 &l1v1, const P2 &l1v2, const P2 &l2v1, const P2 &l2v2
					carve::geom2d::LineIntersectionInfo* info_array[3][3];
					try
					{
						carve::geom2d::LineIntersectionInfo info00 = carve::geom2d::lineSegmentIntersection( verts2d[0], verts2d[1], verts2d_opposite[0], verts2d_opposite[1] );
						carve::geom2d::LineIntersectionInfo info01 = carve::geom2d::lineSegmentIntersection( verts2d[0], verts2d[1], verts2d_opposite[1], verts2d_opposite[2] );
						carve::geom2d::LineIntersectionInfo info02 = carve::geom2d::lineSegmentIntersection( verts2d[0], verts2d[1], verts2d_opposite[2], verts2d_opposite[0] );

						carve::geom2d::LineIntersectionInfo info10 = carve::geom2d::lineSegmentIntersection( verts2d[1], verts2d[2], verts2d_opposite[0], verts2d_opposite[1] );
						carve::geom2d::LineIntersectionInfo info11 = carve::geom2d::lineSegmentIntersection( verts2d[1], verts2d[2], verts2d_opposite[1], verts2d_opposite[2] );
						carve::geom2d::LineIntersectionInfo info12 = carve::geom2d::lineSegmentIntersection( verts2d[1], verts2d[2], verts2d_opposite[2], verts2d_opposite[0] );

						carve::geom2d::LineIntersectionInfo info20 = carve::geom2d::lineSegmentIntersection( verts2d[2], verts2d[0], verts2d_opposite[0], verts2d_opposite[1] );
						carve::geom2d::LineIntersectionInfo info21 = carve::geom2d::lineSegmentIntersection( verts2d[2], verts2d[0], verts2d_opposite[1], verts2d_opposite[2] );
						carve::geom2d::LineIntersectionInfo info22 = carve::geom2d::lineSegmentIntersection( verts2d[2], verts2d[0], verts2d_opposite[2], verts2d_opposite[0] );
						info_array[0][0] = &info00;
						info_array[0][1] = &info01;
						info_array[0][2] = &info02;

						info_array[1][0] = &info10;
						info_array[1][1] = &info11;
						info_array[1][2] = &info12;

						info_array[2][0] = &info20;
						info_array[2][1] = &info21;
						info_array[2][2] = &info22;

						//int sum_intersection_class = info00.iclass + info01.iclass + info02.iclass + info10.iclass + info11.iclass + info12.iclass + info20.iclass + info21.iclass + info22.iclass;

						int line_intersection_count = 0;
						int point_intersection_count = 0;
						for( int i_info=0; i_info < 3; ++i_info )
						{
							int line_intersection_per_line_count = 0;
							int point_intersection_per_line_count = 0;
							for( int j_info=0; j_info < 3; ++j_info )
							{
								carve::geom2d::LineIntersectionInfo* info = info_array[i_info][j_info];
								if( info->iclass == carve::INTERSECTION_LL )
								{
									intersection_result_points.push_back( info->ipoint );
									++line_intersection_per_line_count;
								}
								else if( info->iclass == carve::INTERSECTION_PP )
								{
									++point_intersection_per_line_count;
								}

							}
							if( line_intersection_per_line_count > 0 )
							{
								++line_intersection_count;
							}
							if( point_intersection_per_line_count > 0 )
							{
								++point_intersection_count;
							}
						}

						if( point_intersection_count >= 3 )
						{
							omit_face = true;
						}

					}
					catch( carve::exception&e )
					{
						std::cout << e.str().c_str() << std::endl;
						continue;
					}
					*/
				}
			}
		}


		// handle the rest of the triangles
		for( std::set<carve::mesh::Face<3>* >::iterator it = set_unhandled_faces.begin(); it != set_unhandled_faces.end(); ++it )
		{
			carve::mesh::Face<3>* face = *it;

			std::vector<carve::mesh::Vertex<3>* > face_vertices;
			face->getVertices( face_vertices );

			//set_faces.erase( face );

			if( face_vertices.size() != 3 )
			{
				std::cout << "not a triangle" << std::endl;
				continue;
			}

			carve::mesh::Vertex<3>* face_vertex0 = face_vertices[0];
			carve::mesh::Vertex<3>* face_vertex1 = face_vertices[1];
			carve::mesh::Vertex<3>* face_vertex2 = face_vertices[2];

			int vertex_index0 = poly_cache.addPoint( face_vertex0->v );
			int vertex_index1 = poly_cache.addPoint( face_vertex1->v );
			int vertex_index2 = poly_cache.addPoint( face_vertex2->v );
			poly_cache.m_poly_data->addFace( vertex_index0, vertex_index1, vertex_index2 );
		}
	}


						//std::vector<carve::geom2d::LineIntersectionInfo> vec_info;
						//vec_info.push_back( carve::geom2d::lineSegmentIntersection( verts2d[0], verts2d[1], verts2d_check[0], verts2d_check[1] ) );
						//vec_info.push_back( carve::geom2d::lineSegmentIntersection( verts2d[0], verts2d[1], verts2d_check[1], verts2d_check[2] ) );
						//vec_info.push_back( carve::geom2d::lineSegmentIntersection( verts2d[0], verts2d[1], verts2d_check[2], verts2d_check[0] ) );

						//vec_info.push_back( carve::geom2d::lineSegmentIntersection( verts2d[1], verts2d[2], verts2d_check[0], verts2d_check[1] ) );
						//vec_info.push_back( carve::geom2d::lineSegmentIntersection( verts2d[1], verts2d[2], verts2d_check[1], verts2d_check[2] ) );
						//vec_info.push_back( carve::geom2d::lineSegmentIntersection( verts2d[1], verts2d[2], verts2d_check[2], verts2d_check[0] ) );

						//vec_info.push_back( carve::geom2d::lineSegmentIntersection( verts2d[2], verts2d[0], verts2d_check[0], verts2d_check[1] ) );
						//vec_info.push_back( carve::geom2d::lineSegmentIntersection( verts2d[2], verts2d[0], verts2d_check[1], verts2d_check[2] ) );
						//vec_info.push_back( carve::geom2d::lineSegmentIntersection( verts2d[2], verts2d[0], verts2d_check[2], verts2d_check[0] ) );

						//for( int i_info=0; i_info < vec_info.size(); ++i_info )
						//{
						//	carve::geom2d::LineIntersectionInfo& info = vec_info[i_info];
						//	if( info.iclass == carve::INTERSECTION_LL )
						//	{

						//	}
						//}
						

						
						//if( sum_intersection_class > 0 )
						//{
						//	// 
						//	if( info00.iclass > 0 )
						//	{
						//		intersection_result_p0 = info00.ipoint;

						//		if( info01.iclass <= 0 && info02.iclass <= 0 )
						//		{
						//			if( info10.iclass > 0 )
						//			{
						//				intersection_result_p1 = info00.p2;
						//				
						//				intersection_result_p1 = info00.p2;
						//			}
						//		}
						//	}

						//}

						//std::vector<carve::geom::vector<3>* > intersected_points( 3, nullptr );
						//int num_intersected = 0;
						//for( int i_triangle = 0; i_triangle < 3; ++i_triangle )
						//{
						//	carve::mesh::Vertex<3>* face_vertex = face_vertices[i_triangle];

						//	// project one triangle to the other
						//	carve::geom::linesegment<3> line( face_vertex->v, face_vertex->v+face_normal );
						//	//carve::geom::vector<3> intersect_point1;
						//	//bool intersected1;
						//	//bool intersected1 = face_check->simpleLineSegmentIntersection( line, intersect_point1 );

						//	//intersected_points[i_triangle] = nullptr;

						//	carve::mesh::MeshSet<3>::vertex_t::vector_t p;
						//	//carve::IntersectionClass intersects =	carve::geom3d::lineSegmentPlaneIntersection(face_check_plane, line, p);
						//	//                                          const Plane &p, const Vector &v1, const Vector &v2, Vector &v, double &t
						//	double t;
						//	carve::IntersectionClass intersects = carve::geom3d::rayPlaneIntersection(face_check_plane, face_vertex->v, face_vertex->v+face_normal, p, t );
						//	
						//	if (intersects == carve::INTERSECT_NONE || intersects == carve::INTERSECT_BAD)
						//	{
						//		//intersected_points[i_triangle] = nullptr;
						//		//intersected1 = false;
						//	}
						//	else
						//	{

						//		//std::vector<carve::geom::vector<2> > verts;
						//		//getProjectedVertices(verts);
						//		if (carve::geom2d::pointInPolySimple(verts2d_check, face_check->project(p)))
						//		{
						//			intersected_points[i_triangle] = &(p);
						//			//intersect_point1 = p;
						//			++num_intersected;
						//			//intersected1 = true;
						//		}
						//		//intersected1 = false;
						//	}
						//}

						//if( num_intersected == 3 )
						//{
						//	omit_face = true;
						//	map_face_subtract.insert( std::make_pair( face, std::vector<carve::geom::vector<3> >() ) );
						//	//std::map<carve::mesh::Face<3>*, std::vector<carve::geom::vector<3> > > map_face_subtract;

						//	//triangleinter
						//}
						//else if( num_intersected > 0 )
						//{
						//	//triangle
						//}
						//else
						//{
						//	// no intersection. could be disjoint, or enclosing the other triangle completely. this will be handled by other triangle
						//}


						//carve::geom::TriangleInt tr_interserction = carve::geom::triangle_intersection( 

						//if( face->nVertices() == 3 )
						//{
						//	std::vector<carve::mesh::Vertex<3> *> face_vertices_check;
						//	face_check->getVertices( face_vertices_check );

						//	if( face_vertices.size() == face_vertices_check.size() )
						//	{
						//		if( (face_vertex0->v - face_vertices_check[0]->v).length2() < 0.001 )
						//		{
						//			if( (face_vertex1->v - face_vertices_check[1]->v).length2() < 0.001 )
						//			{
						//				if( (face_vertex2->v - face_vertices_check[2]->v).length2() < 0.001 )
						//				{
						//					// omit face
						//					omit_face = true;
						//				}

						//			}
						//		}
						//	}
						//}




						//// check if one face is included in the other
						//
						//bool all_in = true;
						//bool all_out = true;
						//int num_in = 0;
						//int num_out = 0;
						//int num_border = 0;

						//for( int i_vert = 0; i_vert < verts2d_check.size(); ++i_vert )
						//{
						//	carve::geom::vector<2>& vert_check = verts2d_check[i_vert];
						//	carve::geom2d::PolyInclusionInfo pi = carve::geom2d::pointInPoly( verts2d, vert_check );
						//	switch(pi.iclass)
						//	{
						//	case carve::POINT_VERTEX:
						//		++num_border;
						//		break;
						//	case carve::POINT_EDGE:
						//		++num_border;
						//		break;
						//	case carve::POINT_ON:
						//		++num_border;
						//		break;
						//	case carve::POINT_IN:
						//		++num_in;
						//		break;
						//	case carve::POINT_OUT:
						//		++num_out;
						//		break;
						//	default:
						//		++num_out;
						//		break;
						//	}
						//}

						//if( num_in > 0 )
						//{
						//	if( num_out == 0 )
						//	{

						//	}
						//}

						//if( num_out == 0 ) // num_in + num_border == verts2d_check.size()
						//{
						//	// face is completely enclosed by other face
						//	// remove inner face, re-triangulate outer face

						//	// check if faces are disjoint, or one within the ohter
						//	bool disjoint = true;
						//	int num_on = 0;
						//	for( int i_vert = 0; i_vert < verts2d.size(); ++i_vert )
						//	{
						//		carve::geom::vector<2>& vert = verts2d[i_vert];
						//		carve::geom2d::PolyInclusionInfo pi = carve::geom2d::pointInPoly( verts2d_check, vert );
						//		switch(pi.iclass)
						//		{
						//		case carve::POINT_VERTEX:
						//			++num_on;
						//			break;
						//		case carve::POINT_EDGE:
						//			++num_on;
						//			break;
						//		case carve::POINT_ON:
						//			++num_on;
						//			break;
						//		case carve::POINT_IN:
						//			//++num_in;
						//			break;
						//		case carve::POINT_OUT:
						//			disjoint = false;
						//			break;
						//		default:
						//			//++num_out;
						//			break;
						//		}
						//	}

						//	//map_additional_face_openings.insert( 
						//}
			//		}
			//	}
			//}

			

			// check winding order
			//carve::geom3d::Vector normal_2d = GeomUtils::computePolygon2DNormal( verts2d );
			//if( normal_2d.z < 0 )
			//{
			//	std::reverse( verts2d.begin(), verts2d.end() );
			//}

			//std::vector<carve::triangulate::tri_idx> triangulated;
			//if( verts2d.size() > 3 )
			//{
			//	try
			//	{
			//		carve::triangulate::triangulate(verts2d, triangulated);
			//		carve::triangulate::improve(verts2d, triangulated);
			//	}
			//	catch(...)
			//	{
			//		std::cout << __FUNC__ << " carve::triangulate::incorporateHolesIntoPolygon failed " << std::endl;
			//		continue;
			//	}
			//}
			//else
			//{
			//	triangulated.push_back( carve::triangulate::tri_idx( 0, 1, 2 ) );
			//}

			//if( omit_face )
			//{
			//	continue;
			//}



			// now insert points to polygon, avoiding points with same coordinates
			//int i_vert = 0;
			//carve::mesh::Edge<3>* edge = face->edge;
			//do
			//{
			//	const carve::geom::vector<3>& v = edge->vert->v;
			//	edge = edge->next;
			//	int vertex_index = poly_cache.addPoint( v );
			//	map_merged_idx[i_vert] = vertex_index;
			//	++i_vert;
			//} while( edge != face->edge );




	meshset = nullptr;
	meshset = shared_ptr<carve::mesh::MeshSet<3> >( poly_cache.m_poly_data->createMesh(carve::input::opts()) );

	double volume_check2 = 0;
	for( size_t i = 0; i < meshset->meshes.size(); ++i )
	{
		carve::mesh::Mesh<3>* mesh = meshset->meshes[i];
		volume_check2 += mesh->volume();
	}

	if( abs(volume_check - volume_check2) > 0.0001 )
	{
		std::cout << __FUNC__ << " volume check failed." << std::endl;
	}

	int num_vertices2 = meshset->vertex_storage.size();
	if( num_vertices1 != num_vertices2 )
	{
		std::cout << __FUNC__ << " num vertices check failed." << std::endl;
	}

	{
		#ifdef IFCPP_OPENMP
		ScopedLock lock( writelock_count );
		#endif
		if( intersection_test_count > count_begin )
		{
			std::cout << "intersection_test_count: " << intersection_test_count << std::endl;
		}
	}
}


void retriangulateMeshSet( shared_ptr<carve::mesh::MeshSet<3> >& meshset )
{
	if( !meshset )
	{
		return;
	}

	PolyInputCache3D poly_cache;
	double volume_check = 0;

	for( size_t ii = 0; ii < meshset->meshes.size(); ++ii )
	{
		carve::mesh::Mesh<3>* mesh = meshset->meshes[ii];
		volume_check += mesh->volume();
		const std::vector<carve::mesh::Face<3>* >& vec_faces = mesh->faces;

		for( size_t i2 = 0; i2 < vec_faces.size(); ++i2 )
		{
			carve::mesh::Face<3>* face = vec_faces[i2];

			std::vector<carve::mesh::Vertex<3>* > face_vertices;
			face->getVertices( face_vertices );

			if( face_vertices.size() != 3 )
			{
				std::cout << "not a triangle" << std::endl;
				continue;
			}

			int vertex_index0 = poly_cache.addPoint( face_vertices[0]->v );
			int vertex_index1 = poly_cache.addPoint( face_vertices[1]->v );
			int vertex_index2 = poly_cache.addPoint( face_vertices[2]->v );

			if( vertex_index0 == vertex_index1 || vertex_index0 == vertex_index2 || vertex_index1 == vertex_index2 )
			{
				//++num_skipped_triangles;
				continue;
			}

			poly_cache.m_poly_data->addFace( vertex_index0, vertex_index1, vertex_index2 );
		}
	}

	meshset = nullptr;
	meshset = shared_ptr<carve::mesh::MeshSet<3> >( poly_cache.m_poly_data->createMesh(carve::input::opts()) );

	double volume_check2 = 0;
	for( size_t i = 0; i < meshset->meshes.size(); ++i )
	{
		carve::mesh::Mesh<3>* mesh = meshset->meshes[i];
		volume_check2 += mesh->volume();
	}

	if( abs(volume_check - volume_check2) > 0.001 )
	{
		std::cout << __FUNC__ << " volume check failed, vol1: " << volume_check << ", vol2: " << volume_check2 << std::endl;
	}
}


void CSG_Adapter::simplifyMesh( shared_ptr<carve::mesh::MeshSet<3> >& meshset )
{
	//carve::mesh::MeshSimplifier simplifier;
	//double min_colinearity = m_geom_settings->m_min_colinearity;
	//double min_delta_v = m_geom_settings->m_min_delta_v;
	//double min_normal_angle = m_geom_settings->m_min_normal_angle;
	//double min_length = 0.0001;//m_geom_settings->m_min_length;

	//try
	//{
	//	simplifier.removeFins(meshset.get());
	//	//simplifier.cleanFaceEdges( meshset.get() );
	//	//simplifier.removeRemnantFaces( meshset.get() );
	//	//simplifier.mergeCoplanarFaces( meshset.get(), 0.0 );
	//	//simplifier.eliminateShortEdges( meshset.get(), min_length );
	//	//simplifier.removeFins(meshset.get());
	//	simplifier.simplify( meshset.get(), min_colinearity, min_delta_v, min_normal_angle, min_length );
	//	simplifier.removeFins(meshset.get());
	//	//simplifier.removeLowVolumeManifolds(meshset, 0.01);
	//	simplifier.improveMesh( meshset.get(), m_geom_settings->m_min_colinearity, m_geom_settings->m_min_delta_v, m_geom_settings->m_min_normal_angle );
	//}
	//catch(...)
	//{
	//	std::cout << "simplifier.eliminateShortEdges failed." << std::endl;
	//}

	//carve::csg::CarveTriangulatorWithImprovement triang;
	//for( size_t ii = 0; ii < meshset->meshes.size(); ++ii )
	//{
	//	carve::mesh::Mesh<3>* mesh = meshset->meshes[ii];
	//	std::vector<carve::mesh::Face<3>* >& vec_faces = mesh->faces;
	//	carve::mesh::Face<3>* face_orig = nullptr;
	//	triang.processOutputFace( vec_faces, face_orig, false );
	//}
	
	if( !meshset )
	{
		return;
	}

	bool rebuild = true;
	int num_vertices = meshset->vertex_storage.size();
	if( rebuild && num_vertices > 8 )
	{
		retriangulateMeshSet( meshset );
#ifdef _DEBUG
		std::stringstream err;
		bool result_meshset_ok = ConverterOSG::checkMeshSet( meshset.get(), err, -1 );
		if( !result_meshset_ok )
		{
			std::cout << err.str().c_str() << std::endl;
		}
#endif
	
		//simplifier.removeFins(meshset.get());
		//removeThinCSGRemainings( meshset );
	}
}

void applyPosition( carve::mesh::MeshSet<3>* meshset, const carve::math::Matrix& pos )
{
	for (size_t i = 0; i < meshset->vertex_storage.size(); ++i )
	{
		carve::geom::vector<3>& point = meshset->vertex_storage[i].v;
		point = pos*point;
	}
	for (size_t i = 0; i < meshset->meshes.size(); ++i)
	{
		meshset->meshes[i]->recalc();
	}
}

double y_pos = 0;
bool CSG_Adapter::computeCSG( carve::mesh::MeshSet<3>* op1, carve::mesh::MeshSet<3>* op2, const carve::csg::CSG::OP operation, 
									 const int entity1, const int entity2, std::stringstream& err, shared_ptr<carve::mesh::MeshSet<3> >& result )
{
	bool csg_operation_ok = false;
	try
	{
		bool meshset1_ok = ConverterOSG::checkMeshSet( op1, err, entity1 );
		bool meshset2_ok = ConverterOSG::checkMeshSet( op2, err, entity2 );

		// check if meshset aabb is far away from origin. if so, move to origin, compute, move back
		carve::geom::vector<3> translate_avoid_large_numbers;
		const carve::geom::aabb<3>& aabb_op1 = op1->getAABB();
		if( aabb_op1.pos.length2() > 10000 )
		{
			const carve::geom::aabb<3>& aabb_op2 = op2->getAABB();

			if( aabb_op2.pos.length2() > 10000 )
			{
				carve::geom::vector<3> aabb_op1_direction( aabb_op1.pos );
				aabb_op1_direction.normalize();

				carve::geom::vector<3> aabb_op2_direction( aabb_op2.pos );
				aabb_op2_direction.normalize();

				double cos_angle = dot( aabb_op1_direction, aabb_op2_direction );
				if( cos_angle > -0.5 )
				{
					// if close to -1, the bboxes are in opposite direction, not useful to translate
					// if close to 1, the bboxes are somewhere in the same direction, good to translate

					// check extent
					if( aabb_op1.extent.length2() < 1000 && aabb_op2.extent.length2() < 1000 )
					{
						if( aabb_op1.pos.length2() > aabb_op2.pos.length2() )
						{
							// TODO: take biggest |x|, biggest |y|, biggest |z|
							translate_avoid_large_numbers = carve::geom::VECTOR( aabb_op1.pos.x, aabb_op1.pos.y, aabb_op1.pos.z );
						}
						else
						{
							translate_avoid_large_numbers = carve::geom::VECTOR( aabb_op2.pos.x, aabb_op2.pos.y, aabb_op2.pos.z );
						}
					}
				}
			}
		}


		if( translate_avoid_large_numbers.length2() > 1000.0 )
		{
			carve::math::Matrix mat_trans = carve::math::Matrix::TRANS( -translate_avoid_large_numbers );
			applyPosition( op1, mat_trans );
			applyPosition( op2, mat_trans );
		}

		if( meshset1_ok && meshset2_ok )
		{
			carve::csg::CSG csg;
			csg.hooks.registerHook(new carve::csg::CarveTriangulator(), carve::csg::CSG::Hooks::PROCESS_OUTPUT_FACE_BIT);
			//csg.hooks.registerHook(new carve::csg::CarveTriangulatorWithImprovement(), carve::csg::CSG::Hooks::PROCESS_OUTPUT_FACE_BIT);
			//csg.hooks.registerHook(new carve::csg::CarveHoleResolver(), carve::csg::CSG::Hooks::PROCESS_OUTPUT_FACE_BIT);
			
			//carve::csg::CSG::CLASSIFY_TYPE m_classify_type;
			result = shared_ptr<carve::mesh::MeshSet<3> >( csg.compute( op1, op2, operation, nullptr, carve::csg::CSG::CLASSIFY_EDGE ) );
			bool result_meshset_ok = ConverterOSG::checkMeshSet( result.get(), err, -1 );
			if( result_meshset_ok )
			{
				csg_operation_ok = true;
			}
			else
			{
#ifdef _DEBUG
				std::cout << "csg.compute result nok ok." << std::endl;
#endif
			}
		}

		if( translate_avoid_large_numbers.length2() > 1000.0 )
		{
			carve::math::Matrix mat_trans = carve::math::Matrix::TRANS( translate_avoid_large_numbers );
			applyPosition( result.get(), mat_trans );
			applyPosition( op1, mat_trans );
			applyPosition( op2, mat_trans );
		}
	}
	catch( carve::exception& ce )
	{
		csg_operation_ok = false;
		err << "csg operation failed, id1=" << entity1 << ", id2=" << entity2 << ", ";
		err << ce.str() << std::endl;
	}
	catch (const std::out_of_range& oor)
	{
		csg_operation_ok = false;
		err << "csg operation failed, id1=" << entity1 << ", id2=" << entity2 << ", ";
		err << oor.what() << std::endl;
	}
	catch(std::exception& e)
	{
		csg_operation_ok = false;
		err << "csg operation failed, id1=" << entity1 << ", id2=" << entity2 << ", ";
		err << e.what() << std::endl;
	}
	catch(...)
	{
		csg_operation_ok = false;
		err << "csg operation failed, id1=" << entity1 << ", id2=" << entity2 << std::endl;
	}

#ifdef _DEBUG
	if( !csg_operation_ok )
	{
		std::cout << "!csg_operation_ok. id1=" << entity1 << ", id2=" << entity2 << std::endl;

		if( result )
		{
			carve::mesh::MeshSet<3>* result_copy = result->clone();//new carve::mesh::MeshSet<3>();
			applyPosition( result_copy, carve::math::Matrix( carve::math::Matrix::TRANS( 0, y_pos, 0 ) ) );
			renderMeshsetInDebugViewer( result_copy, osg::Vec4(0.0f, 0.5f, 0.0f, 1.0f), false );
		}

		carve::mesh::MeshSet<3>* op1_copy = op1->clone();
		applyPosition( op1_copy, carve::math::Matrix( carve::math::Matrix::TRANS( 0, y_pos, 0 ) ) );
		renderMeshsetInDebugViewer( op1_copy, osg::Vec4(0.0f, 0.8f, 0.0f, 1.0f), true );

		carve::mesh::MeshSet<3>* op2_copy = op2->clone();
		applyPosition( op2_copy, carve::math::Matrix( carve::math::Matrix::TRANS( 0, y_pos, 0 ) ) );
		renderMeshsetInDebugViewer( op2_copy, osg::Vec4(0.8f, 0.0f, 0.0f, 1.0f), true );

		y_pos += 1;

		dumpMeshsets( op1, op2, result.get(), entity1, entity2 );
	}
#endif
	return csg_operation_ok;
}
