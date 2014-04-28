#include "GeometryInputData.h"

ItemData::ItemData()
{
	m_csg_computed = false;
}
ItemData::~ItemData()
{
}

void ItemData::createMeshSetsFromClosedPolyhedrons()
{
	for( unsigned int i=0; i<closed_polyhedrons.size(); ++i )
	{
		shared_ptr<carve::input::PolyhedronData>& poly_data = closed_polyhedrons[i];
		if( poly_data->getVertexCount() < 3 )
		{
			continue;
		}

		shared_ptr<carve::mesh::MeshSet<3> > meshset( poly_data->createMesh(carve::input::opts()) );
		//if( meshset->isClosed() )
		{
			meshsets.push_back( meshset );
		}
		//else
		//{
		//	item_meshsets_open.push_back( meshset );
		//}
	}
	closed_polyhedrons.clear();
}

void ItemData::addItemData( shared_ptr<ItemData>& other )
{
	std::copy( other->closed_polyhedrons.begin(), other->closed_polyhedrons.end(), std::back_inserter( closed_polyhedrons ) );
	std::copy( other->open_polyhedrons.begin(), other->open_polyhedrons.end(), std::back_inserter( open_polyhedrons ) );
	std::copy( other->open_or_closed_polyhedrons.begin(), other->open_or_closed_polyhedrons.end(), std::back_inserter( open_or_closed_polyhedrons ) );
	std::copy( other->polylines.begin(), other->polylines.end(), std::back_inserter( polylines ) );
	std::copy( other->meshsets.begin(), other->meshsets.end(), std::back_inserter( meshsets ) );
	std::copy( other->statesets.begin(), other->statesets.end(), std::back_inserter( statesets ) );
	m_csg_computed = other->m_csg_computed || m_csg_computed;
}

bool isIdentity( const carve::math::Matrix& mat )
{
	if( abs(mat._11 -1.0) > 0.00001 )  return false;
	if( abs(mat._22 -1.0) > 0.00001 )  return false;
	if( abs(mat._33 -1.0) > 0.00001 )  return false;
	if( abs(mat._44 -1.0) > 0.00001 )  return false;
	
	if( abs(mat._12) > 0.00001 )  return false;
	if( abs(mat._13) > 0.00001 )  return false;
	if( abs(mat._14) > 0.00001 )  return false;
	
	if( abs(mat._21) > 0.00001 )  return false;
	if( abs(mat._23) > 0.00001 )  return false;
	if( abs(mat._24) > 0.00001 )  return false;
	
	if( abs(mat._31) > 0.00001 )  return false;
	if( abs(mat._32) > 0.00001 )  return false;
	if( abs(mat._34) > 0.00001 )  return false;

	if( abs(mat._41) > 0.00001 )  return false;
	if( abs(mat._42) > 0.00001 )  return false;
	if( abs(mat._43) > 0.00001 )  return false;
	return true;
}



void ItemData::applyPosition( const carve::math::Matrix& mat )
{
	if( isIdentity( mat ) )
	{
		//return;
	}
	for( size_t i=0; i<open_polyhedrons.size(); ++i )
	{
		shared_ptr<carve::input::PolyhedronData>& shell_data = open_polyhedrons[i];
		for( size_t j=0; j<shell_data->points.size(); ++j )
		{
			carve::geom::vector<3>& point = shell_data->points[j];
			point = mat*point;
		}
	}

	for( size_t i=0; i<closed_polyhedrons.size(); ++i )
	{
		shared_ptr<carve::input::PolyhedronData>& shell_data = closed_polyhedrons[i];
		for( size_t j=0; j<shell_data->points.size(); ++j )
		{
			carve::geom::vector<3>& point = shell_data->points[j];
			point = mat*point;
		}
	}

	for( size_t i_poly=0; i_poly<open_or_closed_polyhedrons.size(); ++i_poly )
	{
		shared_ptr<carve::input::PolyhedronData>& shell_data = open_or_closed_polyhedrons[i_poly];

		for( size_t j=0; j<shell_data->points.size(); ++j )
		{
			carve::geom::vector<3>& point = shell_data->points[j];
			point = mat*point;
		}
	}

	for( std::vector<shared_ptr<carve::mesh::MeshSet<3> > >::iterator it_meshsets = meshsets.begin(); it_meshsets != meshsets.end(); ++it_meshsets )
	{
		shared_ptr<carve::mesh::MeshSet<3> >& item_meshset = (*it_meshsets);
		//item_meshset->transform( mat );

		for (size_t i = 0; i < item_meshset->vertex_storage.size(); ++i )
		{
			carve::geom::vector<3>& point = item_meshset->vertex_storage[i].v;
			point = mat*point;
		}
		for (size_t i = 0; i < item_meshset->meshes.size(); ++i)
		{
          item_meshset->meshes[i]->recalc();
        }
	}

	for( int polyline_i = 0; polyline_i < polylines.size(); ++polyline_i )
	{
		shared_ptr<carve::input::PolylineSetData>& polyline_data = polylines.at(polyline_i);
		for( size_t j=0; j<polyline_data->points.size(); ++j )
		{
			carve::geom::vector<3>& point = polyline_data->points[j];
			point = mat*point;
		}
	}
}

shared_ptr<ItemData> ItemData::getDeepCopy()
{
	shared_ptr<ItemData> copy_item( new ItemData() );

	for( size_t i=0; i<open_polyhedrons.size(); ++i )
	{
		shared_ptr<carve::input::PolyhedronData>& shell_data_ptr = open_polyhedrons[i];
		carve::input::PolyhedronData& shell_data = *(shell_data_ptr.get());
		copy_item->open_polyhedrons.push_back( shared_ptr<carve::input::PolyhedronData>( new carve::input::PolyhedronData( shell_data ) ) );
	}

	for( size_t i=0; i<closed_polyhedrons.size(); ++i )
	{
		shared_ptr<carve::input::PolyhedronData>& shell_data = closed_polyhedrons[i];
		copy_item->closed_polyhedrons.push_back( shared_ptr<carve::input::PolyhedronData>( new carve::input::PolyhedronData( *(shell_data.get()) ) ) );
	}

	for( size_t i_poly=0; i_poly<open_or_closed_polyhedrons.size(); ++i_poly )
	{
		shared_ptr<carve::input::PolyhedronData>& shell_data = open_or_closed_polyhedrons[i_poly];
		copy_item->open_or_closed_polyhedrons.push_back( shared_ptr<carve::input::PolyhedronData>( new carve::input::PolyhedronData( *(shell_data.get()) ) ) );
	}

	for( std::vector<shared_ptr<carve::mesh::MeshSet<3> > >::iterator it_meshsets = meshsets.begin(); it_meshsets != meshsets.end(); ++it_meshsets )
	{
		shared_ptr<carve::mesh::MeshSet<3> >& item_meshset = (*it_meshsets);
		copy_item->meshsets.push_back( shared_ptr<carve::mesh::MeshSet<3> >( item_meshset->clone() ) );
	}

	for( int polyline_i = 0; polyline_i < polylines.size(); ++polyline_i )
	{
		shared_ptr<carve::input::PolylineSetData>& polyline_data = polylines.at(polyline_i);
		copy_item->polylines.push_back( shared_ptr<carve::input::PolylineSetData>( new carve::input::PolylineSetData( *(polyline_data.get()) ) ) );
	}

	std::copy( statesets.begin(), statesets.end(), std::back_inserter( copy_item->statesets ) );

	return copy_item;
}

void ShapeInputData::addInputData( shared_ptr<ShapeInputData>& other )
{
	std::copy( other->vec_item_data.begin(), other->vec_item_data.end(), std::back_inserter( vec_item_data ) );
	std::copy( other->vec_statesets.begin(), other->vec_statesets.end(), std::back_inserter( vec_statesets ) );
}

void ShapeInputData::deepCopyFrom( shared_ptr<ShapeInputData>& other )
{
	vec_item_data.clear();
	vec_statesets.clear();

	for( int item_i = 0; item_i < other->vec_item_data.size(); ++item_i )
	{
		shared_ptr<ItemData>& item_data = other->vec_item_data[item_i];
		vec_item_data.push_back( shared_ptr<ItemData>( item_data->getDeepCopy() ) );
	}
	std::copy( other->vec_statesets.begin(), other->vec_statesets.end(), std::back_inserter( vec_statesets ) );
	//addInputData( other );
}

// PolyInputCache
PolyInputCache3D::PolyInputCache3D()
{
	m_poly_data = shared_ptr<carve::input::PolyhedronData>( new carve::input::PolyhedronData() );
}

int PolyInputCache3D::addPoint( const carve::geom::vector<3>& v )
{
#ifdef ROUND_IFC_COORDINATES
	const double vertex_x = round(v.x*ROUND_IFC_COORDINATES_UP)*ROUND_IFC_COORDINATES_DOWN;
	const double vertex_y = round(v.y*ROUND_IFC_COORDINATES_UP)*ROUND_IFC_COORDINATES_DOWN;
	const double vertex_z = round(v.z*ROUND_IFC_COORDINATES_UP)*ROUND_IFC_COORDINATES_DOWN;
#else
	const double vertex_x = v.x;
	const double vertex_y = v.y;
	const double vertex_z = v.z;
#endif

	//  return a pair, with its member pair::first set to an iterator pointing to either the newly inserted element or to the element with an equivalent key in the map
	std::map<double, std::map<double, int> >& map_y_index = existing_vertices_coords.insert( std::make_pair(vertex_x, std::map<double, std::map<double, int> >() ) ).first->second;
	std::map<double, int>& map_z_index = map_y_index.insert( std::make_pair( vertex_y, std::map<double, int>() ) ).first->second;

	it_find_z = map_z_index.find( vertex_z );
	if( it_find_z != map_z_index.end() )
	{
		// vertex already exists in polyhedron. return its index
		int vertex_index = it_find_z->second;
		return vertex_index;
	}
	else
	{
		// add point to polyhedron
		int vertex_index = m_poly_data->addVertex( v );
		map_z_index[vertex_z] = vertex_index;
		return vertex_index;
	}
}
