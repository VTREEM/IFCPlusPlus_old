#include "GeometryInputData.h"

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

void ItemData::applyPosition( carve::math::Matrix& mat )
{
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

void ShapeInputData::addInputData( shared_ptr<ShapeInputData>& other )
{
	std::copy( other->vec_item_data.begin(), other->vec_item_data.end(), std::back_inserter( vec_item_data ) );
	std::copy( other->vec_statesets.begin(), other->vec_statesets.end(), std::back_inserter( vec_statesets ) );
}
