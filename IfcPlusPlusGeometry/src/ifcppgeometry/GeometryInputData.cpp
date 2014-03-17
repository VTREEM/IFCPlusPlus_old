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
