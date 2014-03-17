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

#include "DebugViewerCallback.h"

#ifdef  _DEBUG

#include <osgText/Text>
#include <ifcpp/model/UnitConverter.h>
#include <ifcpp/IFC4/include/IfcSphere.h>
#include <ifcpp/IFC4/include/IfcPositiveLengthMeasure.h>
#include <ifcpp/IFC4/include/IfcUShapeProfileDef.h>
#include <ifcpp/IFC4/include/IfcNonNegativeLengthMeasure.h>
#include <ifcpp/IFC4/include/IfcPlaneAngleMeasure.h>
#include <ifcpp/IFC4/include/IfcExtrudedAreaSolid.h>
#include <ifcpp/IFC4/include/IfcDirection.h>
#include <ifcpp/IFC4/include/IfcBeam.h>
#include <ifcpp/IFC4/include/IfcLocalPlacement.h>
#include <ifcpp/IFC4/include/IfcProductDefinitionShape.h>
#include <ifcpp/IFC4/include/IfcExtrudedAreaSolid.h>
#include <ifcpp/IFC4/include/IfcCircleHollowProfileDef.h>
#include <ifcpp/IFC4/include/IfcProfileTypeEnum.h>
#include <ifcpp/IFC4/include/IfcPositiveLengthMeasure.h>
#include <ifcpp/IFC4/include/IfcDirection.h>
#include <ifcpp/IFC4/include/IfcShapeRepresentation.h>
#include <ifcpp/IFC4/include/IfcRelVoidsElement.h>
#include <ifcpp/IFC4/include/IfcOpeningElement.h>
#include <ifcpp/IFC4/include/IfcCartesianPoint.h>
#include <ifcpp/IFC4/include/IfcLengthMeasure.h>
#include <ifcpp/IFC4/include/IfcPolyline.h>
#include <ifcpp/IFC4/include/IfcAxis2Placement3d.h>
#include <ifcpp/IFC4/include/IfcArbitraryClosedProfileDef.h>
#include <ifcppgeometry/ReaderWriterIFC.h>
#include <ifcppgeometry/ConverterOSG.h>
#include <ifcppgeometry/SolidModelConverter.h>
#include <ifcppgeometry/GeomUtils.h>


void* obj_call_on_render_polyhedron = NULL;
void* obj_call_on_render_meshset = NULL;
void* obj_call_on_render_polyline = NULL;
void* obj_call_on_render_paths = NULL;
void (*func_call_on_render_polyhedron)(void*, const carve::input::PolyhedronData* poly, const osg::Vec4f& color, const bool wireframe) = NULL;
void (*func_call_on_render_meshset)(void*, const carve::mesh::MeshSet<3>* meshset, const osg::Vec4f& color, const bool wireframe) = NULL;
void (*func_call_on_render_polyline)(void*, const carve::input::PolylineSetData* poly_line, const osg::Vec4f& color) = NULL;
void (*func_call_on_render_paths)(void*, const std::vector<std::vector<carve::geom::vector<2> > >& paths ) = NULL;

void setRenderPolyhedronCallBack( void* obj_ptr, void (*func)(void*, const carve::input::PolyhedronData* poly, const osg::Vec4f& color, const bool wireframe ) )
{
	obj_call_on_render_polyhedron = obj_ptr;
	func_call_on_render_polyhedron = func;
}

void setRenderMeshsetCallBack( void* obj_ptr, void (*func)(void*, const carve::mesh::MeshSet<3>* meshset, const osg::Vec4f& color, const bool wireframe ) )
{
	obj_call_on_render_meshset = obj_ptr;
	func_call_on_render_meshset = func;
}

void setRenderPolylineCallBack( void* obj_ptr, void (*func)(void*, const carve::input::PolylineSetData* poly_line, const osg::Vec4f& color ) )
{
	obj_call_on_render_polyline = obj_ptr;
	func_call_on_render_polyline = func;
}

void setRenderPathsCallBack( void* obj_ptr, void (*func)(void*, const std::vector<std::vector<carve::geom::vector<2> > >& paths ) )
{
	obj_call_on_render_paths = obj_ptr;
	func_call_on_render_paths = func;
}

void renderShapeInputDataInDebugViewer( const ShapeInputData* input_data, const osg::Vec4f& color, const bool wireframe )
{
	for( int i=0; i<input_data->vec_item_data.size(); ++i )
	{
		const shared_ptr<ItemData>&	item_data = input_data->vec_item_data[i];
		//item_data->createMeshSetsFromClosedPolyhedrons();

		std::vector<shared_ptr<carve::input::PolyhedronData> >& vec_poly_data = item_data->closed_polyhedrons;
		for( int j=0; j<vec_poly_data.size(); ++j )
		{
			shared_ptr<carve::input::PolyhedronData> poly_data = vec_poly_data[j];
			//shared_ptr<carve::mesh::MeshSet<3> > meshset( poly_data->createMesh(carve::input::opts()) );
			renderPolyhedronInDebugViewer( poly_data.get(), color, wireframe );
		}

		for( int j=0; j<item_data->meshsets.size(); ++j )
		{
			renderMeshsetInDebugViewer( item_data->meshsets[j].get(), color, wireframe );
		}
	}
}

void renderPolyhedronInDebugViewer( const carve::input::PolyhedronData* poly, const osg::Vec4f& color, const bool wireframe )
{
	if( func_call_on_render_polyhedron )
	{
		if( obj_call_on_render_polyhedron )
		{
			func_call_on_render_polyhedron( obj_call_on_render_polyhedron, poly, color, wireframe );
		}
	}
}

void renderMeshsetInDebugViewer( const carve::mesh::MeshSet<3>* meshset, const osg::Vec4f& color, const bool wireframe )
{
	if( func_call_on_render_meshset )
	{
		if( obj_call_on_render_meshset )
		{
			func_call_on_render_meshset( obj_call_on_render_meshset, meshset, color, wireframe );
		}
	}
}

void renderPolylineInDebugViewer( const carve::input::PolylineSetData* poly_line, const osg::Vec4f& color )
{
	if( func_call_on_render_polyline )
	{
		if( obj_call_on_render_meshset )
		{
			func_call_on_render_polyline( obj_call_on_render_meshset, poly_line, color );
		}
	}
}

void renderPathsInDebugViewer( const std::vector<std::vector<carve::geom::vector<2> > >& paths )
{
	if( func_call_on_render_paths )
	{
		if( obj_call_on_render_paths )
		{
			func_call_on_render_paths( obj_call_on_render_paths, paths );
		}
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void drawVertexNumbers( const carve::input::PolyhedronData* poly, const osg::Vec4f& color, osg::Geode* geode )
{
	if( poly )
	{
		double x_min = DBL_MAX;
		double x_max = -DBL_MAX;
		double y_min = DBL_MAX;
		double y_max = -DBL_MAX;

		const size_t num_vertices = poly->points.size();
		for( int i=0; i<num_vertices; ++i )
		{
			const carve::poly::Vertex<3>& vertex = poly->points[i];
			const carve::geom::vector<3>& point = vertex.v;

			x_min = std::min( x_min, point.x );
			x_max = std::max( x_max, point.x );
			y_min = std::min( y_min, point.y );
			y_max = std::max( y_max, point.y );

		}

		double max_extend = std::max( x_max-x_min, y_max-y_min );
		double min_extend = std::min( x_max-x_min, y_max-y_min );


		//double font_size = 0.5*(max_extend + min_extend)*0.1;
		double font_size = min_extend*0.01;
		for( int i=0; i<num_vertices; ++i )
		{
			const carve::poly::Vertex<3>& vertex = poly->points[i];
			const carve::geom::vector<3>& point = vertex.v;

			osgText::Text* txt = new osgText::Text;
			txt->setFont("fonts/arial.ttf");
			//txt->setColor( color );
			txt->setColor( osg::Vec4f( 0, 0, 0, 1 ) );
			txt->setCharacterSize( font_size );
			txt->setAutoRotateToScreen( true );
			txt->setPosition( osg::Vec3( point.x, point.y, point.z ) );
			txt->getOrCreateStateSet()->setMode( GL_LIGHTING, osg::StateAttribute::OFF );

			std::stringstream strs;
			strs << i;
			txt->setText( strs.str().c_str() );
			geode->addDrawable(txt);
		}
	}
}

void dumpMeshset( carve::mesh::MeshSet<3>* meshset, const int poly_num, std::stringstream& strs_out )
{
	strs_out << "carve::input::PolyhedronData* poly_data" << poly_num << " = new carve::input::PolyhedronData();" << std::endl;
	if( !meshset )
	{
		strs_out << "// meshset not valid" << std::endl;
		return;
	}
	if( meshset->meshes.size() < 1 )
	{
		strs_out << "// meshset->meshes.size() < 1" << std::endl;
		return;
	}

	//strs_out << "carve::input::PolyhedronData* poly_data" << poly_num << " = new carve::input::PolyhedronData();" << std::endl;
	carve::poly::Polyhedron * poly = carve::polyhedronFromMesh(meshset, -1);

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

void dumpMeshset( carve::mesh::MeshSet<3>* meshset, int poly_num, const int file_out_num )
{
	std::stringstream strs_out;
	strs_out << "carve::input::PolyhedronData* poly_data" << poly_num << " = new carve::input::PolyhedronData();" << std::endl;
	carve::poly::Polyhedron * poly = carve::polyhedronFromMesh(meshset, -1);

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
	std::stringstream file_name;
	file_name << "dump_mesh" << file_out_num << ".txt";
	std::ofstream ofs( file_name.str().c_str(), std::ofstream::out);
	ofs << strs_out.str().c_str();
	ofs.close();
}



int dump_count = 1;
void dumpMeshsets( carve::mesh::MeshSet<3>* first_operand_meshset, 
								carve::mesh::MeshSet<3>* second_operand_meshset, 
								carve::mesh::MeshSet<3>* result_meshset,
								int id1, int id2 )
{
	std::stringstream cpp_input;

	if( first_operand_meshset )
	{
		cpp_input << "// entity_id: " << id1 << std::endl;
		dumpMeshset( first_operand_meshset, 1, cpp_input );
	}
	
	if( second_operand_meshset )
	{
		cpp_input << "// entity_id: " << id2 << std::endl;
		dumpMeshset( second_operand_meshset, 2, cpp_input );
	}
	
	if( result_meshset )
	{
		cpp_input << "// result: " << std::endl;
		dumpMeshset( result_meshset, 3, cpp_input );
	}

	std::stringstream file_name;
	file_name << "dump_csg_failure" << dump_count << ".txt";
	std::ofstream ofs( file_name.str().c_str(), std::ofstream::out);
	ofs << cpp_input.str().c_str();
	ofs.close();

	++dump_count;
}

int vtk_dump_count = 1;
void dumpToVTK( const std::vector<std::vector<carve::geom::vector<2> > >& paths )
{
	int num_points = 0;
	std::vector<carve::geom::vector<2> > vec_merged;
	for( std::vector<std::vector<carve::geom::vector<2> > >::const_iterator it = paths.begin(); it!=paths.end(); ++it )
	{
		const std::vector<carve::geom::vector<2> >& vec_path = (*it);
		num_points += vec_path.size();
		
		for( std::vector<carve::geom::vector<2> >::const_iterator it2 = vec_path.begin(); it2!=vec_path.end(); ++it2 )
		{
			carve::geom::vector<2> point = (*it2);
			vec_merged.push_back( point );
		}
	}

//# vtk DataFile Version 3.0
//vtk output
//ASCII
//DATASET POLYDATA
//POINTS 4 float
//0 0 0
//1 0 0
//1.1 1.1 0
//0 1 0
//POLYGONS 1 5
//4 0 1 2 3
//CELL_DATA 1
//POINT_DATA 4
//SCALARS nodal float
//LOOKUP_TABLE default
//0 1 2 1.1

	std::stringstream out;
	out << "# vtk DataFile Version 3.0" << std::endl;
	out << "vtk output" << std::endl;
	out << "ASCII" << std::endl;
	out << "DATASET POLYDATA" << std::endl;
	out << "POINTS " << num_points << " float" << std::endl;

	int point_id = 0;
	//for( std::vector<std::vector<carve::geom::vector<2> > >::const_iterator it = paths.begin(); it!=paths.end(); ++it )
	{
		//const std::vector<carve::geom::vector<2> >& vec_path = (*it);

		for( std::vector<carve::geom::vector<2> >::iterator it2 = vec_merged.begin(); it2!=vec_merged.end(); ++it2 )
		{
			const carve::geom::vector<2>& point = (*it2);
			out << point_id << " " << point.x << " " << point.y << std::endl;
			++point_id;
		}
	}

	out << "LINES " << num_points << " " << num_points*3 << std::endl;
	//out << num_points+1 << " ";
	for( int i=0; i<vec_merged.size()-1; ++i )
	{
		out << "2 " << i << " "  << i+1 << std::endl;
	}
	out << std::endl;
	//out << "CELL_DATA 1" << std::endl;
	//out << "POINT_DATA " << num_points << std::endl;


	//out << "POLYGONS 1 " << num_points+1 << std::endl;
	//out << num_points+1 << " ";
	//for( int i=0; i<vec_merged.size(); ++i )
	//{
	//	out << i << " ";
	//}
	//out << std::endl;
	//out << "CELL_DATA 1" << std::endl;
	//out << "POINT_DATA " << num_points << std::endl;
	
	//out << "SCALARS nodal float" << std::endl;
	//out << "LOOKUP_TABLE default" << std::endl;
	//out << "0 1 2 1.1" << std::endl;

	std::stringstream file_name;
	file_name << "dump_paths" << vtk_dump_count << ".vtk";
	std::ofstream ofs( file_name.str().c_str(), std::ofstream::out);
	ofs << out.str().c_str();
	ofs.close();
	++vtk_dump_count;
}

shared_ptr<IfcCartesianPoint> createCartesianPoint( double x, double y )
{
	shared_ptr<IfcCartesianPoint> p0( new IfcCartesianPoint() );
	p0->m_Coordinates.push_back( shared_ptr<IfcLengthMeasure>( new IfcLengthMeasure() ) );
	p0->m_Coordinates.push_back( shared_ptr<IfcLengthMeasure>( new IfcLengthMeasure() ) );
	p0->m_Coordinates[0]->m_value = x;
	p0->m_Coordinates[1]->m_value = y;
	return p0;
}

shared_ptr<IfcCartesianPoint> createCartesianPoint( double x, double y, double z )
{
	shared_ptr<IfcCartesianPoint> p0( new IfcCartesianPoint() );
	p0->m_Coordinates.push_back( shared_ptr<IfcLengthMeasure>( new IfcLengthMeasure() ) );
	p0->m_Coordinates.push_back( shared_ptr<IfcLengthMeasure>( new IfcLengthMeasure() ) );
	p0->m_Coordinates.push_back( shared_ptr<IfcLengthMeasure>( new IfcLengthMeasure() ) );
	p0->m_Coordinates[0]->m_value = x;
	p0->m_Coordinates[1]->m_value = y;
	p0->m_Coordinates[2]->m_value = z;
	return p0;
}

void createTest( osg::Group* group, osg::Group* root )
{
	if( false )
	{
		//#149238= IFCCARTESIANPOINT((286.999999742634,14545.0960300668,6591.20787626065));
		shared_ptr<IfcCartesianPoint> _149238 = createCartesianPoint( 286.999999742634,14545.0960300668,6591.20787626065 );

		//#41466= IFCDIRECTION((1.81898940690073E-013,0.996194701818039,0.0871557001559201));
		shared_ptr<IfcDirection> _41466( new IfcDirection() );
		_41466->m_DirectionRatios.push_back( 1.81898940690073E-013 );
		_41466->m_DirectionRatios.push_back( 0.996194701818039 );
		_41466->m_DirectionRatios.push_back( 0.0871557001559201 );

		//#6= IFCCARTESIANPOINT((0.,0.,0.));
		shared_ptr<IfcCartesianPoint> _6 = createCartesianPoint( 0,0,0 );
		
		//#7= IFCDIRECTION((1.,0.,0.));
		shared_ptr<IfcDirection> _7( new IfcDirection() );
		_7->m_DirectionRatios.push_back( 1 );
		_7->m_DirectionRatios.push_back( 0 );
		_7->m_DirectionRatios.push_back( 0 );

		//#8= IFCDIRECTION((0.,1.,0.));
		shared_ptr<IfcDirection> _8( new IfcDirection() );
		_8->m_DirectionRatios.push_back( 0 );
		_8->m_DirectionRatios.push_back( 1 );
		_8->m_DirectionRatios.push_back( 0 );


		//#9= IFCDIRECTION((0.,0.,1.));
		shared_ptr<IfcDirection> _9( new IfcDirection() );
		_9->m_DirectionRatios.push_back( 0 );
		_9->m_DirectionRatios.push_back( 0 );
		_9->m_DirectionRatios.push_back( 1 );

		//#10= IFCAXIS2PLACEMENT3D(#6,#9,#7);
		shared_ptr<IfcAxis2Placement3D> _10( new IfcAxis2Placement3D() );
		_10->m_Location = _6;
		_10->m_Axis = _9;
		_10->m_RefDirection = _7;

		//#25= IFCLOCALPLACEMENT($,#10);
		shared_ptr<IfcLocalPlacement> _25( new IfcLocalPlacement() );
		//_25->m_PlacementRelTo = $;
		_25->m_RelativePlacement = _10;


		//#27= IFCLOCALPLACEMENT(#25,#10);
		shared_ptr<IfcLocalPlacement> _27( new IfcLocalPlacement() );
		_27->m_PlacementRelTo = _25;
		_27->m_RelativePlacement = _10;

		//#149239= IFCAXIS2PLACEMENT3D(#149238,#41466,#7);
		shared_ptr<IfcAxis2Placement3D> _149239( new IfcAxis2Placement3D() );
		_149239->m_Location = _149238;
		_149239->m_Axis = _41466;
		_149239->m_RefDirection = _7;

		//#29= IFCLOCALPLACEMENT(#27,#10);
		shared_ptr<IfcLocalPlacement> _29( new IfcLocalPlacement() );
		_29->m_PlacementRelTo = _27;
		_29->m_RelativePlacement = _10;

		//#149240= IFCLOCALPLACEMENT(#29,#149239);
		shared_ptr<IfcLocalPlacement> _149240( new IfcLocalPlacement() );
		_149240->m_PlacementRelTo = _29;
		_149240->m_RelativePlacement = _149239;

		//#107620= IFCCIRCLEHOLLOWPROFILEDEF(.AREA.,'RO114.3*3.6',#254,57.15,3.6);
		shared_ptr<IfcCircleHollowProfileDef> _107620( new IfcCircleHollowProfileDef() );
		_107620->setId( 107620 );
		_107620->m_Radius = shared_ptr<IfcPositiveLengthMeasure>( new IfcPositiveLengthMeasure() );
		_107620->m_Radius->m_value = 57.15;
		_107620->m_WallThickness = shared_ptr<IfcPositiveLengthMeasure>( new IfcPositiveLengthMeasure() );
		_107620->m_WallThickness->m_value = 3.6;

		//#244= IFCDIRECTION((0.,0.,-1.));
		shared_ptr<IfcDirection> _244( new IfcDirection() );
		_244->m_DirectionRatios.push_back( 0 );
		_244->m_DirectionRatios.push_back( 0 );
		_244->m_DirectionRatios.push_back( -1 );

		//#245= IFCDIRECTION((0.,-1.,0.));
		shared_ptr<IfcDirection> _245( new IfcDirection() );
		_245->m_DirectionRatios.push_back( 0 );
		_245->m_DirectionRatios.push_back( -1 );
		_245->m_DirectionRatios.push_back( 0 );

		//#256= IFCDIRECTION((-1.,0.,0.));
		shared_ptr<IfcDirection> _256( new IfcDirection() );
		_256->m_DirectionRatios.push_back( -1 );
		_256->m_DirectionRatios.push_back( 0 );
		_256->m_DirectionRatios.push_back( 0 );

		//#149007= IFCCARTESIANPOINT((7290.99999999655,1.38075712319602E-009,-5.15260594064864E-008));
		shared_ptr<IfcCartesianPoint> _149007 = createCartesianPoint( 7290.99999999655,1.38075712319602E-009,-5.15260594064864E-008 );

		//#149008= IFCAXIS2PLACEMENT3D(#149007,#256,#245);
		shared_ptr<IfcAxis2Placement3D> _149008( new IfcAxis2Placement3D() );
		_149008->m_Location = _149007;
		_149008->m_Axis = _256;
		_149008->m_RefDirection = _245;

		//#149241= IFCEXTRUDEDAREASOLID(#107620,#149008,#9,7290.99999999662);
		shared_ptr<IfcExtrudedAreaSolid> _149241( new IfcExtrudedAreaSolid() );
		_149241->setId( 149241 );
		_149241->m_SweptArea = _107620;
		_149241->m_Position = _149008;
		_149241->m_ExtrudedDirection = _9;
		_149241->m_Depth = shared_ptr<IfcPositiveLengthMeasure>( new IfcPositiveLengthMeasure() );
		_149241->m_Depth->m_value = 7290.99999999662;

		//#149242= IFCSHAPEREPRESENTATION(#12,'Body','SweptSolid',(#149241));
		shared_ptr<IfcShapeRepresentation> _149242( new IfcShapeRepresentation() );
		_149242->m_Items.push_back( _149241 );// std::vector<shared_ptr<IfcRepresentationItem> >	;

		//#149244= IFCPRODUCTDEFINITIONSHAPE($,$,(#149242));
		shared_ptr<IfcProductDefinitionShape> _149244( new IfcProductDefinitionShape() );
		_149244->m_Representations.push_back( _149242 );

		//#149245= IFCBEAM('1GXYHh000KCJ4pDJ4mC38t',#5,'Druckrohr','RO114.3*3.6','RO114.3*3.6',#149240,#149244,'ID5086246B-0000-1431-3133-353130303237');
		shared_ptr<IfcBeam> _149245( new IfcBeam() );
		_149245->m_Representation = _149244;
		_149245->m_ObjectPlacement = _149240;

		shared_ptr<IfcPolyline> _149040( new IfcPolyline() );
		_149040->setId( 149040 );
		_149040->m_Points.push_back( createCartesianPoint(0.,0.) ); 
		_149040->m_Points.push_back( createCartesianPoint(20.,6.00266503170133E-011) ); 
		_149040->m_Points.push_back( createCartesianPoint(28.,8.) ); 
		_149040->m_Points.push_back( createCartesianPoint(197.289993286133,8.) ); 
		_149040->m_Points.push_back( createCartesianPoint(198.505584716797,8.10574626922607) ); 
		_149040->m_Points.push_back( createCartesianPoint(199.684341430664,8.42096996307373) ); 
		_149040->m_Points.push_back( createCartesianPoint(200.790466308594,8.93609523773193) ); 
		_149040->m_Points.push_back( createCartesianPoint(201.790313720703,9.63546752929688) ); 
		_149040->m_Points.push_back( createCartesianPoint(202.653533935547,10.497838973999) ); 
		_149040->m_Points.push_back( createCartesianPoint(203.353897094727,11.4970064163208) ); 
		_149040->m_Points.push_back( createCartesianPoint(203.8701171875,12.6026096343994) ); 
		_149040->m_Points.push_back( createCartesianPoint(204.186492919922,13.7810564041138) ); 
		_149040->m_Points.push_back( createCartesianPoint(204.293441772461,14.9965400695801) ); 
		_149040->m_Points.push_back( createCartesianPoint(204.187698364258,16.2121295928955) ); 
		_149040->m_Points.push_back( createCartesianPoint(203.872482299805,17.3908863067627) ); 
		_149040->m_Points.push_back( createCartesianPoint(203.357345581055,18.4969997406006) ); 
		_149040->m_Points.push_back( createCartesianPoint(202.657974243164,19.4968566894531) ); 
		_149040->m_Points.push_back( createCartesianPoint(201.795608520508,20.3600788116455) );
		_149040->m_Points.push_back( createCartesianPoint(200.796432495117,21.0604381561279) );
		_149040->m_Points.push_back( createCartesianPoint(199.690841674805,21.5766544342041) );
		_149040->m_Points.push_back( createCartesianPoint(198.512390136719,21.8930397033691) );
		_149040->m_Points.push_back( createCartesianPoint(197.296905517578,21.9999847412109) );
		_149040->m_Points.push_back( createCartesianPoint(28.0069046020508,21.9999847412109) );
		_149040->m_Points.push_back( createCartesianPoint(20.0069046020508,29.9999847412109) );
		_149040->m_Points.push_back( createCartesianPoint(0.00690512266010046,29.9999847412109) );

		//#149041= IFCARBITRARYCLOSEDPROFILEDEF(.AREA.,'BL125*30',#149040);
		shared_ptr<IfcArbitraryClosedProfileDef> _149041( new IfcArbitraryClosedProfileDef() );
		_149041->setId( 149041 );
		_149041->m_OuterCurve = _149040;

		//#149042= IFCCARTESIANPOINT((7311.00273145771,-14.9999816531677,62.4999984072053));
		shared_ptr<IfcCartesianPoint> _149042 = createCartesianPoint( 7311.00273145771,-14.9999816531677,62.4999984072053 );

		//#149043= IFCAXIS2PLACEMENT3D(#149042,#244,#256);
		shared_ptr<IfcAxis2Placement3D> _149043( new IfcAxis2Placement3D() );
		_149043->m_Location = _149042;
		_149043->m_Axis = _244;
		_149043->m_RefDirection = _256;

		//#149044= IFCEXTRUDEDAREASOLID(#149041,#149043,#9,125.);
		shared_ptr<IfcExtrudedAreaSolid> _149044( new IfcExtrudedAreaSolid() );
		_149044->m_SweptArea = _149041;
		_149044->m_Position = _149043;
		_149044->m_ExtrudedDirection = _9;
		_149044->m_Depth = shared_ptr<IfcPositiveLengthMeasure>( new IfcPositiveLengthMeasure() );
		_149044->m_Depth->m_value = 125.;

		//#149248= IFCSHAPEREPRESENTATION(#12,'Body','SweptSolid',(#149044));
		shared_ptr<IfcShapeRepresentation> _149248( new IfcShapeRepresentation() );
		_149248->m_Items.push_back( _149044 );// std::vector<shared_ptr<IfcRepresentationItem> >	;

		//#149249= IFCPRODUCTDEFINITIONSHAPE($,$,(#149248));
		shared_ptr<IfcProductDefinitionShape> _149249( new IfcProductDefinitionShape() );
		_149249->m_Representations.push_back( _149248 );


		//#149246= IFCAXIS2PLACEMENT3D(#149238,#41466,#7);
		shared_ptr<IfcAxis2Placement3D> _149246( new IfcAxis2Placement3D() );
		_149246->m_Location = _149238;
		_149246->m_Axis = _41466;
		_149246->m_RefDirection = _7;

		//#149247= IFCLOCALPLACEMENT(#29,#149246);
		shared_ptr<IfcLocalPlacement> _149247( new IfcLocalPlacement() );
		_149247->m_PlacementRelTo = _29;
		_149247->m_RelativePlacement = _149246;


		//#149250= IFCOPENINGELEMENT('1GXYHh000KDZ4pDJ4mC38t',#5,$,$,'Recess',#149247,#149249,$);
		shared_ptr<IfcOpeningElement> _149250( new IfcOpeningElement() );
		_149250->m_ObjectPlacement = _149247;
		_149250->m_Representation = _149249;
		
		//#149252= IFCAXIS2PLACEMENT3D(#149238,#41466,#7);
		shared_ptr<IfcAxis2Placement3D> _149252( new IfcAxis2Placement3D() );
		_149252->m_Location = _149238;
		_149252->m_Axis = _41466;
		_149252->m_RefDirection = _7;

		//#149253= IFCLOCALPLACEMENT(#29,#149252);
		shared_ptr<IfcLocalPlacement> _149253( new IfcLocalPlacement() );
		_149253->m_PlacementRelTo = _29;
		_149253->m_RelativePlacement = _149252;

		//#149051= IFCCARTESIANPOINT((-20.0027314640337,-14.9999691907611,-62.5000015977345));
		shared_ptr<IfcCartesianPoint> _149051 = createCartesianPoint( -20.0027314640337,-14.9999691907611,-62.5000015977345 );

		//#149052= IFCAXIS2PLACEMENT3D(#149051,#9,#7);
		shared_ptr<IfcAxis2Placement3D> _149052( new IfcAxis2Placement3D() );
		_149052->m_Location = _149051;
		_149052->m_Axis = _9;
		_149052->m_RefDirection = _7;

		//#149053= IFCEXTRUDEDAREASOLID(#149041,#149052,#9,125.);
		shared_ptr<IfcExtrudedAreaSolid> _149053( new IfcExtrudedAreaSolid() );
		_149053->m_SweptArea = _149041;
		_149053->m_Position = _149052;
		_149053->m_ExtrudedDirection = _9;
		_149053->m_Depth = shared_ptr<IfcPositiveLengthMeasure>( new IfcPositiveLengthMeasure() );
		_149053->m_Depth->m_value = 125.;

		//#149254= IFCSHAPEREPRESENTATION(#12,'Body','SweptSolid',(#149053));
		shared_ptr<IfcShapeRepresentation> _149254( new IfcShapeRepresentation() );
		_149254->m_Items.push_back( _149053 );

		//#149255= IFCPRODUCTDEFINITIONSHAPE($,$,(#149254));
		shared_ptr<IfcProductDefinitionShape> _149255( new IfcProductDefinitionShape() );
		_149255->m_Representations.push_back( _149254 );
	
		//#149256= IFCOPENINGELEMENT('1GXYHh000KEZ4pDJ4mC38t',#5,$,$,'Recess',#149253,#149255,$);
		shared_ptr<IfcOpeningElement> _149256( new IfcOpeningElement() );
		_149256->m_ObjectPlacement = _149253;
		_149256->m_Representation = _149255;

		//#149257= IFCRELVOIDSELEMENT('3V22ZULk9538pMFeV3O$qc',#5,$,$,#149245,#149256);
		shared_ptr<IfcRelVoidsElement> _149257( new IfcRelVoidsElement() );
		_149257->m_RelatingBuildingElement = _149245;
		_149257->m_RelatedOpeningElement = _149256;
		_149245->m_HasOpenings_inverse.push_back( _149257 );
	
		//#149251= IFCRELVOIDSELEMENT('1VaMJNoaH5CRSlJEQgQCZ$',#5,$,$,#149245,#149250);
		shared_ptr<IfcRelVoidsElement> _149251( new IfcRelVoidsElement() );
		_149251->m_RelatingBuildingElement = _149245;
		_149251->m_RelatedOpeningElement = _149250;
		_149245->m_HasOpenings_inverse.push_back( _149251 );
		
		shared_ptr<ReaderWriterIFC> reader( new ReaderWriterIFC() );

		reader->getUnitConverter()->m_length_unit_factor = 0.001;
	
		shared_ptr<ShapeInputData> product_shape( new ShapeInputData() );
		try
		{
			reader->convertIfcProduct( _149245, product_shape );
		}
		catch( std::exception& e )
		{
			std::cout << e.what() << std::endl;
		}

		group->addChild( product_shape->product_switch );
		
		dumpMeshset( product_shape->vec_item_data[0]->meshsets[0].get(), 1, 1 );

		shared_ptr<ShapeInputData> product_shape_opening( new ShapeInputData() );
		try
		{
			reader->convertIfcProduct( _149256, product_shape_opening );
		}
		catch( std::exception& e )
		{
			std::cout << e.what() << std::endl;
		}
		//group->addChild( product_shape_opening->product_switch );
	
		dumpMeshset( product_shape_opening->vec_item_data[0]->meshsets[0].get(), 2, 2 );


		shared_ptr<ShapeInputData> product_shape_opening2( new ShapeInputData() );
		try
		{
			reader->convertIfcProduct( _149250, product_shape_opening2 );
		}
		catch( std::exception& e )
		{
			std::cout << e.what() << std::endl;
		}
		//group->addChild( product_shape_opening->product_switch );
	
		dumpMeshset( product_shape_opening2->vec_item_data[0]->meshsets[0].get(), 3, 3 );
		

		return;
	}


	{

		// entity_id: 180190
carve::input::PolyhedronData* poly_data1 = new carve::input::PolyhedronData();
poly_data1->addVertex( carve::geom::VECTOR(0.0425, 36.3854, 6.34221 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.0425, 36.3754, 6.34221 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.0425, 36.3754, 6.59221 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.0425, 36.3854, 6.59221 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.11525, 36.3807, 6.34221 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.11525, 36.3754, 6.34221 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.1975, 36.3754, 6.59221 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.11525, 36.3854, 6.4088 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.11525, 36.3854, 6.34221 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.11637, 36.3854, 6.43805 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.1975, 36.3854, 6.34221 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.12475, 36.3854, 6.34221 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.12475, 36.3854, 6.3564 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.12475, 36.3854, 6.435 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.11525, 36.3854, 6.435 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.11838, 36.3854, 6.43946 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.118192, 36.3854, 6.43937 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.11763, 36.3854, 6.43911 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.11554, 36.3854, 6.43662 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.115487, 36.3854, 6.43642 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.11533, 36.3854, 6.43582 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.12163, 36.3854, 6.43946 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.12143, 36.3854, 6.43951 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.12083, 36.3854, 6.43968 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.11746, 36.3854, 6.43899 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.11695, 36.3854, 6.43864 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.124732, 36.3854, 6.4352 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.12468, 36.3854, 6.43582 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.12, 36.3854, 6.43975 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.119795, 36.3854, 6.43973 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.11918, 36.3854, 6.43968 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.12447, 36.3854, 6.43662 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.124382, 36.3854, 6.4368 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.12412, 36.3854, 6.43737 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.11625, 36.3854, 6.43788 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.11589, 36.3854, 6.43737 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.116805, 36.3854, 6.43849 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.11531, 36.3854, 6.43561 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.124627, 36.3854, 6.43602 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.124, 36.3854, 6.43754 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.12364, 36.3854, 6.43805 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.123495, 36.3854, 6.43819 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.12306, 36.3854, 6.43864 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.12289, 36.3854, 6.43875 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.12238, 36.3854, 6.43911 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.122192, 36.3854, 6.43919 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.120622, 36.3854, 6.43969 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.11898, 36.3854, 6.43962 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.115802, 36.3854, 6.43718 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.12475, 36.3754, 6.4088 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.12475, 36.3754, 6.34221 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.1975, 36.3754, 6.34221 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.12364, 36.3754, 6.43805 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.11525, 36.3754, 6.3564 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.12475, 36.3754, 6.435 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.11525, 36.3754, 6.435 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.11838, 36.3754, 6.43946 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.11858, 36.3754, 6.43951 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.11918, 36.3754, 6.43968 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.12376, 36.3754, 6.43788 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.12412, 36.3754, 6.43737 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.12163, 36.3754, 6.43946 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.121817, 36.3754, 6.43937 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.12238, 36.3754, 6.43911 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.12, 36.3754, 6.43975 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.120207, 36.3754, 6.43973 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.12083, 36.3754, 6.43968 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.11527, 36.3754, 6.4352 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.11533, 36.3754, 6.43582 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.11554, 36.3754, 6.43662 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.115627, 36.3754, 6.4368 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.11589, 36.3754, 6.43737 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.12447, 36.3754, 6.43662 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.124522, 36.3754, 6.43642 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.12468, 36.3754, 6.43582 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.11695, 36.3754, 6.43864 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.11712, 36.3754, 6.43875 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.11763, 36.3754, 6.43911 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.12306, 36.3754, 6.43864 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.123205, 36.3754, 6.43849 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.124697, 36.3754, 6.43561 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.115382, 36.3754, 6.43602 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.11601, 36.3754, 6.43754 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.11637, 36.3754, 6.43805 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.116515, 36.3754, 6.43819 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.117817, 36.3754, 6.43919 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.119385, 36.3754, 6.43969 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.12103, 36.3754, 6.43962 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.12255, 36.3754, 6.43899 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.124207, 36.3754, 6.43718 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.1975, 36.3854, 6.59221 ) );
poly_data1->addVertex( carve::geom::VECTOR(0.12475, 36.3801, 6.34221 ) );
poly_data1->addFace( 0, 1, 2);
poly_data1->addFace( 2, 3, 0);
poly_data1->addFace( 1, 0, 4);
poly_data1->addFace( 4, 5, 1);
poly_data1->addFace( 2, 1, 68);
poly_data1->addFace( 3, 2, 6);
poly_data1->addFace( 7, 8, 0);
poly_data1->addFace( 3, 9, 34);
poly_data1->addFace( 10, 11, 12);
poly_data1->addFace( 10, 12, 13);
poly_data1->addFace( 14, 7, 0);
poly_data1->addFace( 16, 17, 3);
poly_data1->addFace( 20, 0, 3);
poly_data1->addFace( 90, 23, 46);
poly_data1->addFace( 24, 25, 3);
poly_data1->addFace( 26, 27, 10);
poly_data1->addFace( 29, 30, 3);
poly_data1->addFace( 32, 33, 90);
poly_data1->addFace( 3, 35, 48);
poly_data1->addFace( 36, 9, 3);
poly_data1->addFace( 37, 14, 0);
poly_data1->addFace( 10, 13, 26);
poly_data1->addFace( 27, 38, 90);
poly_data1->addFace( 38, 31, 90);
poly_data1->addFace( 31, 32, 90);
poly_data1->addFace( 90, 39, 40);
poly_data1->addFace( 20, 37, 0);
poly_data1->addFace( 90, 40, 41);
poly_data1->addFace( 90, 33, 39);
poly_data1->addFace( 90, 41, 42);
poly_data1->addFace( 90, 42, 43);
poly_data1->addFace( 90, 43, 44);
poly_data1->addFace( 90, 44, 45);
poly_data1->addFace( 90, 45, 21);
poly_data1->addFace( 90, 21, 22);
poly_data1->addFace( 90, 22, 23);
poly_data1->addFace( 90, 46, 28);
poly_data1->addFace( 28, 3, 90);
poly_data1->addFace( 3, 28, 29);
poly_data1->addFace( 3, 30, 47);
poly_data1->addFace( 3, 47, 15);
poly_data1->addFace( 3, 15, 16);
poly_data1->addFace( 3, 17, 24);
poly_data1->addFace( 3, 25, 36);
poly_data1->addFace( 3, 34, 35);
poly_data1->addFace( 3, 48, 18);
poly_data1->addFace( 3, 18, 19);
poly_data1->addFace( 3, 19, 20);
poly_data1->addFace( 4, 0, 8);
poly_data1->addFace( 49, 50, 51);
poly_data1->addFace( 6, 52, 59);
poly_data1->addFace( 1, 5, 53);
poly_data1->addFace( 54, 49, 51);
poly_data1->addFace( 1, 53, 55);
poly_data1->addFace( 2, 58, 86);
poly_data1->addFace( 6, 60, 89);
poly_data1->addFace( 62, 63, 6);
poly_data1->addFace( 65, 66, 6);
poly_data1->addFace( 67, 68, 1);
poly_data1->addFace( 70, 71, 2);
poly_data1->addFace( 74, 51, 6);
poly_data1->addFace( 2, 77, 85);
poly_data1->addFace( 79, 52, 6);
poly_data1->addFace( 80, 54, 51);
poly_data1->addFace( 1, 55, 67);
poly_data1->addFace( 68, 81, 2);
poly_data1->addFace( 81, 69, 2);
poly_data1->addFace( 69, 70, 2);
poly_data1->addFace( 2, 82, 83);
poly_data1->addFace( 74, 80, 51);
poly_data1->addFace( 2, 83, 84);
poly_data1->addFace( 2, 71, 82);
poly_data1->addFace( 2, 84, 75);
poly_data1->addFace( 2, 75, 76);
poly_data1->addFace( 2, 76, 77);
poly_data1->addFace( 2, 85, 56);
poly_data1->addFace( 2, 56, 57);
poly_data1->addFace( 2, 57, 58);
poly_data1->addFace( 2, 86, 64);
poly_data1->addFace( 64, 6, 2);
poly_data1->addFace( 6, 64, 65);
poly_data1->addFace( 6, 66, 87);
poly_data1->addFace( 6, 87, 61);
poly_data1->addFace( 6, 61, 62);
poly_data1->addFace( 6, 63, 88);
poly_data1->addFace( 6, 88, 78);
poly_data1->addFace( 6, 78, 79);
poly_data1->addFace( 6, 59, 60);
poly_data1->addFace( 6, 89, 72);
poly_data1->addFace( 6, 72, 73);
poly_data1->addFace( 6, 73, 74);
poly_data1->addFace( 6, 90, 3);
poly_data1->addFace( 90, 10, 27);
poly_data1->addFace( 10, 51, 91);
poly_data1->addFace( 91, 11, 10);
poly_data1->addFace( 91, 51, 50);
poly_data1->addFace( 90, 6, 51);
poly_data1->addFace( 51, 10, 90);
poly_data1->addFace( 49, 54, 13);
poly_data1->addFace( 13, 12, 49);
poly_data1->addFace( 54, 80, 26);
poly_data1->addFace( 54, 26, 13);
poly_data1->addFace( 12, 11, 91);
poly_data1->addFace( 50, 49, 12);
poly_data1->addFace( 12, 91, 50);
poly_data1->addFace( 27, 26, 80);
poly_data1->addFace( 27, 80, 74);
poly_data1->addFace( 74, 73, 38);
poly_data1->addFace( 74, 38, 27);
poly_data1->addFace( 31, 38, 73);
poly_data1->addFace( 31, 73, 72);
poly_data1->addFace( 72, 89, 32);
poly_data1->addFace( 72, 32, 31);
poly_data1->addFace( 33, 32, 89);
poly_data1->addFace( 33, 89, 60);
poly_data1->addFace( 60, 59, 39);
poly_data1->addFace( 60, 39, 33);
poly_data1->addFace( 40, 39, 59);
poly_data1->addFace( 40, 59, 52);
poly_data1->addFace( 52, 79, 41);
poly_data1->addFace( 52, 41, 40);
poly_data1->addFace( 42, 41, 79);
poly_data1->addFace( 42, 79, 78);
poly_data1->addFace( 78, 88, 43);
poly_data1->addFace( 78, 43, 42);
poly_data1->addFace( 44, 43, 88);
poly_data1->addFace( 44, 88, 63);
poly_data1->addFace( 63, 62, 45);
poly_data1->addFace( 63, 45, 44);
poly_data1->addFace( 21, 45, 62);
poly_data1->addFace( 21, 62, 61);
poly_data1->addFace( 61, 87, 22);
poly_data1->addFace( 61, 22, 21);
poly_data1->addFace( 23, 22, 87);
poly_data1->addFace( 23, 87, 66);
poly_data1->addFace( 66, 65, 46);
poly_data1->addFace( 66, 46, 23);
poly_data1->addFace( 28, 46, 65);
poly_data1->addFace( 28, 65, 64);
poly_data1->addFace( 64, 86, 29);
poly_data1->addFace( 64, 29, 28);
poly_data1->addFace( 30, 29, 86);
poly_data1->addFace( 30, 86, 58);
poly_data1->addFace( 58, 57, 47);
poly_data1->addFace( 58, 47, 30);
poly_data1->addFace( 15, 47, 57);
poly_data1->addFace( 15, 57, 56);
poly_data1->addFace( 56, 85, 16);
poly_data1->addFace( 56, 16, 15);
poly_data1->addFace( 17, 16, 85);
poly_data1->addFace( 17, 85, 77);
poly_data1->addFace( 77, 76, 24);
poly_data1->addFace( 77, 24, 17);
poly_data1->addFace( 25, 24, 76);
poly_data1->addFace( 25, 76, 75);
poly_data1->addFace( 75, 84, 36);
poly_data1->addFace( 75, 36, 25);
poly_data1->addFace( 9, 36, 84);
poly_data1->addFace( 9, 84, 83);
poly_data1->addFace( 83, 82, 34);
poly_data1->addFace( 83, 34, 9);
poly_data1->addFace( 35, 34, 82);
poly_data1->addFace( 35, 82, 71);
poly_data1->addFace( 71, 70, 48);
poly_data1->addFace( 71, 48, 35);
poly_data1->addFace( 18, 48, 70);
poly_data1->addFace( 18, 70, 69);
poly_data1->addFace( 69, 81, 19);
poly_data1->addFace( 69, 19, 18);
poly_data1->addFace( 20, 19, 81);
poly_data1->addFace( 20, 81, 68);
poly_data1->addFace( 68, 67, 37);
poly_data1->addFace( 68, 37, 20);
poly_data1->addFace( 14, 37, 67);
poly_data1->addFace( 14, 67, 55);
poly_data1->addFace( 7, 14, 55);
poly_data1->addFace( 55, 53, 7);
poly_data1->addFace( 53, 5, 4);
poly_data1->addFace( 8, 7, 53);
poly_data1->addFace( 53, 4, 8);
// entity_id: 180231
carve::input::PolyhedronData* poly_data2 = new carve::input::PolyhedronData();
poly_data2->addVertex( carve::geom::VECTOR(0.12475, 36.2754, 6.35721 ) );
poly_data2->addVertex( carve::geom::VECTOR(0.139748, 36.2754, 6.34221 ) );
poly_data2->addVertex( carve::geom::VECTOR(0.12475, 36.2754, 6.34222 ) );
poly_data2->addVertex( carve::geom::VECTOR(0.12475, 36.4854, 6.35721 ) );
poly_data2->addVertex( carve::geom::VECTOR(0.139748, 36.4854, 6.34221 ) );
poly_data2->addVertex( carve::geom::VECTOR(0.12475, 36.4854, 6.34222 ) );
poly_data2->addFace( 0, 1, 4);
poly_data2->addFace( 4, 3, 0);
poly_data2->addFace( 1, 2, 5);
poly_data2->addFace( 5, 4, 1);
poly_data2->addFace( 2, 0, 3);
poly_data2->addFace( 3, 5, 2);
poly_data2->addFace( 1, 0, 2);
poly_data2->addFace( 4, 5, 3);



carve::mesh::MeshSet<3>* meshset1 = poly_data1->createMesh(carve::input::opts());
carve::mesh::MeshSet<3>* meshset2 = poly_data2->createMesh(carve::input::opts());
carve::mesh::MeshSet<3>* meshset3 = NULL;//poly_data3->createMesh(carve::input::opts());

std::stringstream err;
bool meshset1_ok = ConverterOSG::checkMeshSet( meshset1, err, -1 );
bool meshset2_ok = ConverterOSG::checkMeshSet( meshset2, err, -1 );

	carve::mesh::MeshSimplifier simplifier;
	double min_colinearity = 0.0001;
	double min_delta_v = 1.0;
	double min_normal_angle = M_PI/180.0;//0.001;
	double min_length = 0.001;
	//simplifier.improveMesh( result1, min_colinearity, min_delta_v, min_normal_angle );
	simplifier.simplify( meshset1, min_colinearity, min_delta_v, min_normal_angle, min_length );


//renderMeshsetInDebugViewer( meshset1, osg::Vec4f( 0, 0, 0.5, 1 ), false );
renderMeshsetInDebugViewer( meshset2, osg::Vec4f( 0, 0.5, 0.5, 1 ), true );

osg::Geode* geode_txt = new osg::Geode();
root->addChild( geode_txt );

drawVertexNumbers( poly_data1, osg::Vec4( 0, 0, 1, 1 ), geode_txt );
drawVertexNumbers( poly_data2, osg::Vec4( 0, 0, 0, 1 ), geode_txt );


carve::csg::CSG csg;
csg.hooks.registerHook(new carve::csg::CarveTriangulatorWithImprovement(), carve::csg::CSG::Hooks::PROCESS_OUTPUT_FACE_BIT);

carve::mesh::MeshSet<3>* result1 = NULL;
bool failed = false;
try
{
	result1 = csg.compute( meshset1, meshset2, carve::csg::CSG::A_MINUS_B, NULL, carve::csg::CSG::CLASSIFY_EDGE );
	for( int i=0; i<result1->meshes.size(); ++i )
	{
	//	result1->meshes[i]->recalc();
	}

	bool result1_ok = ConverterOSG::checkMeshSet( result1, err, -1 );

	if( result1_ok )
	{
			
		carve::mesh::MeshSimplifier simplifier;
		double min_colinearity = 0.1;
		double min_delta_v = 1.0;
		double min_normal_angle = M_PI/180.0;//0.001;
		double min_length = 0.001;
		//simplifier.improveMesh( result1, min_colinearity, min_delta_v, min_normal_angle );
		simplifier.simplify( result1, min_colinearity, min_delta_v, min_normal_angle, min_length );
		simplifier.removeFins(result1);
	}
}
catch(...)
{
	failed = true;
}

carve::mesh::MeshSet<3>* result2 = NULL;
if( meshset3 )
{
	try
	{
		result2 = csg.compute( result1, meshset3, carve::csg::CSG::A_MINUS_B, NULL, carve::csg::CSG::CLASSIFY_EDGE );
	}
	catch(...)
	{
		failed = true;
	}
}

std::stringstream strs;
bool result_ok = ConverterOSG::checkMeshSet(result2, strs, 0 );
if( !result_ok ) failed = true;


if( result2 )
{
	renderMeshsetInDebugViewer( result2, osg::Vec4f( 0, 0, 0.5, 1 ), false );
}
else if( result1 )
{
	renderMeshsetInDebugViewer( result1, osg::Vec4f( 0, 0, 0.5, 1 ), false );
}



if( meshset3 )
{
	//renderMeshsetInDebugViewer( meshset3, osg::Vec4f( 0, 0.5, 0.5, 1 ), true );
	//drawVertexNumbers( poly_data3, osg::Vec4( 0, 0, 0, 1 ), geode_txt );
}
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
		ConverterOSG::drawMeshSet( tetrahedron.get(), geode );
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

		ConverterOSG::drawMeshSet( triangle.get(), geode );


		//cut triangle with tetrahedron.
		shared_ptr<carve::mesh::MeshSet<3> > is_poly( carve::csg::CSG().compute( tetrahedron.get(), triangle.get(), carve::csg::CSG::INTERSECTION) );

		ConverterOSG::drawMeshSet( is_poly.get(), geode );

	}

	shared_ptr<IfcSphere> sphere( new IfcSphere() );
	sphere->m_Radius = shared_ptr<IfcPositiveLengthMeasure>( new IfcPositiveLengthMeasure() );
	sphere->m_Radius->m_value = 0.75;

	shared_ptr<GeometrySettings> geom_settings( new GeometrySettings() );
	geom_settings->m_num_vertices_per_circle = 50;
	shared_ptr<UnitConverter> unit_converter( new UnitConverter() );
	shared_ptr<RepresentationConverter> representation_converter( new RepresentationConverter( geom_settings, unit_converter ) );

	shared_ptr<ItemData> item_data( new ItemData() );
	std::stringstream strs_err;
	representation_converter->getSolidConverter()->convertIfcCsgPrimitive3D( sphere, carve::math::Matrix::TRANS(1.0, 1.0, 1.0), item_data, strs_err );
	item_data->createMeshSetsFromClosedPolyhedrons();


	if( item_data->meshsets.size() != 1 )
	{
		return;
	}

	shared_ptr<carve::mesh::MeshSet<3> > sphere_meshset = item_data->meshsets[0];
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
	representation_converter->getSolidConverter()->convertIfcExtrudedAreaSolid( solid, carve::math::Matrix::TRANS(0.7, 0.0, 0.0), item_data2, strs_err );
	item_data2->createMeshSetsFromClosedPolyhedrons();


	if( item_data2->meshsets.size() != 1 )
	{
		return;
	}

	shared_ptr<carve::mesh::MeshSet<3> > solid_meshset = item_data2->meshsets[0];
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

	ConverterOSG::drawMeshSet( solid_result.get(), geode_solid );
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

		ConverterOSG::drawMeshSet( opening.get(), geode );

		osg::MatrixTransform* mt = new osg::MatrixTransform( osg::Matrix::translate( 10, 10, 0 ) );
		mt->addChild(geode);
		group->addChild(mt);
	}

	{
		osg::Geode* geode = new osg::Geode();
		ConverterOSG::drawMeshSet( subtract_from.get(), geode );

		osg::MatrixTransform* mt = new osg::MatrixTransform( osg::Matrix::translate( -10, 10, 0 ) );
		mt->addChild(geode);
		group->addChild(mt);
	}
	{
		osg::Geode* geode = new osg::Geode();
		ConverterOSG::drawMeshSet( result.get(), geode );
		group->addChild(geode);
	}
}

void createTest2(osg::Group* group)
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

		ConverterOSG::drawMeshSet( meshset, geode );

	}
}

void createTest4(osg::Group* group)
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
		ConverterOSG::drawMeshSet( meshset.get(), geode );
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

		ConverterOSG::checkMeshSet( meshset.get(), err, -1 );
		ConverterOSG::drawMeshSet( meshset.get(), geode );

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

	ConverterOSG::drawPolyhedron( poly.get(), geode );
	ConverterOSG::drawPolyhedron( poly_opening.get(), geode );

	carve::csg::CSG csg;
	csg.hooks.registerHook(new carve::csg::CarveTriangulator, carve::csg::CSG::Hooks::PROCESS_OUTPUT_FACE_BIT);
	{
		osg::Geode* geode = new osg::Geode();
		osg::MatrixTransform* mt = new osg::MatrixTransform( osg::Matrix::translate( 0, 2, 0 ) );
		mt->addChild(geode);
		group->addChild(mt);

		shared_ptr<carve::mesh::MeshSet<3> > result( csg.compute( poly_data.createMesh(carve::input::opts()), poly_opening_data.createMesh(carve::input::opts()), carve::csg::CSG::A_MINUS_B) );
		ConverterOSG::drawMeshSet( result.get(), geode );
	}
}
#endif
