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

#include <osg/Group>
#include <osg/Material>
#include <osgUtil/Optimizer>
#include <osgViewer/Viewer>
#include <osgGA/GUIActionAdapter>
#include <osgDB/Registry>

#include <QtCore/QFile>
#include <QtCore/QTextStream>

#include "ifcpp/model/IfcPPModel.h"
#include "ifcpp/model/IfcPPException.h"
#include "ifcpp/reader/IfcStepReader.h"
#include "ifcpp/writer/IfcStepWriter.h"
#include "ifcpp/IFC4/include/IfcProduct.h"
#include "ifcppgeometry/ReaderWriterIFC.h"

#include "viewer/CameraMan3D.h"
#include "cmd/CmdRemoveSelectedObjects.h"
#include "cmd/CommandManager.h"
#include "ViewController.h"
#include "IfcPlusPlusSystem.h"

IfcPlusPlusSystem::IfcPlusPlusSystem()
{
	m_view_controller = shared_ptr<ViewController>( new ViewController( this ) );
	m_command_manager = shared_ptr<CommandManager>( new CommandManager() );

	m_drag = false;
	m_ga_t0 = NULL;

	m_ifc_model		= shared_ptr<IfcPPModel>( new IfcPPModel() );//m_reader_writer->getIfcPPModel();
	//

	if( osgDB::Registry::instance() )
	{
		osgDB::ReaderWriter* rw = osgDB::Registry::instance()->getReaderWriterForExtension( "ifc" );
		ReaderWriterIFC* rw_ifc = dynamic_cast<ReaderWriterIFC*>(rw);
		if( rw_ifc )
		{
			m_reader_writer = rw_ifc;
		}
	}

	if( !m_reader_writer )
	{
		m_reader_writer = new ReaderWriterIFC();
	}
	m_reader_writer->setModel( m_ifc_model );
}

IfcPlusPlusSystem::~IfcPlusPlusSystem()
{
}

void IfcPlusPlusSystem::setIfcModel( shared_ptr<IfcPPModel>& model )
{
	m_ifc_model = model;
}

bool IfcPlusPlusSystem::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	bool handled=false;
	try
	{
		switch( ea.getEventType() )
		{
			case osgGA::GUIEventAdapter::FRAME : 
			{
				break;
			}
			case osgGA::GUIEventAdapter::MOVE :
			{
				//handled = handleMouseMoved( ea, aa );
				break;
			}
			case  osgGA::GUIEventAdapter::KEYDOWN :
			{
				const int key = ea.getKey();
				if( key == osgGA::GUIEventAdapter::KEY_Escape )
				{
				}
				else if( key == osgGA::GUIEventAdapter::KEY_Return )
				{
				}
				else if( key == osgGA::GUIEventAdapter::KEY_Delete )
				{
					shared_ptr<CmdRemoveSelectedObjects> cmd_remove( new CmdRemoveSelectedObjects( this ) );
					m_command_manager->executeCommand( cmd_remove );
				}
				else if( ea.getKey() == osgGA::GUIEventAdapter::KEY_F5 )
				{

				}
				else if( key=='l' )
				{

				}
				else if( key == 'm' )
				{
					osg::Material* mat = m_view_controller->getDefaultMaterial();
					float shinyness = mat->getShininess( osg::Material::FRONT_AND_BACK ) + 1.f;
					mat->setShininess( osg::Material::FRONT_AND_BACK, shinyness );
					std::cout << "shininess: " << shinyness << std::endl;
				}
				else if( key == 'n' )
				{
					osg::Material* mat = m_view_controller->getDefaultMaterial();
					float shinyness = mat->getShininess( osg::Material::FRONT_AND_BACK ) - 1.f;
					mat->setShininess( osg::Material::FRONT_AND_BACK, shinyness );
					std::cout << "shininess: " << shinyness << std::endl;
				}
				else if( key == 'b' )
				{
					m_view_controller->toggleBoundingSphere();
				}
				else if( key == 't' )
				{
					// TODO: fix or disable conflicting window transparency statesets
					m_view_controller->toggleModelTransparency();
				}
				break;
			}
			case osgGA::GUIEventAdapter::DOUBLECLICK :
			{
				//handled = handleMouseDoubleClick( ea, aa );
				break;
			}
			case osgGA::GUIEventAdapter::PUSH :
			{
				m_drag = false;
				// TODO: set camera center to first intersection point
				break;
			}
			case osgGA::GUIEventAdapter::RELEASE :
			{
				if( m_ga_t0 != NULL  )
				{
					if( !m_drag )
					{
						unsigned int buttonMask = m_ga_t0->getButtonMask();
						if( buttonMask==osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON )
						{
							handled = intersectModel( ea, aa, false, false );
						}
					}
				}

				m_drag = false;
				break;
			}
			case osgGA::GUIEventAdapter::DRAG :
			{
				m_drag = true;
				break;
			}

			default:
				break;
		}
	}
	catch( IfcPPException& e )
	{
		std::cout << e.what();
	}
#ifndef _DEBUG
	catch( std::exception& e )
	{
		std::cout << e.what();
	}
#endif
	m_ga_t0 = &ea;
	return handled;
}

#ifdef _DEBUG
#define _DEBUG_GEOMETRY
#endif

//#define POLYTOPE_INTERSECTOR
bool IfcPlusPlusSystem::intersectModel( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa, bool click, bool doubleclick )
{
	osgViewer::View* view = dynamic_cast<osgViewer::View*>(&aa);
	if( !view )
	{
		return false;
	}
	double mx = ea.getXnormalized();
	double my = ea.getYnormalized();
	
	double w = 0.005;
	double h = 0.005;
	
#ifdef POLYTOPE_INTERSECTOR
	osg::ref_ptr<osgUtil::PolytopeIntersector> picker = new osgUtil::PolytopeIntersector( osgUtil::Intersector::PROJECTION, mx-w, my-h, mx+w, my+h );
#else
	osg::ref_ptr<osgUtil::LineSegmentIntersector> picker = new osgUtil::LineSegmentIntersector( osgUtil::Intersector::PROJECTION, ea.getXnormalized(),ea.getYnormalized() );
#endif
	picker->setIntersectionLimit( osgUtil::Intersector::LIMIT_ONE_PER_DRAWABLE );
	osgUtil::IntersectionVisitor iv( picker.get() );
	osg::Camera* cam = view->getCamera();
	cam->accept(iv);

	if( picker->containsIntersections() )
	{
#ifdef POLYTOPE_INTERSECTOR
		osgUtil::PolytopeIntersector::Intersection intersection = picker->getFirstIntersection();
#else
		osgUtil::LineSegmentIntersector::Intersection intersection = picker->getFirstIntersection();
#endif
		
		osg::NodePath& nodePath = intersection.nodePath;

		for( unsigned int i=0; i<nodePath.size(); ++i )
		{
			osg::Node* node = nodePath[nodePath.size()-i-1];
			const std::string node_name = node->getName();

			// check if picked object is a representation of an IfcProduct
			if( node_name.length() == 0 ) continue;
			if( node_name.at(0) != '#' ) continue;

			osg::Group* group = dynamic_cast<osg::Group*>( node );
			if( !group )
			{
				continue;
			}

			std::string id_str = node_name.substr(1,node_name.length()-1);
			int id = atoi( id_str.c_str() );

			std::map<int, shared_ptr<selectedEntity> >::iterator it_selected = m_map_selected.find( id );

			if( it_selected != m_map_selected.end() )
			{
				shared_ptr<selectedEntity> selected_entity = it_selected->second;
				// is already selected, so deselect
				setObjectSelected( selected_entity->entity, false, selected_entity->osg_group );
				return true;
			}
			else
			{
				// select
				const std::map<int,shared_ptr<IfcPPEntity> >& map_ifc_objects = m_ifc_model->getMapIfcObjects();
				std::map<int,shared_ptr<IfcPPEntity> >::const_iterator it_find = map_ifc_objects.find(id);
				if( it_find != map_ifc_objects.end() )
				{
					shared_ptr<IfcPPEntity> entitiy_selected = it_find->second;
					setObjectSelected( entitiy_selected, true, group );

#ifdef _DEBUG_GEOMETRY
					shared_ptr<IfcProduct> product_selected = dynamic_pointer_cast<IfcProduct>(entitiy_selected);
					
					if( product_selected )
					{
						std::stringstream err;
						try
						{
							
							osg::MatrixTransform* mt = new osg::MatrixTransform( osg::Matrix::translate( 0, 0, 1 ) );
							shared_ptr<ReaderWriterIFC::ProductShape> read_result( new ReaderWriterIFC::ProductShape() );
							osg::ref_ptr<ReaderWriterIFC> reader_writer( new ReaderWriterIFC() );
							reader_writer->convertIfcProduct( product_selected, read_result );
							mt->addChild( read_result->product_group );
							m_view_controller->getRootNode()->addChild( mt );
						}
						catch( IfcPPException& e )
						{
							err << e.what();
						}
					}
#endif

					return true;
				}
			}

			return true;
		}
		return true;
	}
	return false;
}

osg::Group* findNodeByIfcId( osg::Group* group, int ifc_id )
{
	int num_children = group->getNumChildren();
	for( int i=0; i<num_children; ++i )
	{
		osg::Node* child_node = group->getChild( i );
		osg::Group* child = dynamic_cast<osg::Group*>(child_node);
		if( !child )
		{
			continue;
		}

		const std::string child_name = child->getName();

		if( child_name.length() > 0 )
		{
			if( child_name.substr(0,1).compare("#") == 0 )
			{
				std::string id_str = child_name.substr(1,child_name.length()-1);
				int id = atoi( id_str.c_str() );

				if( id == ifc_id )
				{
					return child;
				}
			}
		}
		osg::Group* child_of_child = findNodeByIfcId( child, ifc_id );
		if( child_of_child != 0 )
		{
			return child_of_child;
		}
	}
	return 0;
}


void IfcPlusPlusSystem::setObjectSelected( shared_ptr<IfcPPEntity> ifc_object, bool selected, osg::Group* grp )
{
	const int id = ifc_object->getId();
	if( grp == 0 )
	{
		osg::Group* model_group = m_view_controller->getModelNode();
		grp = findNodeByIfcId( model_group, id );
	}

	if( selected )
	{
		// insert into set of selected objects
		if( grp )
		{
			shared_ptr<selectedEntity> selected_entity( new selectedEntity() );
			selected_entity->entity = ifc_object;
			selected_entity->osg_group = grp;
			selected_entity->original_stateset = grp->getOrCreateStateSet();
			m_map_selected[id] = selected_entity;

			// now overwrite stateset of object
			grp->setStateSet( m_view_controller->getStateSetSelected() );
		}
		emit( signalObjectSelected( ifc_object ) );
	}
	else
	{
		// deselect
		if( grp )
		{
			std::map<int, shared_ptr<selectedEntity> >::iterator it_selected = m_map_selected.find( id );

			if( it_selected != m_map_selected.end() )
			{
				shared_ptr<selectedEntity> selected_entity = it_selected->second;

				osg::StateSet* state_original = selected_entity->original_stateset.get();
				if( state_original )
				{
					grp->setStateSet( state_original );
				}
				m_map_selected.erase( it_selected );
			}
		}
		emit( signalObjectUnSelected( ifc_object ) );
	}
}

void IfcPlusPlusSystem::clearSelection()
{
	m_map_selected.clear();

}
