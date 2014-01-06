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

#pragma warning( disable: 4996 )
#include <QtCore/qglobal.h>
#if (QT_VERSION < QT_VERSION_CHECK(5, 0, 0))
	#include <QtGui/qboxlayout.h>
	#include <QtGui/qtreewidget.h>
	#include <QtGui/qfiledialog.h>
	#include <QtGui/qlabel.h>
	#include <QtGui/qcombobox.h>
	#include <QtGui/qpushbutton.h>
	#include <QtGui/qprogressbar.h>
	#include <QtGui/qlineedit.h>
	#include <QtGui/qsplitter.h>
	#include <QtGui/qapplication.h>
	#include <QtGui/qtextedit.h>
	#include <QtGui/qevent.h>
	#include <QtCore/qsettings.h>
#else
	#include <QtWidgets/QVBoxLayout>
	#include <QtWidgets/QTreeWidget>
	#include <QtWidgets/QFileDialog>
	#include <QtWidgets/QLabel>
	#include <QtWidgets/QComboBox>
	#include <QtWidgets/QPushButton>
	#include <QtWidgets/QProgressBar>
	#include <QtWidgets/QLineEdit>
	#include <QtWidgets/QSplitter>
	#include <QtWidgets/QApplication>
	#include <QtWidgets/QTextEdit>
	#include <QtGui/qevent.h>
	#include <QtCore/QSettings>
#endif

#include "ifcpp/reader/IfcPlusPlusReader.h"
#include "ifcpp/writer/IfcStepWriter.h"
#include "ifcpp/model/shared_ptr.h"
#include "ifcpp/model/IfcPPModel.h"
#include "ifcpp/model/IfcPPException.h"
#include "ifcpp/guid/CreateGuid_64.h"
#include "ifcpp/IFC4/include/IfcLabel.h"
#include "ifcpp/IFC4/include/IfcObjectDefinition.h"
#include "ifcpp/IFC4/include/IfcSpatialStructureElement.h"
#include "ifcpp/IFC4/include/IfcRelContainedInSpatialStructure.h"
#include "ifcpp/IFC4/include/IfcRelAggregates.h"
#include "ifcpp/IFC4/include/IfcAxis2Placement3D.h"
#include "ifcpp/IFC4/include/IfcGeometricRepresentationContext.h"
#include "ifcpp/IFC4/include/IfcDirection.h"
#include "ifcpp/IFC4/include/IfcProject.h"
#include "ifcpp/IFC4/include/IfcAxis2Placement.h"
#include "ifcpp/IFC4/include/IfcAxis2Placement.h"
#include "ifcpp/IFC4/include/IfcLabel.h"
#include "ifcppgeometry/ReaderWriterIFC.h"
#include "ifcppgeometry/GeomUtils.h"

#include "IfcPlusPlusSystem.h"
#include "ViewController.h"
#include "viewer/ViewerWidget.h"
#include "viewer/Orbit3DManipulator.h"
#include "cmd/CmdLoadIfcFile.h"
#include "cmd/CmdWriteIfcFile.h"
#include "cmd/CommandManager.h"
#include "TabReadWrite.h"


class StoreyWidget : public QWidget
{
public:
	StoreyWidget( IfcPlusPlusSystem* sys ) : m_system(sys)
	{
		m_vbox = new QVBoxLayout();
		setLayout( m_vbox );
	}
	~StoreyWidget()
	{

	}

	void update()
	{
		while ((  m_vbox->takeAt(0)) != 0) {
		}
		//m_vbox->takeAt(removeWidget( m_vbox->widget())

		osg::ref_ptr<ReaderWriterIFC> rw = m_system->getReaderWriterIFC();
//		std::vector<shared_ptr<ReaderWriterIFC::BuildingStoreyGroup> >& vec_building_storeys = rw->getBuildingStoreys();

		//for( int i=0; i<vec_building_storeys.size(); ++i )
		//{
		//	shared_ptr<ReaderWriterIFC::BuildingStoreyGroup>& building_storey = vec_building_storeys[i];
		//	osg::ref_ptr<osg::MatrixTransform> storey_transform = building_storey->storey_transform;

		//	m_vbox->addWidget( new QLabel( storey_transform->getName().c_str() ) );
		//}

		
	}
	
	IfcPlusPlusSystem* m_system;
	QVBoxLayout* m_vbox;
};

class IfcTreeWidget : public QTreeWidget
{
public:
	QModelIndex	indexFromItem( QTreeWidgetItem * item, int column = 0 ) const { return QTreeWidget::indexFromItem( item, column ); }
	QTreeWidgetItem* itemFromIndex( const QModelIndex & index ) const { return QTreeWidget::itemFromIndex( index ); }
};

class MyFileDialog : public QFileDialog
{
public:
	MyFileDialog( QWidget* parent, const QString& caption ) : QFileDialog( parent, caption )
	{
		QSettings settings(QSettings::UserScope, QLatin1String("IfcPlusPlus"));
		QStringList keys = settings.allKeys();
		if( keys.contains( "MyFileDialogState" ) )
		{
			restoreState(settings.value(QLatin1String("MyFileDialogState")).toByteArray());
			restoreGeometry(settings.value(QLatin1String("MyFileDialogGeometry")).toByteArray());
		}
		setFileMode( QFileDialog::ExistingFile );
	}
	~MyFileDialog()
	{
		QSettings settings(QSettings::UserScope, QLatin1String("IfcPlusPlus"));
		settings.setValue(QLatin1String("MyFileDialogState"), saveState());
		settings.setValue(QLatin1String("MyFileDialogGeometry"), saveGeometry());
	}
};



TabReadWrite::TabReadWrite( IfcPlusPlusSystem* sys, ViewerWidget* viewer, QWidget* parent ) : m_system(sys), m_viewer(viewer), QWidget( parent )
{
	m_block_selection_signals = false;

	QSettings settings(QSettings::UserScope, QLatin1String("IfcPlusPlus"));
	QStringList keys = settings.allKeys();
	
	if( keys.contains( "recentFiles" ) )
	{
		m_recent_files = settings.value("recentFiles").toStringList();
	}
	
	m_combo_recent_files = new QComboBox();
	connect( m_combo_recent_files, SIGNAL(currentIndexChanged(int)),	this,	SLOT( slotRecentFilesIndexChanged(int) ) );

	m_btn_load = new QPushButton( "Load" );
	m_btn_load->setStyleSheet( "min-width: 30px;" );
	connect( m_btn_load, SIGNAL(clicked()),	this,	SLOT( slotLoadRecentIfcFileClicked() ) );
	
	updateRecentFilesCombo();


	QPushButton* btn_add_file = new QPushButton( "Choose file" );
	connect( btn_add_file, SIGNAL( clicked() ), this, SLOT( slotAddOtherIfcFileClicked() ) );

	// write
	m_le_path_write = new QLineEdit( "IfcPlusPlus-out.ifc" );
	QPushButton* btn_set_out_path = new QPushButton( "..." );
	connect( btn_set_out_path, SIGNAL( clicked() ), this, SLOT( slotSetWritePathClicked() ) );
	

	QString path_out;
	if( settings.contains( "pathIfcFileOut" ) )
	{
		path_out = settings.value("pathIfcFileOut").toString();
	}
	m_le_path_write->setText( path_out );
	QPushButton* btn_write_file = new QPushButton( "Write ifc file" );
	connect( btn_write_file, SIGNAL( clicked() ), this, SLOT( slotWriteFileClicked() ) );

	m_txt_out = new QTextEdit();

#ifdef _DEBUG
	std::stringstream uuid_strs;
//	for( int i=0; i<10; ++i )
	uuid_strs << createGUID32().c_str() << std::endl;
	uuid_strs << CreateCompressedGuidString22().c_str() << std::endl;

	m_txt_out->setText( uuid_strs.str().c_str() );
#endif

	m_storey_widget = new StoreyWidget( m_system );

	m_ifc_tree_widget = new IfcTreeWidget();
	m_ifc_tree_widget->setColumnCount( 2 );
	QStringList tree_headers;
	tree_headers << "Label" << "Object id" << "Class name";
	m_ifc_tree_widget->setHeaderLabels( tree_headers );
	m_ifc_tree_widget->setColumnWidth( 0, 200 );
	m_ifc_tree_widget->setColumnWidth( 1, 100 );
	m_ifc_tree_widget->setColumnWidth( 2, 100 );
	m_ifc_tree_widget->setSelectionBehavior( QAbstractItemView::SelectRows );
	

	connect( m_ifc_tree_widget, SIGNAL( currentItemChanged( QTreeWidgetItem*, QTreeWidgetItem* ) ), this, SLOT( slotTreewidgetSelectionChanged(QTreeWidgetItem*, QTreeWidgetItem*) ) );
	
	m_progress_bar = new QProgressBar();
	m_progress_bar->setRange( 0, 1000 );

	// layout
	QHBoxLayout* combo_hbox = new QHBoxLayout();
	combo_hbox->addWidget( m_combo_recent_files, 1 );
	combo_hbox->addWidget( btn_add_file );
	combo_hbox->addWidget( m_btn_load, 0 );

	//QHBoxLayout* write_hbox = new QHBoxLayout();
	//write_hbox->addWidget( m_le_path_write );
	//write_hbox->addWidget( btn_set_out_path );
	//write_hbox->addWidget( btn_write_file );

	m_io_widget = new QWidget(this);
	QVBoxLayout* io_vbox = new QVBoxLayout(m_io_widget);
	io_vbox->setContentsMargins( 0, 0, 0, 0 );
	io_vbox->addWidget( new QLabel( "Read IFC file" ), 0 );
	io_vbox->addLayout( combo_hbox, 1 );
	io_vbox->addSpacing( 10 );
	//io_vbox->addWidget( new QLabel( "Write IFC file" ), 0 );
	//io_vbox->addLayout( write_hbox );
	io_vbox->addStretch( 1 );
	io_vbox->addWidget( m_progress_bar,	0 );

	m_io_splitter = new QSplitter( Qt::Horizontal );
	m_io_splitter->addWidget( m_io_widget );
	m_io_splitter->addWidget( m_txt_out );
	m_io_splitter->addWidget( m_storey_widget );
	m_io_splitter->addWidget( m_ifc_tree_widget );

	QHBoxLayout* hbox = new QHBoxLayout();
	hbox->addWidget( m_io_splitter );
	setLayout( hbox );

	if( keys.contains( "IOsplitterSizes" ) )
	{
		m_io_splitter->restoreState(settings.value("IOsplitterSizes").toByteArray());
	}
	connect( m_system, SIGNAL( signalObjectsSelected(std::map<int, shared_ptr<IfcPPEntity> >&) ),	this, SLOT( slotObjectsSelected(std::map<int, shared_ptr<IfcPPEntity> >&) ) );
}
TabReadWrite::~TabReadWrite()
{
}

void TabReadWrite::closeEvent( QCloseEvent* )
{
	QSettings settings(QSettings::UserScope, QLatin1String("IfcPlusPlus"));
	settings.setValue("IOsplitterSizes", m_io_splitter->saveState());
}

void TabReadWrite::slotProgressValueWrapper( void* ptr, double value )
{
	TabReadWrite* myself = (TabReadWrite*)ptr;
	if( myself )
	{
		myself->slotProgressValue( value );
	}
}
void TabReadWrite::slotMessageWrapper( void* ptr, const std::string& str )
{
	TabReadWrite* myself = (TabReadWrite*)ptr;
	if( myself )
	{
		myself->slotTxtOut( str.c_str() );
	}
}
void TabReadWrite::slotErrorWrapper( void* ptr, const std::string& str )
{
	TabReadWrite* myself = (TabReadWrite*)ptr;
	if( myself )
	{
		myself->slotTxtOutError( str.c_str() );
	}
}

void TabReadWrite::keyPressEvent( QKeyEvent* e )
{
	if( e->text() == "c" )
	{
		m_recent_files.clear();
		QSettings settings(QSettings::UserScope, QLatin1String("IfcPlusPlus"));
		settings.setValue("recentFiles",m_recent_files );
		updateRecentFilesCombo();
	}
}

void TabReadWrite::updateRecentFilesCombo()
{
	m_combo_recent_files->blockSignals( true );
	m_combo_recent_files->clear();
	foreach( QString path_recent_file, m_recent_files )
	{
		QFileInfo fi( path_recent_file );
		QString file_name = fi.fileName();
		m_combo_recent_files->addItem( file_name );
	}
	m_combo_recent_files->blockSignals( false );
}

QTreeWidgetItem* resolveTreeItems( shared_ptr<IfcPPObject> obj, std::map<int,shared_ptr<IfcObjectDefinition> >& map_visited )
{
	QTreeWidgetItem* item = NULL;

	std::vector<shared_ptr<IfcObjectDefinition> >::iterator it_object_def;
	std::vector<shared_ptr<IfcProduct> >::iterator it_product;

	shared_ptr<IfcObjectDefinition> obj_def = dynamic_pointer_cast<IfcObjectDefinition>(obj);
	if( obj_def )
	{
		if( map_visited.find( obj_def->getId() ) != map_visited.end() )
		{
			return NULL;
		}
		map_visited[obj_def->getId()] = obj_def;


		item = new QTreeWidgetItem();
		
		if( obj_def->m_Name )
		{
			if( obj_def->m_Name->m_value.size() > 0 )
			{
				item->setText( 0, QString::fromLocal8Bit( obj_def->m_Name->m_value.c_str() ) );
			}
		}
		item->setText( 1, QString::number( obj_def->getId() ) );
		item->setText( 2, obj_def->classname() );

		if( obj_def->m_IsDecomposedBy_inverse.size() > 0 )
		{
			std::vector<weak_ptr<IfcRelAggregates> >& vec_IsDecomposedBy = obj_def->m_IsDecomposedBy_inverse;
			std::vector<weak_ptr<IfcRelAggregates> >::iterator it;
			for( it=vec_IsDecomposedBy.begin(); it!=vec_IsDecomposedBy.end(); ++it )
			{
				shared_ptr<IfcRelAggregates> rel_agg( *it );
				std::vector<shared_ptr<IfcObjectDefinition> >& vec = rel_agg->m_RelatedObjects;
	
				for( it_object_def=vec.begin(); it_object_def!=vec.end(); ++it_object_def )
				{
					shared_ptr<IfcObjectDefinition> child_obj_def = (*it_object_def);
					QTreeWidgetItem* child_tree_item = resolveTreeItems( child_obj_def, map_visited );
					if( child_tree_item != NULL )
					{
						item->addChild( child_tree_item );
					}
				}
			}
		}

		shared_ptr<IfcSpatialStructureElement> spatial_ele = dynamic_pointer_cast<IfcSpatialStructureElement>(obj_def);
		if( spatial_ele )
		{
			std::vector<weak_ptr<IfcRelContainedInSpatialStructure> >& vec_contained = spatial_ele->m_ContainsElements_inverse;
			if( vec_contained.size() > 0 )
			{
				std::vector<weak_ptr<IfcRelContainedInSpatialStructure> >::iterator it_rel_contained;
				for( it_rel_contained=vec_contained.begin(); it_rel_contained!=vec_contained.end(); ++it_rel_contained )
				{

					shared_ptr<IfcRelContainedInSpatialStructure> rel_contained( *it_rel_contained );
						
					std::vector<shared_ptr<IfcProduct> >& vec_related_elements = rel_contained->m_RelatedElements;
					std::vector<shared_ptr<IfcProduct> >::iterator it;
			
					for( it=vec_related_elements.begin(); it!=vec_related_elements.end(); ++it )
					{
						shared_ptr<IfcProduct> related_product = (*it);
				
						QTreeWidgetItem* child_tree_item = resolveTreeItems( related_product, map_visited );
						if( child_tree_item != NULL )
						{
							item->addChild( child_tree_item );
						}
					}
				}
			}
		}
	}
	
	return item;
}

void TabReadWrite::slotRecentFilesIndexChanged(int)
{
	slotProgressValue(0);
	m_btn_load->setFocus();
}

void TabReadWrite::slotLoadIfcFile( std::string& path_in )
{
	m_system->getIfcModel()->clearIfcModel();
	slotTxtOut( QString( "loading file: " ) + path_in.c_str() );
	QApplication::processEvents();
	
	clock_t millisecs = clock();
	
	m_ifc_tree_widget->blockSignals(true);
	m_ifc_tree_widget->clear();
	m_ifc_tree_widget->blockSignals(false);
	m_txt_out->clear();
	QSettings settings(QSettings::UserScope, QLatin1String("IfcPlusPlus"));

	if( !QFile::exists(path_in.c_str()) )
	{
		slotTxtOutError( QString("file ") + path_in.c_str() + QString(" does not exist\n") );

		// remove all non-existing files from recent files combo
		for( int i=0; i<m_recent_files.size(); )
		{
			QString recent_file = m_recent_files.at(i);
			if( !QFile::exists(recent_file) )
			{
				m_recent_files.takeAt( i );
			}
			else
			{
				++i;
			}
		}
		settings.setValue("recentFiles",m_recent_files );
		updateRecentFilesCombo();
		return;
	}
	else
	{
		// move to top of recent files list
		int i = m_recent_files.indexOf( path_in.c_str() );
		if( i > 0 )
		{
			QString current_path = m_recent_files.takeAt( i );
			m_recent_files.insert( 0, current_path );
			m_recent_files.removeDuplicates();
			settings.setValue("recentFiles",m_recent_files );
			updateRecentFilesCombo();
		}
		else
		{
			m_recent_files.insert( 0, path_in.c_str() );
			m_recent_files.removeDuplicates();
			settings.setValue("recentFiles",m_recent_files );
			updateRecentFilesCombo();
		}
	}

	// TODO: loadIfcFile in a separate thread
	std::stringstream warning, err;
	try
	{
		shared_ptr<CmdLoadIfcFile> cmd_load( new CmdLoadIfcFile( m_system ) );
		m_system->getReaderWriterIFC()->setProgressCallBack( this, &TabReadWrite::slotProgressValueWrapper );
		m_system->getReaderWriterIFC()->setMessageCallBack( this, &TabReadWrite::slotMessageWrapper );
		m_system->getReaderWriterIFC()->setErrorCallBack( this, &TabReadWrite::slotErrorWrapper );

		cmd_load->setFilePath( path_in );
		m_system->getCommandManager()->executeCommand( cmd_load );
	}
	catch( IfcPPException& e )
	{
		slotTxtOutError( e.what() );
	}
	catch(std::exception& e)
	{
		slotTxtOutError( e.what() );
	}

	m_viewer->update();
	osg::BoundingSphere bs = m_system->getViewController()->getModelNode()->computeBound();

	osgViewer::View* main_view = m_viewer->getMainView();
	if( main_view )
	{
		osgGA::CameraManipulator* camera_manip = main_view->getCameraManipulator();
		Orbit3DManipulator* orbit_manip = dynamic_cast<Orbit3DManipulator*>( camera_manip );
		if( orbit_manip )
		{
			orbit_manip->zoomToBoundingSphere( bs );
		}
	}

	// TODO: adapt near/far plane according to bounding sphere

	clock_t time_diff = clock() - millisecs;
	slotTxtOut( tr("File loaded") + " (" + QString::number( int(time_diff*0.1)*0.01 ) + " sec)" );

	shared_ptr<IfcProject> project = m_system->getIfcModel()->getIfcProject();
	if( project )
	{
		std::map<int,shared_ptr<IfcObjectDefinition> > map_visited;
		QTreeWidgetItem* project_item = resolveTreeItems( project, map_visited );
		if( project_item != NULL )
		{
			m_ifc_tree_widget->blockSignals(true);
			m_ifc_tree_widget->insertTopLevelItem( 0, project_item );
			m_ifc_tree_widget->setColumnWidth( 0, 200 );
			m_ifc_tree_widget->setColumnWidth( 1, 100 );
			m_ifc_tree_widget->setColumnWidth( 2, 100 );
			m_ifc_tree_widget->blockSignals(false);
		}
	}

	shared_ptr<IfcGeometricRepresentationContext> geom_context = m_system->getIfcModel()->getIfcGeometricRepresentationContext3D();
	if( geom_context )
	{
		shared_ptr<IfcAxis2Placement> world_coordinate_system = geom_context->m_WorldCoordinateSystem;
		shared_ptr<IfcAxis2Placement3D> world_coordinate_system3d = dynamic_pointer_cast<IfcAxis2Placement3D>(world_coordinate_system);
		if( world_coordinate_system3d )
		{
			if( world_coordinate_system3d->m_RefDirection )
			{
				if( world_coordinate_system3d->m_RefDirection->m_DirectionRatios.size() > 2 )
				{
					// if z value < 0, then flip viewer
					if( world_coordinate_system3d->m_RefDirection->m_DirectionRatios[2] < 0 )
					{
						//m_viewer->getCameraManager()->setZAxisDown( true );
					}
				}
			}
		}
	}

	m_storey_widget->update();

	slotProgressValue( 1.0 );
}

void TabReadWrite::slotTxtOut( QString txt )
{
	m_txt_out->append( "<div style=\"color:black;\">" + txt.replace( "\n", "<br/>" ) + "</div><br/>" );
}

void TabReadWrite::slotTxtOutWarning( QString txt )
{
	m_txt_out->append( "<div style=\"color:#a97878;\">Warning: " + txt.replace( "\n", "<br/>" ) + "</div><br/>" );
}
void TabReadWrite::slotTxtOutError( QString txt )
{
	m_txt_out->append( "<div style=\"color:red;\">Error: " + txt.replace( "\n", "<br/>" ) + "</div><br/>" );
}

void TabReadWrite::slotProgressValue( double progress )
{
	m_progress_bar->setValue( (int)(progress*1000) );
	QApplication::processEvents();
}

void TabReadWrite::slotLoadRecentIfcFileClicked()
{
	QPushButton* btn_load = (QPushButton*)sender();
	if( !btn_load )
	{
		return;
	}
	m_io_widget->setDisabled(true);

	int row = m_combo_recent_files->currentIndex();
	
	if( row < 0 || row >= m_combo_recent_files->count() )
	{
		return;
	}
	
	if( row < m_recent_files.size() )
	{
		std::string file_name = m_recent_files.at( row ).toStdString();
		slotLoadIfcFile( file_name );
	}
	m_io_widget->setDisabled(false);
}

void TabReadWrite::slotClearRecentIfcFiles()
{
	m_recent_files.clear();
	QSettings settings(QSettings::UserScope, QLatin1String("IfcPlusPlus"));
	settings.setValue("recentFiles",m_recent_files );
	updateRecentFilesCombo();
}

void TabReadWrite::slotAddOtherIfcFileClicked()
{
	MyFileDialog dialog( this, "Choose IFC file" );
	
	QStringList fileNames;
	if( dialog.exec() )
	{
		fileNames = dialog.selectedFiles();
	}

	if( fileNames.size() > 0 )
	{
		QString path_in = fileNames[0];
		std::string path_in_std = path_in.toStdString();
		slotLoadIfcFile( path_in_std );
	}	
}

void TabReadWrite::slotSetWritePathClicked()
{
	QString selectedFilter;
	QString fileName = QFileDialog::getSaveFileName(this,
		"IfcPlusPlus - choose path to write ifc file",
		m_le_path_write->text(),
		"All Files (*);;Text Files (*.ifc)",
		&selectedFilter);
	if( !fileName.isEmpty() )
	{
		m_le_path_write->setText(fileName);
	}
}

void TabReadWrite::slotWriteFileClicked()
{
	QString path = m_le_path_write->text();
	QSettings settings(QSettings::UserScope, QLatin1String("IfcPlusPlus"));
	settings.setValue("pathIfcFileOut", path );

	slotTxtOut( "writing file: " + path );
	int millisecs = clock();

	std::string path_std = path.toStdString();
	try
	{
		shared_ptr<CmdWriteIfcFile> cmd_write( new CmdWriteIfcFile( m_system ) );
		cmd_write->setFilePath( path_std );
		m_system->getCommandManager()->executeCommand( cmd_write );
		//m_system->writeIfcFile( path_std );
	}
	catch( std::exception& e )
	{
		slotTxtOutWarning( "couldn't write file " + path + e.what() );
	}
	
	int time_diff = clock() - millisecs;
	slotTxtOut( "file written (" + QString::number( time_diff*0.001 ) + " sec)" );
	slotProgressValue( 1.0 );
}

QTreeWidgetItem* findItemByIfcId( QTreeWidgetItem* item, int ifc_id )
{
	int num_children = item->childCount();
	for( int i=0; i<num_children; ++i )
	{
		QTreeWidgetItem* child = item->child( i );
		int id = child->text(1).toUInt();
		if( id == ifc_id )
		{
			return child;
		}
		QTreeWidgetItem* child_of_child = findItemByIfcId( child, ifc_id );
		if( child_of_child != 0 )
		{
			return child_of_child;
		}
	}
	return 0;
}

void TabReadWrite::slotObjectsSelected( std::map<int, shared_ptr<IfcPPEntity> >& map )
{
	if( m_block_selection_signals )
	{
		return;
	}

	if( map.size() < 1 )
	{
		return;
	}

	// take the first object from map and highlight it
	shared_ptr<IfcPPEntity>& object = (*(map.begin())).second;
	int selected_id = object->getId();

	for( int i=0; i<m_ifc_tree_widget->topLevelItemCount(); ++i )
	{
		QTreeWidgetItem* toplevel_item = m_ifc_tree_widget->topLevelItem( i );
		QTreeWidgetItem* selected_item = findItemByIfcId( toplevel_item, selected_id );
		if( selected_item != 0 )
		{
			m_ifc_tree_widget->blockSignals(true);
			m_ifc_tree_widget->setCurrentItem( selected_item, 1, QItemSelectionModel::SelectCurrent );
			m_ifc_tree_widget->blockSignals(false);
			break;
		}
	}
}

void TabReadWrite::slotTreewidgetSelectionChanged( QTreeWidgetItem* current, QTreeWidgetItem* previous )
{
	const std::map<int,shared_ptr<IfcPPEntity> >& map_ifc_objects = m_system->getIfcModel()->getMapIfcObjects();
	std::map<int,shared_ptr<IfcPPEntity> >::const_iterator it_find;
	if( previous )
	{
		int id = previous->text(1).toUInt();
		it_find = map_ifc_objects.find(id);
		if( it_find != map_ifc_objects.end() )
		{
			shared_ptr<IfcPPEntity> ifc_object = it_find->second;
			//const shared_ptr<IfcPPEntity> ifc_object = map_ifc_objects[id];
			m_block_selection_signals = true;
			m_system->setObjectSelected( ifc_object, false );
			m_block_selection_signals = false;
		}
	}

	if( current )
	{
		int id = current->text(1).toUInt();
		it_find = map_ifc_objects.find(id);
		if( it_find != map_ifc_objects.end() )
		{
			shared_ptr<IfcPPEntity> ifc_object = it_find->second;
			m_block_selection_signals = true;
			m_system->setObjectSelected( ifc_object, true );
			m_block_selection_signals = false;
		}
	}
}

void TabReadWrite::slotTreewidgetSelectionChanged()
{

}
