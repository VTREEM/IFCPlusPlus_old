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

#pragma once

#include "ifcpp/model/shared_ptr.h"

#include <QtCore/qglobal.h>
#if (QT_VERSION < QT_VERSION_CHECK(5, 0, 0))
	#include <QtGui/qwidget.h>
#else
	#include <QtWidgets/qwidget.h>
#endif

class QTextEdit;
class QProgressBar;
class QComboBox;
class QTreeWidget;
class QLineEdit;
class QPushButton;
class QSplitter;
class QTreeWidgetItem;
class StoreyWidget;
class IfcTreeWidget;

class ViewerWidget;
class IfcPPEntity;
class IfcPlusPlusSystem;


class TabReadWrite : public QWidget
{
	Q_OBJECT
public:
	TabReadWrite( IfcPlusPlusSystem* sys, ViewerWidget* viewer, QWidget* parent = 0 );
	~TabReadWrite();

	static void slotProgressValueWrapper( void* obj_ptr, double value );
	static void slotMessageWrapper( void* obj_ptr, const std::string& str );
	static void slotErrorWrapper( void* obj_ptr, const std::string& str );
	void slotProgressValue( double value );
	void closeEvent( QCloseEvent *event );

public slots:
	void slotTxtOut( QString txt );
	void slotTxtOutWarning( QString txt );
	void slotTxtOutError( QString txt );
	
	void slotLoadIfcFile( std::string& path );
	void slotRecentFilesIndexChanged(int);
	void updateRecentFilesCombo();
	void slotObjectSelected( shared_ptr<IfcPPEntity> object );
	void slotTreewidgetSelectionChanged( QTreeWidgetItem* current, QTreeWidgetItem* previous );
	void slotTreewidgetSelectionChanged();

protected:
	void keyPressEvent( QKeyEvent* event );
	

private:
	IfcPlusPlusSystem*	m_system;
	ViewerWidget*	m_viewer;
	QTextEdit*		m_txt_out;
	QProgressBar*	m_progress_bar;
	
	QComboBox*		m_combo_recent_files;
	QStringList		m_recent_files;
	QPushButton*	m_btn_load;
	QWidget*		m_io_widget;

	StoreyWidget*	m_storey_widget;
	IfcTreeWidget*	m_ifc_tree_widget;

	QLineEdit*		m_le_path_write;
	QSplitter*		m_io_splitter;
	bool			m_block_selection_signals;

private slots:
	void slotAddOtherIfcFileClicked();
	void slotLoadRecentIfcFileClicked();
	void slotSetWritePathClicked();
	void slotWriteFileClicked();
	void slotClearRecentIfcFiles();
};
