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

#include <iostream>
#include <QtGui/QApplication>
#include <QSplashScreen>
#include "gui/TabReadWrite.h"
#include "gui/MainWindow.h"
#include "ifcpp/model/IfcPPException.h"
#include "ifcppgeometry/Utility.h"
#include "viewer/ViewerWidget.h"
#include "ViewController.h"
#include "IfcPlusPlusSystem.h"

class IfcPlusPlusApplication : public QApplication
{
public:
	IfcPlusPlusApplication( int &argc, char **argv ) : QApplication( argc, argv )
	{
	}
	~IfcPlusPlusApplication()
	{
	}
protected:
	virtual bool notify( QObject* receiver, QEvent* e )
	{
		bool errRet = false;
		bool tmp;
		try
		{
			tmp = QApplication::notify (receiver, e);
		}
		catch( std::exception& e )
		{
			std::cout << " * ApplicationEx::notify :" << std::endl;
			std::cout << "exception occurred : " << e.what() << std::endl;
			// TODO: write file with error report
		}
		catch( std::exception* e )
		{
			std::cout << " * ApplicationEx::notify :" << std::endl;
			std::cout << "exception occurred : " << e->what() << std::endl;
			// TODO: write file with error report
		}
		catch(...)
		{

		}
		return errRet;
	}
};

int main(int argc, char *argv[])
{
	IfcPlusPlusApplication app(argc, argv);
	QPixmap pixmap( ":img/IfcPlusPlusViewerSplash.png" );
	QSplashScreen splash( pixmap );

#ifndef _DEBUG
	splash.show();
	app.processEvents();
	QTimer::singleShot( 1500, &splash, SLOT(close()));
#endif

	IfcPlusPlusSystem* sys = new IfcPlusPlusSystem();
	ViewerWidget* viewer_widget = new ViewerWidget();
	
	viewer_widget->setRootNode( sys->getViewController()->getRootNode() );
	viewer_widget->setModelNode( sys->getViewController()->getModelNode() );

	MainWindow* window = new MainWindow( sys, viewer_widget );
	app.connect( &app,		SIGNAL(lastWindowClosed()),				&app,	SLOT(quit()) );
	window->show();
	viewer_widget->setFocus();
	viewer_widget->startTimer();
	viewer_widget->addEventHandler( sys );
	
	if( argc > 1 )
	{
		std::string arg1 = argv[1];
		
		if( arg1.length() > 4 )
		{
			std::string file_type = arg1.substr(arg1.find_last_of(".") + 1);
			std::transform(file_type.begin(), file_type.end(), file_type.begin(), toupper);

			if( file_type.compare( "IFC" ) == 0 || file_type.compare( "STP" ) == 0  )
			{
				window->getTabReadWrite()->slotLoadIfcFile( arg1 );
			}
		}
	}

	int re=0;
	try
	{
		re = app.exec();
	}
	catch( IfcPPException& e )
	{
		std::cout << "IfcPPException in app.exec(): " << e.what();
	}
#ifndef _DEBUG
	catch( std::exception& e )
	{
		std::cout << "std::exception in app.exec(): " << e.what();
	}
	catch( std::exception* e )
	{
		std::cout << "std::exception in app.exec(): " << e->what();
	}
#endif

	viewer_widget->setDone(true);
	viewer_widget->stopTimer();

	return re;
}
