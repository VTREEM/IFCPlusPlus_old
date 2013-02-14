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
#include <time.h>
#include "ifcpp/IFC2X4/include/IfcLabel.h"
#include "ifcpp/IFC2X4/include/IfcIdentifier.h"
#include "ifcpp/IFC2X4/include/IfcUnitEnum.h"
#include "ifcpp/IFC2X4/include/IfcSIUnitName.h"
#include "ifcpp/IFC2X4/include/IfcLengthMeasure.h"
#include "ifcpp/IFC2X4/include/IfcChangeActionEnum.h"
#include "ifcpp/IFC2X4/include/IfcValue.h"
#include "ifcpp/IFC2X4/include/IfcPlaneAngleMeasure.h"
#include "ifcpp/IFC2X4/include/IfcGloballyUniqueId.h"
#include "ifcpp/IFC2X4/include/IfcDimensionCount.h"
#include "ifcpp/IFC2X4/include/IfcPerson.h"
#include "ifcpp/IFC2X4/include/IfcOrganization.h"
#include "ifcpp/IFC2X4/include/IfcPersonAndOrganization.h"
#include "ifcpp/IFC2X4/include/IfcApplication.h"
#include "ifcpp/IFC2X4/include/IfcSIUnit.h"
#include "ifcpp/IFC2X4/include/IfcCartesianPoint.h"
#include "ifcpp/IFC2X4/include/IfcAxis2Placement3D.h"
#include "ifcpp/IFC2X4/include/IfcOwnerHistory.h"
#include "ifcpp/IFC2X4/include/IfcDimensionalExponents.h"
#include "ifcpp/IFC2X4/include/IfcMeasureWithUnit.h"
#include "ifcpp/IFC2X4/include/IfcConversionBasedUnit.h"
#include "ifcpp/IFC2X4/include/IfcUnitAssignment.h"
#include "ifcpp/IFC2X4/include/IfcProject.h"
#include "ifcpp/IFC2X4/include/IfcGeometricRepresentationContext.h"
#include "ifcpp/IFC2X4/include/IfcProduct.h"
#include "ifcpp/IFC2X4/include/IfcDirection.h"

#include "ifcpp/guid/CreateGuid_64.h"
#include "UnitConverter.h"
#include "IfcPPException.h"
#include "IfcPPModel.h"

IfcPPModel::IfcPPModel()
{
	m_unit_converter = shared_ptr<UnitConverter>( new UnitConverter() );
	initFileHeader( "IfcPlusPlus-export.ifc" );
}

IfcPPModel::~IfcPPModel()
{
}

void IfcPPModel::initIfcModel()
{
	clearIfcModel();

	shared_ptr<IfcProject> project( new IfcProject( 1 ) );
	insertEntity(project);

	shared_ptr<IfcPerson> person( new IfcPerson() );
	person->m_FamilyName = shared_ptr<IfcLabel>( new IfcLabel( "FamilyName" ) );
	person->m_GivenName = shared_ptr<IfcLabel>( new IfcLabel( "GivenName" ) );
	insertEntity(person);
	
	shared_ptr<IfcOrganization> org( new IfcOrganization() );
	org->m_Name = shared_ptr<IfcLabel>(new IfcLabel( "OrganizationName" ));
	insertEntity(org);

	shared_ptr<IfcPersonAndOrganization> person_org( new IfcPersonAndOrganization() );
	person_org->m_ThePerson = person;
	person_org->m_TheOrganization = org;
	insertEntity(person_org);

	shared_ptr<IfcApplication> app( new IfcApplication() );
	app->m_ApplicationDeveloper = org;
	app->m_Version = shared_ptr<IfcLabel>( new IfcLabel( "1.0" ) );
	app->m_ApplicationFullName = shared_ptr<IfcLabel>( new IfcLabel( "IfcPlusPlus" ) );
	app->m_ApplicationIdentifier = shared_ptr<IfcIdentifier>( new IfcIdentifier( "IfcPlusPlus" ) );
	insertEntity(app);


	shared_ptr<IfcCartesianPoint> point( new IfcCartesianPoint() );
	point->m_Coordinates.push_back( shared_ptr<IfcLengthMeasure>( new IfcLengthMeasure( 0.0 ) ) );
	point->m_Coordinates.push_back( shared_ptr<IfcLengthMeasure>( new IfcLengthMeasure( 0.0 ) ) );
	point->m_Coordinates.push_back( shared_ptr<IfcLengthMeasure>( new IfcLengthMeasure( 0.0 ) ) );
	insertEntity(point);

	shared_ptr<IfcAxis2Placement3D> axis_placement( new IfcAxis2Placement3D() );
	axis_placement->m_Location = point;
	insertEntity(axis_placement);

	shared_ptr<IfcOwnerHistory> owner_history ( new IfcOwnerHistory() );
	owner_history->m_OwningUser = person_org;
	owner_history->m_OwningApplication = app;
	owner_history->m_ChangeAction = shared_ptr<IfcChangeActionEnum>( new IfcChangeActionEnum( IfcChangeActionEnum::ENUM_ADDED ) );
	insertEntity(owner_history);

	shared_ptr<IfcDimensionalExponents> dim_exp( new IfcDimensionalExponents() );
	dim_exp->m_LengthExponent = 0;
	dim_exp->m_MassExponent = 0;
	dim_exp->m_TimeExponent = 0;
	dim_exp->m_ElectricCurrentExponent = 0;
	dim_exp->m_ThermodynamicTemperatureExponent = 0;
	dim_exp->m_AmountOfSubstanceExponent = 0;
	dim_exp->m_LuminousIntensityExponent = 0;
	insertEntity(dim_exp);

	// length unit [m]
	shared_ptr<IfcSIUnit> si_unit( new IfcSIUnit() );
	si_unit->m_UnitType = shared_ptr<IfcUnitEnum>( new IfcUnitEnum( IfcUnitEnum::ENUM_LENGTHUNIT ) );
	si_unit->m_Name = shared_ptr<IfcSIUnitName>( new IfcSIUnitName( IfcSIUnitName::ENUM_METRE ) );
	insertEntity(si_unit);

	// plane unit [rad]
	shared_ptr<IfcSIUnit> plane_angle_unit( new IfcSIUnit() );
	plane_angle_unit->m_UnitType = shared_ptr<IfcUnitEnum>( new IfcUnitEnum( IfcUnitEnum::ENUM_PLANEANGLEUNIT ) );
	plane_angle_unit->m_Name = shared_ptr<IfcSIUnitName>( new IfcSIUnitName( IfcSIUnitName::ENUM_RADIAN ) );
	insertEntity(plane_angle_unit);

	// assign units
	shared_ptr<IfcUnitAssignment> unit_assignment( new IfcUnitAssignment() );
	unit_assignment->m_Units.push_back( si_unit );
	unit_assignment->m_Units.push_back( plane_angle_unit );
	insertEntity(unit_assignment);

	
	project->m_GlobalId = shared_ptr<IfcGloballyUniqueId>(new IfcGloballyUniqueId( CreateCompressedGuidString22() ) );
	project->m_OwnerHistory = owner_history;
	project->m_Name = shared_ptr<IfcLabel>(new IfcLabel( "IfcPlusPlus project" ) );
	project->m_UnitsInContext = unit_assignment;

	// set up world coordinate system
	shared_ptr<IfcDirection> axis( new IfcDirection() );
	insertEntity(axis);
	axis->m_DirectionRatios.push_back( 0.0 );
	axis->m_DirectionRatios.push_back( 0.0 );
	axis->m_DirectionRatios.push_back( 1.0 );

	shared_ptr<IfcDirection> ref_direction( new IfcDirection() );
	insertEntity(ref_direction);
	ref_direction->m_DirectionRatios.push_back( 0.0 );
	ref_direction->m_DirectionRatios.push_back( 0.0 );
	ref_direction->m_DirectionRatios.push_back( 1.0 );

	shared_ptr<IfcCartesianPoint> location( new IfcCartesianPoint() );
	insertEntity(location);
	location->m_Coordinates.push_back( shared_ptr<IfcLengthMeasure>(new IfcLengthMeasure(0.0) ) );
	location->m_Coordinates.push_back( shared_ptr<IfcLengthMeasure>(new IfcLengthMeasure(0.0) ) );
	location->m_Coordinates.push_back( shared_ptr<IfcLengthMeasure>(new IfcLengthMeasure(0.0) ) );

	shared_ptr<IfcAxis2Placement3D> world_coordinate_system( new IfcAxis2Placement3D() );
	insertEntity(world_coordinate_system);
	world_coordinate_system->m_Location = location;
	world_coordinate_system->m_Axis = axis;
	world_coordinate_system->m_RefDirection = ref_direction;

	// 3d representation context
	shared_ptr<IfcGeometricRepresentationContext> geom_context( new IfcGeometricRepresentationContext() );
	insertEntity(geom_context);
	geom_context->m_CoordinateSpaceDimension = shared_ptr<IfcDimensionCount>(new IfcDimensionCount( 3 ) );
	geom_context->m_WorldCoordinateSystem = world_coordinate_system;

	updateCache();
}

shared_ptr<IfcProject> IfcPPModel::getIfcProject()
{
	return m_ifc_project;
}
shared_ptr<IfcGeometricRepresentationContext> IfcPPModel::getIfcGeometricRepresentationContext3D()
{
	return m_geom_context_3d;
}

void IfcPPModel::setIfcProject( shared_ptr<IfcProject> project )
{
	m_ifc_project = project;
}

void IfcPPModel::insertEntity( shared_ptr<IfcPPEntity> e, bool overwrite_existing )
{
	int entity_id = e->getId();
	if( entity_id < 0 )
	{
		int next_unused_id = getMaxUsedEntityId() + 1;
		e->setId( next_unused_id );
		entity_id = next_unused_id;
	}

	std::map<int,shared_ptr<IfcPPEntity> >::iterator lb = m_map_entities.lower_bound( entity_id );

	if( lb != m_map_entities.end() && !(m_map_entities.key_comp()(entity_id, lb->first)) )
	{
		// key already exists
		if( overwrite_existing )
		{
			(*lb).second = e;
		}
		else
		{
			std::stringstream strs;
			strs << "IfcPPModel::insertEntity: entity with id " << entity_id << " already exists in model" << std::endl;
			throw IfcPPException( strs.str().c_str() );
		}
	}
	else
	{
		// the key does not exist in the map
		m_map_entities.insert( lb, std::map<int,shared_ptr<IfcPPEntity> >::value_type( entity_id, e ) );
	}

	// TODO: if type is IfcRoot (or subtype), and GlobalID not set, create one
}

void IfcPPModel::removeEntity( shared_ptr<IfcPPEntity> e )
{
	int remove_id = e->getId();
	std::map<int,shared_ptr<IfcPPEntity> >::iterator it_find = m_map_entities.find(remove_id);
	if( it_find != m_map_entities.end() )
	{
		shared_ptr<IfcPPEntity> entity_found( it_find->second );
		entity_found->unlinkSelf();
		m_map_entities.erase( it_find );
	}
}

void IfcPPModel::removeEntity( int entity_id )
{
	std::map<int,shared_ptr<IfcPPEntity> >::iterator it_find = m_map_entities.find(entity_id);
	if( it_find != m_map_entities.end() )
	{
		shared_ptr<IfcPPEntity> entity_found( it_find->second );
		entity_found->unlinkSelf();
		m_map_entities.erase( it_find );
	}
}

int IfcPPModel::getMaxUsedEntityId()
{
	if( m_map_entities.size() == 0 )
	{
		return 0;
	}

	std::map<int,shared_ptr<IfcPPEntity> >::iterator it_entities = m_map_entities.end();
	--it_entities;
	int max_used_id = it_entities->first;
	return max_used_id;
}


void IfcPPModel::initFileHeader( std::string file_name )
{
	std::stringstream strs;
	strs << "ISO-10303-21;" << std::endl;
	strs << "HEADER;" << std::endl;
	strs << "FILE_DESCRIPTION(('IFC4'),'2;1');" << std::endl;
	strs << "FILE_NAME('" << file_name.c_str() << "','";
	
	//2011-04-21T14:25:12
	time_t rawtime;
	struct tm timeinfo;
	time( &rawtime );

#ifdef _WIN32
	errno_t err_time;
	err_time = localtime_s( &timeinfo, &rawtime );
#else
	timeinfo = localtime( &rawtime );
#endif

	char buffer [80];
	strftime(buffer,80,"%Y-%m-%dT%H:%M:%S", &timeinfo);
	strs << buffer;

	strs << "',(''),('',''),'','IfcPlusPlus','');" << std::endl;
	strs << "FILE_SCHEMA(('IFC4'));" << std::endl;
	strs << "ENDSEC;" << std::endl;

	m_file_header = strs.str().c_str();
}

void IfcPPModel::setFileHeader( std::string header )
{
	m_file_header = header;
}

void IfcPPModel::setFileSchema( std::string schema )
{
	m_file_schema = schema;
}

void IfcPPModel::clearIfcModel()
{
	m_ifc_project.reset();
	m_geom_context_3d.reset();

	std::map<int,shared_ptr<IfcPPEntity> >::iterator it_entities;
	for( it_entities = m_map_entities.begin(); it_entities != m_map_entities.end(); ++it_entities )
	{
		shared_ptr<IfcPPEntity> entity( it_entities->second );
		entity->unlinkSelf();
	}
	m_map_entities.clear();
}

void IfcPPModel::resetIfcModel()
{
	clearIfcModel();
	initIfcModel();
	updateCache();
}

void IfcPPModel::updateCache()
{
	bool found_project = false;
	bool found_geom_context = false;

	// try to find IfcProject and IfcGeometricRepresentationContext
	std::map<int,shared_ptr<IfcPPEntity> >::iterator it;
	std::map<int,shared_ptr<IfcPPEntity> >::iterator it_end = m_map_entities.end();
	for( it=m_map_entities.begin(); it!=it_end; ++it )
	{
		shared_ptr<IfcPPEntity> obj = it->second;
		if( dynamic_pointer_cast<IfcProject>(obj) )
		{
			m_ifc_project = dynamic_pointer_cast<IfcProject>( obj );
			found_project = true;
			if( found_geom_context )
			{
				break;
			}
		}
		else if( dynamic_pointer_cast<IfcGeometricRepresentationContext>(obj) )
		{
			shared_ptr<IfcGeometricRepresentationContext> context = dynamic_pointer_cast<IfcGeometricRepresentationContext>( obj );
			if( context->m_CoordinateSpaceDimension )
			{
				if( context->m_CoordinateSpaceDimension->m_value == 3 )
				{
					m_geom_context_3d = context;
					found_geom_context = true;
					if( found_project )
					{
						break;
					}
				}
			}
		}
	}

	if( found_project )
	{
		m_unit_converter->setIfcProject( m_ifc_project );
	}
}

void IfcPPModel::clearCache()
{
	m_ifc_project.reset();
	m_geom_context_3d.reset();
}

void IfcPPModel::resolveInverseAttributes()
{
	std::vector<shared_ptr<IfcObjectDefinition> >::iterator it_object_def;
	std::vector<shared_ptr<IfcProduct> >::iterator it_product;
	std::map<int,shared_ptr<IfcPPEntity> >::iterator it;

	for( it=m_map_entities.begin(); it!=m_map_entities.end(); ++it )
	{
		shared_ptr<IfcPPEntity> obj = it->second;
		if( !obj )
		{
			continue;
		}
		obj->setInverseCounterparts( obj );
	}
}
