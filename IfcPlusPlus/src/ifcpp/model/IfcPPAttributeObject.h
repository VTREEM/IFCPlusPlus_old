
#pragma once
#include <string>
#include <vector>
#include "ifcpp/model/shared_ptr.h"
#include "ifcpp/model/IfcPPObject.h"

//enum IfcPPAttributeType {
//	IfcPPAttribute_bool,
//	IfcPPAttribute_int,
//	IfcPPAttribute_double,
//	IfcPPAttribute_string,
//	IfcPPAttribute_type,
//	IfcPPAttribute_entity
//};
//enum IfcPPAttributeCardinality {
//	IfcPP_cardinality_single,
//	IfcPP_cardinality_vec2d,
//	IfcPP_cardinality_vec3d
//};


class IfcPPAttributeObject : public IfcPPObject
{
public:
	IfcPPAttributeObject()
	{
	}
	//IfcPPAttributeObject( std::string& name ) : m_attribute_name( name )
	//{
	//}
	~IfcPPAttributeObject()
	{
	}
	//std::string m_attribute_name;
};


class IfcPPAttributeObjectBool : public IfcPPObject
{
public:
	IfcPPAttributeObjectBool( bool attribute_value );
	bool m_value;
};

class IfcPPAttributeObjectInt : public IfcPPObject
{
public:
	IfcPPAttributeObjectInt( int attribute_value );
	int m_value;
};

class IfcPPAttributeObjectDouble : public IfcPPObject
{
public:
	IfcPPAttributeObjectDouble( double attribute_value );
	double m_value;
};

class IfcPPAttributeObjectString : public IfcPPObject
{
public:
	IfcPPAttributeObjectString( std::string& attribute_value );
	std::string m_value;
};

class IfcPPAttributeObjectVector : public IfcPPObject
{
public:
	IfcPPAttributeObjectVector();
	IfcPPAttributeObjectVector( std::vector<shared_ptr<IfcPPObject> > vec );
	~IfcPPAttributeObjectVector();
	std::vector<shared_ptr<IfcPPObject> > m_vec;
};

//class IfcPPAttributeEntity : public IfcPPAttributeObject
//{
//public:
//	IfcPPAttributeEntity()
//	{
//	}
//	IfcPPAttributeEntity( std::string& name, shared_ptr<IfcPPEntity>& entity ) : IfcPPAttributeObject( name ), m_attribute_entity(entity)
//	{
//	}
//	~IfcPPAttributeEntity()
//	{
//	}
//	shared_ptr<IfcPPEntity> m_attribute_entity;
//};
