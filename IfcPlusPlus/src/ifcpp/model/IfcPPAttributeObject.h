
#pragma once
#include <string>
#include <vector>
#include "ifcpp/model/shared_ptr.h"

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

class IfcPPAttributeObject
{
public:
	IfcPPAttributeObject( std::string& attribute_name )
	{
	}

	std::string m_attribute_name;
	//IfcPPAttributeType m_type;
	//IfcPPAttributeCardinality m_cardinality;
};


class IfcPPAttributeObjectBool : public IfcPPAttributeObject
{
public:
	IfcPPAttributeObjectBool( std::string& attribute_name, bool attribute_value ) : m_attribute_name( attribute_name ), m_value(attribute_value)
	{
	}
	bool m_value;
};

class IfcPPAttributeObjectVector : public IfcPPAttributeObject
{
public:
	IfcPPAttributeObjectVector( std::string& attribute_name, bool attribute_value )
	{
	}

	std::vector<shared_ptr<IfcPPAttributeObject> > m_vec;
	
};

