
#pragma once
#include <string>
#include <vector>
#include "ifcpp/model/shared_ptr.h"
#include "ifcpp/model/IfcPPObject.h"

class IfcPPAttributeObject : public IfcPPObject
{
public:
	IfcPPAttributeObject()
	{
	}
	~IfcPPAttributeObject()
	{
	}
};

class IfcPPAttributeObjectBool : public IfcPPObject
{
public:
	IfcPPAttributeObjectBool( bool attribute_value );
	bool m_value;
};

class IfcPPAttributeObjectLogical : public IfcPPObject
{
public:
	IfcPPAttributeObjectLogical( LogicalEnum attribute_value );
	LogicalEnum m_value;
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
