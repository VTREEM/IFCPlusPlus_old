#include "IfcPPAttributeObject.h"

IfcPPAttributeObjectBool::IfcPPAttributeObjectBool( bool attribute_value ) : m_value(attribute_value)
{
}
IfcPPAttributeObjectLogical::IfcPPAttributeObjectLogical( LogicalEnum attribute_value ) : m_value(attribute_value)
{
}

IfcPPAttributeObjectInt::IfcPPAttributeObjectInt( int attribute_value ) : m_value(attribute_value)
{
}

IfcPPAttributeObjectDouble::IfcPPAttributeObjectDouble( double attribute_value ) : m_value(attribute_value)
{
}

IfcPPAttributeObjectString::IfcPPAttributeObjectString( std::string& attribute_value ) : m_value(attribute_value)
{
}

IfcPPAttributeObjectVector::IfcPPAttributeObjectVector()
{
}

IfcPPAttributeObjectVector::IfcPPAttributeObjectVector( std::vector<shared_ptr<IfcPPObject> > vec )
{
	std::copy( vec.begin(), vec.end(), std::back_inserter( m_vec ) );
}

IfcPPAttributeObjectVector::~IfcPPAttributeObjectVector()
{
}
