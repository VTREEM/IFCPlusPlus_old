/* -*-c++-*- IfcPlusPlus - www.ifcplusplus.com - Copyright (C) 2011 Fabian Gerold
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
#include <sstream>
#include <limits>

#include "ifcpp/model/IfcPPException.h"
#include "ifcpp/model/IfcPPAttributeObject.h"
#include "ifcpp/reader/ReaderUtil.h"
#include "ifcpp/writer/WriterUtil.h"
#include "ifcpp/IfcPPEntityEnums.h"
#include "include/IfcTable.h"
#include "include/IfcTableRow.h"
#include "include/IfcValue.h"

// ENTITY IfcTableRow 
IfcTableRow::IfcTableRow() {}
IfcTableRow::IfcTableRow( int id ) { m_id = id; }
IfcTableRow::~IfcTableRow() {}

// method setEntity takes over all attributes from another instance of the class
void IfcTableRow::setEntity( shared_ptr<IfcPPEntity> other_entity )
{
	shared_ptr<IfcTableRow> other = dynamic_pointer_cast<IfcTableRow>(other_entity);
	if( !other) { return; }
	m_RowCells = other->m_RowCells;
	m_IsHeading = other->m_IsHeading;
}
void IfcTableRow::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_id << "=IFCTABLEROW" << "(";
	writeTypeList( stream, m_RowCells, true );
	stream << ",";
	if( m_IsHeading == false ) { stream << ".F."; }
	else if( m_IsHeading == true ) { stream << ".T."; }
	stream << ");";
}
void IfcTableRow::getStepParameter( std::stringstream& stream, bool ) const { stream << "#" << m_id; }
void IfcTableRow::readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<IfcPPEntity> >& map )
{
	const int num_args = (int)args.size();
	if( num_args<2 ){ std::stringstream strserr; strserr << "Wrong parameter count for entity IfcTableRow, expecting 2, having " << num_args << ". Object id: " << getId() << std::endl; throw IfcPPException( strserr.str().c_str() ); }
	#ifdef _DEBUG
	if( num_args>2 ){ std::cout << "Wrong parameter count for entity IfcTableRow, expecting 2, having " << num_args << ". Object id: " << getId() << std::endl; }
	#endif
	readSelectList( args[0], m_RowCells, map );
	if( _stricmp( args[1].c_str(), ".F." ) == 0 ) { m_IsHeading = false; }
	else if( _stricmp( args[1].c_str(), ".T." ) == 0 ) { m_IsHeading = true; }
}
void IfcTableRow::getAttributes( std::vector<std::pair<std::string, shared_ptr<IfcPPObject> > >& vec_attributes )
{
	shared_ptr<IfcPPAttributeObjectVector> RowCells_vec_object( new  IfcPPAttributeObjectVector() );
	std::copy( m_RowCells.begin(), m_RowCells.end(), std::back_inserter( RowCells_vec_object->m_vec ) );
	vec_attributes.push_back( std::make_pair( "RowCells", RowCells_vec_object ) );
	vec_attributes.push_back( std::make_pair( "IsHeading", shared_ptr<IfcPPAttributeObjectBool>( new  IfcPPAttributeObjectBool( m_IsHeading ) ) ) );
}
void IfcTableRow::getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<IfcPPObject> > >& vec_attributes )
{
	vec_attributes.push_back( std::make_pair( "OfTable_inverse", shared_ptr<IfcPPEntity>( m_OfTable_inverse ) ) );
}
void IfcTableRow::setInverseCounterparts( shared_ptr<IfcPPEntity> )
{
}
void IfcTableRow::unlinkSelf()
{
}
