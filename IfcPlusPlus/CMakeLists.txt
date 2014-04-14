PROJECT(IfcPP)

file(GLOB IfcPP_Source 					src/ifcpp/*.*)
file(GLOB IfcPP_guid_Source 			src/ifcpp/guid/*.*)
file(GLOB IfcPP_IFC2X4_Source 			src/ifcpp/IFC2X4/*.*)
#file(GLOB IfcPP_IFC2X4_include_Source 	src/ifcpp/IFC2X4/include/*.*)
file(GLOB IfcPP_model_Source 			src/ifcpp/model/*.*)
file(GLOB IfcPP_reader_Source 			src/ifcpp/reader/*.*)
file(GLOB IfcPP_writer_Source 			src/ifcpp/writer/*.*)
file(GLOB IfcPP_xml_Source 				src/ifcpp/xml/*.*)

set(IfcPlusPlus_CXX_SRC ${IfcPP_Source} ${IfcPP_guid_Source} ${IfcPP_IFC2X4_Source} ${IfcPP_model_Source} ${IfcPP_reader_Source} ${IfcPP_writer_Source} ${IfcPP_xml_Source} )

ADD_LIBRARY(IfcPlusPlus STATIC ${IfcPlusPlus_CXX_SRC})
