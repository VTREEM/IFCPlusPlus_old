PROJECT(IfcPlusPlusGeometry)

set(IfcPlusPlusGeometry_CXX_SRC 
	src/ifcppgeometry/ConverterOSG.cpp
	src/ifcppgeometry/Nurbs.cpp
	src/ifcppgeometry/PlacementConverter.cpp
	src/ifcppgeometry/ProfileConverter.cpp
	src/ifcppgeometry/ReaderWriterIFC.cpp
	src/ifcppgeometry/RepresentationConverter.cpp
	src/ifcppgeometry/SectionedSpine.cpp
	src/ifcppgeometry/StylesConverter.cpp
	src/ifcppgeometry/Utility.cpp)

ADD_LIBRARY(IfcPlusPlusGeometry STATIC ${IfcPlusPlusGeometry_CXX_SRC})
