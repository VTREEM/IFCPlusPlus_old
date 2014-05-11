PROJECT(IfcPlusPlusGeometry)

set(IfcPlusPlusGeometry_CXX_SRC 
	src/ifcppgeometry/ConverterOSG.cpp
	src/ifcppgeometry/CSG_Adapter.cpp
	src/ifcppgeometry/CurveConverter.cpp
	src/ifcppgeometry/DebugViewerCallback.cpp
	src/ifcppgeometry/FaceConverter.cpp
	src/ifcppgeometry/GeometryInputData.cpp
	src/ifcppgeometry/GeometryInputData.cpp
	src/ifcppgeometry/GeometrySettings.cpp
	src/ifcppgeometry/GeomUtils.cpp
	src/ifcppgeometry/PlacementConverter.cpp
	src/ifcppgeometry/ProfileCache.cpp
	src/ifcppgeometry/ProfileConverter.cpp
	src/ifcppgeometry/ReaderWriterIFC.cpp
	src/ifcppgeometry/RepresentationConverter.cpp
	src/ifcppgeometry/SectionedSpine.cpp
	src/ifcppgeometry/SolidModelConverter.cpp
	src/ifcppgeometry/StylesConverter.cpp

ADD_LIBRARY(IfcPlusPlusGeometry STATIC ${IfcPlusPlusGeometry_CXX_SRC})
