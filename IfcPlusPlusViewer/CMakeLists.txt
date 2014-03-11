cmake_minimum_required (VERSION 2.8)

IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE "Release")
ENDIF(NOT CMAKE_BUILD_TYPE)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/${CMAKE_BUILD_TYPE})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/${CMAKE_BUILD_TYPE})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/${CMAKE_BUILD_TYPE})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/${CMAKE_BUILD_TYPE})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/${CMAKE_BUILD_TYPE})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/${CMAKE_BUILD_TYPE})

find_package(Qt5Core REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5OpenGL REQUIRED)
include_directories(${Qt5Widgets_INCLUDES})
include_directories(${Qt5OpenGL_INCLUDES})
add_definitions(${Qt5Widgets_DEFINITIONS})
set(CMAKE_CXX_FLAGS "${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS}")

INCLUDE_DIRECTORIES(${INCLUDE_DIRECTORIES} 
        ${QT_INCLUDES}
        ${QT_INCLUDE_DIR}
        ${QT_QT_INCLUDE_DIR}
        ${QT_QTCORE_INCLUDE_DIR}
        ${QT_QTGUI_INCLUDE_DIR}
        ${QT_QTOPENGL_INCLUDE_DIR}
)

FIND_PACKAGE(OpenSceneGraph REQUIRED osgDB osgUtil osgGA osgFX osgSim osgText osgQt osgViewer)


INCLUDE_DIRECTORIES(${INCLUDE_DIRECTORIES} ${CARVE_SOURCE_DIR}/include) # when building with the external dependency
INCLUDE_DIRECTORIES(${INCLUDE_DIRECTORIES} ${CARVE_SOURCE_DIR}/src/include) # When building with the embedded Carve
INCLUDE_DIRECTORIES(${INCLUDE_DIRECTORIES} ${CARVE_BINARY_DIR}/include)

INCLUDE_DIRECTORIES(${INCLUDE_DIRECTORIES} ${IFCPP_SOURCE_DIR}/IfcPlusPlus/src/)
INCLUDE_DIRECTORIES(${INCLUDE_DIRECTORIES} ${IFCPP_SOURCE_DIR}/IfcPlusPlusGeometry/src/)
INCLUDE_DIRECTORIES(${INCLUDE_DIRECTORIES} ${IFCPP_SOURCE_DIR}/IfcPlusPlusViewer/src/)
INCLUDE_DIRECTORIES(${INCLUDE_DIRECTORIES} ${IFCPP_SOURCE_DIR}/IfcPlusPlusViewer/src/ifcplusplusviewer)

INCLUDE_DIRECTORIES( ${INCLUDE_DIRECTORIES} ${OPENTHREADS_INCLUDE_DIR} )
INCLUDE_DIRECTORIES( ${INCLUDE_DIRECTORIES} ${OSG_INCLUDE_DIR} )
INCLUDE_DIRECTORIES( ${INCLUDE_DIRECTORIES} ${OSGGA_INCLUDE_DIR} )
INCLUDE_DIRECTORIES( ${INCLUDE_DIRECTORIES} ${OSGDB_INCLUDE_DIR} )
INCLUDE_DIRECTORIES( ${INCLUDE_DIRECTORIES} ${OSGUTIL_INCLUDE_DIR} )
INCLUDE_DIRECTORIES( ${INCLUDE_DIRECTORIES} ${OSGQT_INCLUDE_DIR} )
INCLUDE_DIRECTORIES( ${INCLUDE_DIRECTORIES} ${OSGANIMATION_INCLUDE_DIR} )
INCLUDE_DIRECTORIES( ${INCLUDE_DIRECTORIES} ${OSGANIMATION_INCLUDE_DIR} )
INCLUDE_DIRECTORIES( ${INCLUDE_DIRECTORIES} ${OSG_BINARY_DIR}/include ) # For the generated Config file

set(IFCPPVIEWER_SOURCE_FILES 
  src/ifcplusplusviewer/IfcPlusPlusSystem.cpp
  src/ifcplusplusviewer/ViewController.cpp
  src/ifcplusplusviewer/main.cpp
  src/ifcplusplusviewer/cmd/CmdLoadIfcFile.cpp
  src/ifcplusplusviewer/cmd/CmdRemoveSelectedObjects.cpp
  src/ifcplusplusviewer/cmd/CmdWriteIfcFile.cpp
  src/ifcplusplusviewer/cmd/Command.cpp
  src/ifcplusplusviewer/cmd/CommandManager.cpp
  src/ifcplusplusviewer/gui/DebugViewer.cpp
  src/ifcplusplusviewer/gui/IfcTreeWidget.cpp
  src/ifcplusplusviewer/gui/MainWindow.cpp
  src/ifcplusplusviewer/gui/StepHighlighter.cpp
  src/ifcplusplusviewer/gui/TabReadWrite.cpp
  src/ifcplusplusviewer/gui/TabTextEdit.cpp
  src/ifcplusplusviewer/gui/TabView.cpp
  src/ifcplusplusviewer/viewer/Orbit3DManipulator.cpp
  src/ifcplusplusviewer/viewer/SubGroupIntersectionVisitor.cpp
  src/ifcplusplusviewer/viewer/ViewerUtil.cpp
  src/ifcplusplusviewer/viewer/ViewerWidget.cpp
)

set(IFCPPVIEWER_HEADER_FILES
  src/ifcplusplusviewer/IfcPlusPlusSystem.h
  src/ifcplusplusviewer/ViewController.h
  src/ifcplusplusviewer/cmd/CmdLoadIfcFile.h
  src/ifcplusplusviewer/cmd/CmdRemoveSelectedObjects.h
  src/ifcplusplusviewer/cmd/CmdWriteIfcFile.h
  src/ifcplusplusviewer/cmd/Command.h
  src/ifcplusplusviewer/cmd/CommandManager.h
  src/ifcplusplusviewer/gui/DebugViewer.h
  src/ifcplusplusviewer/gui/IfcTreeWidget.h
  src/ifcplusplusviewer/gui/MainWindow.h
  src/ifcplusplusviewer/gui/StepHighlighter.h
  src/ifcplusplusviewer/gui/TabReadWrite.h
  src/ifcplusplusviewer/gui/TabTextEdit.h
  src/ifcplusplusviewer/gui/TabView.h
  src/ifcplusplusviewer/viewer/Orbit3DManipulator.h
  src/ifcplusplusviewer/viewer/SubGroupIntersectionVisitor.h
  src/ifcplusplusviewer/viewer/ViewerUtil.h
  src/ifcplusplusviewer/viewer/ViewerWidget.h
)

find_package(Boost 1.53)
if(Boost_FOUND)
  include_directories(${Boost_INCLUDE_DIRS})
  message(STATUS "Using Boost libraries for Carve")
else(Boost_FOUND)
  message(STATUS "Could not find Boost libraries for Carve")
endif(Boost_FOUND)

QT5_WRAP_CPP( IFCPPVIEWER_MOC_GENERATED_FILES ${IFCPPVIEWER_HEADER_FILES})

LINK_DIRECTORIES (${LINK_DIRECTORIES} ${CMAKE_BINARY_DIR}/${CMAKE_BUILD_TYPE}) 

# Linking to the Carve library
LINK_DIRECTORIES (${LINK_DIRECTORIES} ${CARVE_BINARY_DIR}/lib) 
LINK_DIRECTORIES (${LINK_DIRECTORIES} ${CARVE_BINARY_DIR}/common) 

ADD_EXECUTABLE(IfcPlusPlusViewer 
  ${IFCPPVIEWER_HEADER_FILES} 
  ${IFCPPVIEWER_MOC_GENERATED_FILES}
  ${IFCPPVIEWER_SOURCE_FILES}
  ${IfcPlusPlusViewer_HEADERS_MOC} 
  ${IfcPlusPlusViewer_FORMS_HEADERS} 
  ${IfcPlusPlusViewer_RESOURCES_RCC}
)

TARGET_LINK_LIBRARIES(IfcPlusPlusViewer IfcPlusPlusGeometry)

TARGET_LINK_LIBRARIES(IfcPlusPlusViewer ${OPENTHREADS_LIBRARY} )
TARGET_LINK_LIBRARIES(IfcPlusPlusViewer ${OSG_LIBRARY} )
TARGET_LINK_LIBRARIES(IfcPlusPlusViewer ${OSGDB_LIBRARY} )
TARGET_LINK_LIBRARIES(IfcPlusPlusViewer ${OSGGA_LIBRARY} )
TARGET_LINK_LIBRARIES(IfcPlusPlusViewer ${OSGUTIL_LIBRARY} )
TARGET_LINK_LIBRARIES(IfcPlusPlusViewer ${OSGANIMATION_LIBRARY} )
TARGET_LINK_LIBRARIES(IfcPlusPlusViewer ${OSGFX_LIBRARY} )
TARGET_LINK_LIBRARIES(IfcPlusPlusViewer ${OSGQT_LIBRARY} )
TARGET_LINK_LIBRARIES(IfcPlusPlusViewer ${OSGVIEWER_LIBRARY} )

qt5_use_modules(IfcPlusPlusViewer Core Gui Widgets OpenGL)
