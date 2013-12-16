cmake_minimum_required (VERSION 2.6)

project (IFCPP)

add_subdirectory (IfcPlusPlus)
add_subdirectory (IfcPlusPlusGeometry)

if (BUILD_IFCPP_VIEWER)
    add_subdirectory (IfcPlusPlusViewer)
endif()
