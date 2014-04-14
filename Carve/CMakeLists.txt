INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/src)
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/src/include)
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/src/carve/include)

list(APPEND carve_CXX_SRC
src/common/geometry.cpp
src/lib/carve.cpp
src/lib/convex_hull.cpp
src/lib/csg.cpp
src/lib/csg_collector.cpp
src/lib/face.cpp
src/lib/geom.cpp
src/lib/geom2d.cpp
src/lib/geom3d.cpp
src/lib/intersect.cpp
src/lib/intersect_classify_edge.cpp
src/lib/intersect_classify_group.cpp
src/lib/intersect_debug.cpp
src/lib/intersect_face_division.cpp
src/lib/intersect_group.cpp
src/lib/intersect_half_classify_group.cpp
src/lib/intersection.cpp
src/lib/math.cpp
src/lib/mesh.cpp
src/lib/octree.cpp
src/lib/pointset.cpp
src/lib/polyhedron.cpp
src/lib/polyline.cpp
src/lib/tag.cpp
src/lib/timing.cpp
src/lib/triangle_intersection.cpp
src/lib/triangulator.cpp
)

# create library
ADD_LIBRARY(Carve STATIC ${carve_CXX_SRC})