This is fork from [IFCPlusPlus](https://code.google.com/p/ifcplusplus/), "First wave" (commits up to the repository mysterious reset which happened in 2014-06-01).  
The main changes are meant to make it compile with Clang using the CMake build system.  

IFCPlusPlus requires Carve, Boost, and OpenSceneGraph.  
Carve is included with this repository but compiling against the [official repository](https://github.com/VTREEM/Carve) is preferred.  
The optional viewer requires Qt.  

** Building IFCPlusPlus with Carve and Boost as external dependencies: **  

Example for OS X 10.9:    
. $YOUR_COTS_BIN_DIR: where your out-of-source builds are  
. $YOUR_COTS_SRC_DIR: where your source codes are 

    ccmake -D CMAKE_BUILD_TYPE=Release \
           -D CMAKE_OSX_ARCHITECTURES:STRING="x86_64" \
           -D CMAKE_OSX_SYSROOT:STRING="/Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX10.9.sdk/" \
           -D CMAKE_INSTALL_PREFIX:STRING="$YOUR_COTS_BIN_DIR/OpenSceneGraph/build-OSX;$YOUR_COTS_SRC_DIR/OpenSceneGraph"  \
           -D OSG_BINARY_DIR:STRING=$YOUR_COTS_BIN_DIR/OpenSceneGraph/build-OSX \
           -D CARVE_SOURCE_DIR:STRING=$YOUR_COTS_SRC_DIR/Carve \
           -D CARVE_BINARY_DIR:STRING=$YOUR_COTS_BIN_DIR/Carve/build-OSX \
           -D DESIRED_QT_VERSION:STRING="4" \
           -D BUILD_IFCPP_VIEWER:BOOL=OFF \
           -D CARVE_SYSTEM_BOOST:BOOL=TRUE \
           -D CARVE_BOOST_COLLECTIONS:BOOL=TRUE \
           -D Boost_DIR:STRING="$YOUR_COTS_BIN_DIR/boost_1_53_0" \
           -D BOOST_INCLUDEDIR:STRING="$YOUR_COTS_SRC_DIR/boost_1_53_0" \
           $YOUR_COTS_SRC_DIR/ifcplusplus
    (C)configure, (G)enerate  
    make -j4  

    