cmake_minimum_required(VERSION 3.2)

project(dynamic_adjustment)

#Set the CMake Module path
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
#Release mode is light and faster.
set( CMAKE_CXX_COMPILER "g++" )
set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -march=native -O3" )


#The output path of executable files and link libraries.
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


#Dependencies.
#Eigen
INCLUDE_DIRECTORIES("/usr/include/eigen3")

#Opencv
FIND_PACKAGE( OpenCV REQUIRED )
INCLUDE_DIRECTORIES( ${OpenCV_DIRS} )

#Sophus
FIND_PACKAGE( Sophus REQUIRED )
INCLUDE_DIRECTORIES( ${Sophus_INCLUDE_DIRS} )

# G2O
FIND_PACKAGE( G2O REQUIRED )
INCLUDE_DIRECTORIES( ${G2O_INCLUDE_DIRS} )

#SuiteSparse
FIND_PACKAGE(SuiteSparse)
include_directories(${SUITESPARSE_INCLUDE_DIRS})



#Link with third party libs.
set( THIRD_PARTY_LIBS 
    ${OpenCV_LIBS}
    ${Sophus_LIBRARIES}
    g2o_core g2o_stuff g2o_types_sba
)


#The path deposit the header files.
INCLUDE_DIRECTORIES(
        ${PROJECT_SOURCE_DIR}/include
)


#Source directory.
ADD_SUBDIRECTORY( src )
ADD_SUBDIRECTORY( test )
ADD_SUBDIRECTORY( main )

