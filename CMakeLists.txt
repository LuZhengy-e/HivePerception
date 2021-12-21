cmake_minimum_required(VERSION 2.8)
project(HIVE_SLAM)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3")
LIST( APPEND CMAKE_MODULE_PATH /home/luzhengye/package/g2o/cmake_modules )
set(G2O_ROOT /usr/local/include/g2o)

# Check C++11 support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
if(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
  add_definitions(-DCOMPILEDWITHC11)
  message(STATUS "Using flag -std=c++11.")
else()
  message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(OpenCV QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
else()
   message(STATUS "Using Opencv ${OpenCV_VERSION}")
endif()

add_definitions(-DBOOST_ERROR_CODE_HEADOR_ONLY) ## for boosr

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(G2O REQUIRED)
if(G2O_FOUND)
  message(STATUS "Using G2O ${G2O_ROOT}")
endif()
find_package(PCL REQUIRED)
# find_package(Boost REQUIRED COMPONENTS system filesystem thread)

set(Jsoncpp_INCLUDE_PATH "/usr/include/jsoncpp")
set(Jsoncpp_LIBRARY "/usr/lib/x86_64-linux-gnu/libjsoncpp.so")

# set(PCL_INCLUDE_DIRS "/usr/include/pcl-1.7")
# set(PCL_LIBRARY_DIRS "/usr/lib/x86_64-linux-gnu/libpcl_common.so")

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
${Jsoncpp_INCLUDE_PATH}
${G2O_INCLUDE_DIRS}
${PCL_INCLUDE_DIRS}
${Boost_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
src/Optimizer.cpp
src/data.cpp
src/Element.cpp
src/KeyFrame.cpp
src/Marker.cpp
src/Pole.cpp
src/transform.cpp
)

target_link_libraries(
${PROJECT_NAME}
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${G2O_CORE_LIBRARY}
${G2O_STUFF_LIBRARY}
# g2o_core
# g2o_stuff
g2o_types_sba
${PCL_LIBRARIES}
${Boost_LIBRARIES}
)

add_executable(test test.cpp)
target_link_libraries(test ${PROJECT_NAME})

add_executable(bin2pcd bin2pcd.cpp)
target_link_libraries(bin2pcd ${PROJECT_NAME})