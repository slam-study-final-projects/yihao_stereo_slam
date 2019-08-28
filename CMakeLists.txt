cmake_minimum_required( VERSION 2.8 )

project(final_project)
# set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_BUILD_TYPE "RelWithDebInfo")
# set( CMAKE_CXX_FLAGS "-O3" )
set( CMAKE_CXX_FLAGS "-O1" )

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
include_directories("/usr/include/eigen3")
find_package(OpenCV REQUIRED)
find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS} ${Sophus_INCLUDE_DIRS} ${Pangolin_INCLUDE_DIRS})

Find_Package(G2O REQUIRED)
Find_Package(Eigen3 REQUIRED)
Find_Package(Cholmod REQUIRED)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

SET(G2O_LIBS g2o_cli g2o_ext_freeglut_minimal g2o_simulator g2o_solver_slam2d_linear 
    g2o_types_icp g2o_types_slam2d g2o_types_sba g2o_types_slam3d g2o_core g2o_interface 
    g2o_solver_csparse g2o_solver_structure_only g2o_csparse_extension g2o_opengl_helper g2o_solver_dense 
    g2o_stuff g2o_types_sclam2d g2o_parser g2o_solver_pcg g2o_types_data g2o_types_sim3 cxsparse)

include_directories(${EIGEN3_INCLUDE_DIR} ${CHOLMOD_INCLUDE_DIR}) 

add_library(loader STATIC
  loader.h
  loader.cc
)

target_link_libraries(loader
  ${OpenCV_LIBS}
)

add_library(depth_generator STATIC
  depth_generator.h
  depth_generator.cc
)

target_link_libraries(depth_generator
  ${OpenCV_LIBS}
)

add_library(orb_matcher STATIC
  orb_matcher.h
  orb_matcher.cc
)

target_link_libraries(orb_matcher
  ${OpenCV_LIBS}
)

add_library(icp_translator STATIC
  icp_translator.h
  icp_translator.cc
)

target_link_libraries(icp_translator
  ${OpenCV_LIBS}
)

add_library(slam_work STATIC
  slam_work.h
  slam_work.cc
)

target_link_libraries(slam_work
  loader depth_generator orb_matcher icp_translator 
  ${OpenCV_LIBS}
)

add_executable(final_project final_project.cpp)
target_link_libraries(final_project slam_work ${OpenCV_LIBS} ${Sophus_LIBRARIES} ${Pangolin_LIBRARIES} ${G2O_LIBS} ${CHOLMOD_LIBRARIES})
