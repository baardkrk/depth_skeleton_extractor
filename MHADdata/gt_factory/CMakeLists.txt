cmake_minimum_required (VERSION 2.6)
project(GT_factory)

# Version number
set (GT_factory_VERSION_MAJOR 0)
set (GT_factory_VERSION_MINOR 1)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

include_directories (
  "${PROJECT_SOURCE_DIR}/inlcude"
  )

add_executable(MHAD_BVH_2_16_KEYPOINTS src/bvh_read.cpp)
target_link_libraries (MHAD_BVH_2_16_KEYPOINTS Eigen3::Eigen)

#add_executable(util_test src/util.cpp)

set_property(
  TARGET
  MHAD_BVH_2_16_KEYPOINTS
  #  util_test
  PROPERTY
  CXX_STANDARD 11
)



