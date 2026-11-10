cmake_minimum_required(VERSION 3.16)
# VTK 的 MPI 导入目标要求启用 C 语言。
project(weld_curvature LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(PCL 1.12 REQUIRED COMPONENTS
  common
  io
  filters
  sample_consensus
  segmentation
  kdtree
  search
  features
  visualization
)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(weld_curvature weld_curvature.cpp)
target_link_libraries(weld_curvature PRIVATE ${PCL_LIBRARIES})

if(MSVC)
  target_compile_options(weld_curvature PRIVATE /W3)
else()
  target_compile_options(weld_curvature PRIVATE -Wall -Wextra -Wno-unused-parameter)
endif()
