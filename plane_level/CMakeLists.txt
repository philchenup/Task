cmake_minimum_required(VERSION 3.16)
project(plane_level LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCV REQUIRED)

add_library(plane_level
  src/geometry.cpp
  src/camera.cpp
  src/hand_eye.cpp
  src/steger.cpp
  src/registration.cpp
  src/leveling.cpp
)
target_include_directories(plane_level PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(plane_level PUBLIC ${OpenCV_LIBS})

add_executable(plane_level_tests tests/test_plane_level.cpp)
target_link_libraries(plane_level_tests PRIVATE plane_level)

add_executable(plane_level_demo src/demo.cpp)
target_link_libraries(plane_level_demo PRIVATE plane_level)

# Same Kabsch estimator as pcl::registration::TransformationEstimationSVD.
# Compiled only when PCL is installed; the OpenCV path is the one the tests run.
find_package(PCL QUIET COMPONENTS common registration)
if(PCL_FOUND)
  target_sources(plane_level PRIVATE src/pcl_bridge.cpp)
  target_compile_definitions(plane_level PUBLIC PLANE_LEVEL_HAS_PCL)
  target_include_directories(plane_level PUBLIC ${PCL_INCLUDE_DIRS})
  target_link_libraries(plane_level PUBLIC ${PCL_LIBRARIES})
endif()
