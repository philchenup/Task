cmake_minimum_required(VERSION 3.16)
project(lobb_curved_weld LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(PCL 1.10 REQUIRED COMPONENTS common io filters kdtree search visualization)

add_executable(lobb_weld
  src/main.cpp
  src/extractor.cpp
  src/demo_cloud.cpp
)

target_include_directories(lobb_weld PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${PCL_INCLUDE_DIRS}
)
target_link_directories(lobb_weld PRIVATE ${PCL_LIBRARY_DIRS})
target_link_libraries(lobb_weld PRIVATE ${PCL_LIBRARIES})
add_definitions(${PCL_DEFINITIONS})

if(MSVC)
  target_compile_options(lobb_weld PRIVATE /W3 /O2)
else()
  target_compile_options(lobb_weld PRIVATE -Wall -Wextra -Wno-unused-parameter)
endif()

enable_testing()
add_test(NAME sine_fillet COMMAND lobb_weld --demo sine --output ${CMAKE_BINARY_DIR}/demo-sine)
add_test(NAME pipe_plate COMMAND lobb_weld --demo pipe --output ${CMAKE_BINARY_DIR}/demo-pipe)
set_tests_properties(sine_fillet pipe_plate PROPERTIES TIMEOUT 180)
