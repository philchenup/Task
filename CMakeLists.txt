cmake_minimum_required(VERSION 3.16)
project(MeltCamZoom VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

find_package(Qt6 REQUIRED COMPONENTS Widgets Test)

add_library(meltcam_zoom STATIC
    src/MeltCamZoom.cpp
    src/MeltCamZoom.h
)
target_include_directories(meltcam_zoom PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(meltcam_zoom PUBLIC Qt6::Widgets)

if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(meltcam_zoom PRIVATE -Wall -Wextra -Wpedantic)
endif()

enable_testing()
add_executable(meltcam_zoom_test tests/meltcam_zoom_test.cpp)
target_link_libraries(meltcam_zoom_test PRIVATE meltcam_zoom Qt6::Test)
if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(meltcam_zoom_test PRIVATE -Wall -Wextra -Wpedantic)
endif()
add_test(NAME meltcam_zoom_test COMMAND meltcam_zoom_test)
set_tests_properties(meltcam_zoom_test PROPERTIES ENVIRONMENT "QT_QPA_PLATFORM=offscreen")
