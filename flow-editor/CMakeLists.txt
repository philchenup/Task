cmake_minimum_required(VERSION 3.5)
project(flow-editor LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

find_package(Qt5 5.15 REQUIRED COMPONENTS Widgets)

add_executable(flow-editor
    src/main.cpp
    src/StepRegistry.cpp
    src/PortItem.cpp
    src/EdgeItem.cpp
    src/NodeItem.cpp
    src/FlowScene.cpp
    src/FlowView.cpp
    src/StepLibrary.cpp
    src/ProcedureList.cpp
    src/MainWindow.cpp
    src/Types.h
    src/StepRegistry.h
    src/PortItem.h
    src/EdgeItem.h
    src/NodeItem.h
    src/FlowScene.h
    src/FlowView.h
    src/StepLibrary.h
    src/ProcedureList.h
    src/MainWindow.h
)

target_include_directories(flow-editor PRIVATE src)
target_link_libraries(flow-editor PRIVATE Qt5::Widgets)

if(MSVC)
    target_compile_options(flow-editor PRIVATE /utf-8 /W3)
else()
    target_compile_options(flow-editor PRIVATE -Wall -Wextra)
endif()
