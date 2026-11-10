cmake_minimum_required(VERSION 3.16)
project(hik_camera_demo LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)

set(MVS_ROOT "/opt/MVS" CACHE PATH "Hikrobot MVS SDK root directory")

find_path(MVS_INCLUDE_DIR NAMES MvCameraControl.h
    HINTS
        "${MVS_ROOT}/include"
        "C:/Program Files (x86)/MVS/Development/Includes"
        "C:/Program Files/MVS/Development/Includes"
)

if(WIN32)
    find_library(MVS_LIBRARY NAMES MvCameraControl
        HINTS
            "C:/Program Files (x86)/MVS/Development/Libraries/win64"
            "C:/Program Files/MVS/Development/Libraries/win64"
    )
else()
    find_library(MVS_LIBRARY NAMES MvCameraControl
        HINTS
            "${MVS_ROOT}/lib/64"
            "${MVS_ROOT}/lib/aarch64"
            "${MVS_ROOT}/lib/armhf"
    )
endif()

if(NOT MVS_INCLUDE_DIR OR NOT MVS_LIBRARY)
    message(FATAL_ERROR
        "Hikrobot MVS SDK not found. Install MVS and set -DMVS_ROOT= to its root "
        "(Linux default is /opt/MVS).")
endif()

add_executable(hik_camera_demo
    src/main.cpp
    src/MainWindow.cpp
    src/MainWindow.h
    src/MvCameraWorker.cpp
    src/MvCameraWorker.h
)

target_include_directories(hik_camera_demo PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${MVS_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(hik_camera_demo PRIVATE
    Qt${QT_VERSION_MAJOR}::Widgets
    ${OpenCV_LIBS}
    ${MVS_LIBRARY}
)

if(UNIX AND NOT APPLE)
    get_filename_component(MVS_LIBRARY_DIR "${MVS_LIBRARY}" DIRECTORY)
    set_target_properties(hik_camera_demo PROPERTIES
        BUILD_RPATH "${MVS_LIBRARY_DIR}"
        INSTALL_RPATH "${MVS_LIBRARY_DIR}"
    )
endif()
