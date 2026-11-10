cmake_minimum_required(VERSION 3.16)
project(Decoder LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

find_package(Qt6 REQUIRED COMPONENTS Widgets)

add_executable(decoder
    main.cpp
    DecoderWindow.cpp
    DecoderWindow.h
    LicenseCode.cpp
    LicenseCode.h
)
target_link_libraries(decoder PRIVATE Qt6::Widgets)

add_executable(decoder_test
    decoder_test.cpp
    DecoderWindow.cpp
    DecoderWindow.h
    LicenseCode.cpp
    LicenseCode.h
)
target_link_libraries(decoder_test PRIVATE Qt6::Widgets)
