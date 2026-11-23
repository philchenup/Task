QT += widgets
CONFIG += c++14 warn_on
CONFIG -= app_bundle
TARGET = flow-editor
TEMPLATE = app

greaterThan(QT_MAJOR_VERSION, 5): error("This example is written for Qt 5.15")
!versionAtLeast(QT_VERSION, 5.15.0): error("Qt 5.15 is required")

win32-msvc* {
    QMAKE_CXXFLAGS += /utf-8
}

INCLUDEPATH += src

SOURCES += \
    src/main.cpp \
    src/StepRegistry.cpp \
    src/PortItem.cpp \
    src/EdgeItem.cpp \
    src/NodeItem.cpp \
    src/FlowScene.cpp \
    src/FlowView.cpp \
    src/StepLibrary.cpp \
    src/ProcedureList.cpp \
    src/MainWindow.cpp

HEADERS += \
    src/Types.h \
    src/StepRegistry.h \
    src/PortItem.h \
    src/EdgeItem.h \
    src/NodeItem.h \
    src/FlowScene.h \
    src/FlowView.h \
    src/StepLibrary.h \
    src/ProcedureList.h \
    src/MainWindow.h
