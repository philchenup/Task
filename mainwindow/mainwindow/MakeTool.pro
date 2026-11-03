QT += widgets
CONFIG += c++11
CONFIG += warn_on

# 源文件为 UTF-8。MSVC 需要这个选项才能正确编译中文字符串。
msvc {
    QMAKE_CXXFLAGS += /utf-8
}

TARGET = MakeToolDemo
TEMPLATE = app

INCLUDEPATH += $$PWD/third_party

SOURCES += \
    main.cpp \
    tool/MakeTool.cpp

HEADERS += \
    tool/MakeTool.h
