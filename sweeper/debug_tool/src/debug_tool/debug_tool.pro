# ----------------------------------------------------
# Debug Tool Project File
# Author: liuzheng
# Create: 2017/7/18
# ------------------------------------------------------

TEMPLATE = app
TARGET = DebugTool

QT += core gui widgets

CONFIG += c++11
CONFIG += debug_and_release
DEFINES += WIN64

INCLUDEPATH += ./GeneratedFiles \
               "D:/Program Files/boost/include/boost-1_70" \
               "D:/Program Files/jsoncpp/include" \
               "D:/Program Files/libzmq/include"

CONFIG(debug, debug|release) {
    DEFINES += TEST

    DESTDIR = ../x64/Debug
    MOC_DIR += ./GeneratedFiles/Debug
    OBJECTS_DIR += ./GeneratedFiles/Debug
    UI_DIR += ./GeneratedFiles
    RCC_DIR += ./GeneratedFiles

    INCLUDEPATH += ./GeneratedFiles/Debug
    LIBS += "D:/Program Files/jsoncpp/lib/jsoncppd.lib" \
            "D:/Program Files/libzmq/lib/libzmq-v141-mt-4_3_1.lib" \
            -L"D:/Program Files/boost/lib" "boost_system-vc141-mt-gd-x64-1_70.lib"
}

CONFIG(release, debug|release) {
    DESTDIR = ../x64/Release
    MOC_DIR += ./GeneratedFiles/Release
    OBJECTS_DIR += ./GeneratedFiles/Release
    UI_DIR += ./GeneratedFiles
    RCC_DIR += ./GeneratedFiles

    INCLUDEPATH += ./GeneratedFiles/Release
    LIBS += "D:/Program Files/jsoncpp/lib/jsoncpp.lib" \
            "D:/Program Files/libzmq/lib/libzmq-v141-mt-4_3_1.lib" \
            "D:/Program Files/boost/lib/boost_filesystem-vc141-mt-x64-1_70.lib" \
            -L"D:/Program Files/boost/lib" "boost_filesystem-vc141-mt-x64-1_70.lib"
}

HEADERS += src/QAlgorithmWidget.h \
           src/QDataDisplayDialog.h \
           src/QEulerAngleWidget.h \
           src/QPerceptionParamWidget.h \
           src/QPerceptionWidget.h \
           src/QPlanningWidget.h \
           src/QVarianceWidget.h \
           src/QBaseShowWidget.h \
           src/QDebugToolMainWnd.h \
           src/QLocationWidget.h \
           src/QPerceptionShow3DWidget.h \
           src/QPlanningParamWidget.h \
           src/GlobalDefine.h \
           src/QBaseWidget.h \
           src/QErrorFigureWidget.h \
           src/QLocusWidget.h \
           src/QPerceptionShowWidget.h \
           src/QPlanningShowWidget.h \
           src/QUltrasonicWidget.h \
           src/ReadDataManager.h

SOURCES += src/main.cpp \
           src/GlobalDefine.cpp \
           src/QBaseShowWidget.cpp \
           src/QDebugToolMainWnd.cpp \
           src/QLocationWidget.cpp \
           src/QPerceptionShow3DWidget.cpp \
           src/QPlanningParamWidget.cpp \
           src/QBaseWidget.cpp \
           src/QErrorFigureWidget.cpp \
           src/QLocusWidget.cpp \
           src/QPerceptionShowWidget.cpp \
           src/QPlanningShowWidget.cpp \
           src/QUltrasonicWidget.cpp \
           src/ReadDataManager.cpp \
           src/QAlgorithmWidget.cpp \
           src/QDataDisplayDialog.cpp \
           src/QEulerAngleWidget.cpp \
           src/QPerceptionParamWidget.cpp \
           src/QPerceptionWidget.cpp \
           src/QPlanningWidget.cpp \
           src/QVarianceWidget.cpp
