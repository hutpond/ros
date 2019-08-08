#-------------------------------------------------
#
# Project created by QtCreator 2019-07-16T10:39:50
#
#-------------------------------------------------

QT       += core gui widgets serialport

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = PlanningPathEditor
TEMPLATE = app

CONFIG += debug_and_release
CONFIG(debug, debug|release) {
  DEFINES += TEST
}

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

CONFIG += c++17

SOURCES += \
        QDrawPathWidget.cpp \
        QOpenDriveObject.cpp \
        QPanelWidget.cpp \
        QProjectObject.cpp \
        QReadDataObject.cpp \
        QSettingDialog.cpp \
        main.cpp \
        QEditorMainWindow.cpp

HEADERS += \
        QDrawPathWidget.h \
        QEditorMainWindow.h \
        QOpenDriveObject.h \
        QPanelWidget.h \
        QProjectObject.h \
        QReadDataObject.h \
        QSettingDialog.h

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

RESOURCES += \
  planningpatheditor.qrc

DISTFILES +=
