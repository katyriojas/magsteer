#-------------------------------------------------
#
# Project created by QtCreator 2017-07-11T11:18:06
#
#-------------------------------------------------

QT       += core

QT       -= gui

TARGET = SmaractEtherTest
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app


SOURCES += main.cpp \
    ActsAsynch.cpp

HEADERS += \
    MCSControl.h \
    ActsAsynch.h

LIBS += -L"C:\Users\riojaske\Desktop\SmaractEtherTest_API"  -lMCSControl
