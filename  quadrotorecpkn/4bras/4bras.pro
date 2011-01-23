#-------------------------------------------------
#
# Project created by QtCreator 2010-12-30T01:25:44
#
#-------------------------------------------------

QT       += core gui

TARGET = 4bras
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    Mycom/Mycom.cpp \
    Mycom/win_qextserialport.cpp \
    Mycom/qextserialbase.cpp \
    Mycom/ComThread.cpp

HEADERS  += mainwindow.h \
    Mycom/Mycom.h \
    Mycom/win_qextserialport.h \
    Mycom/qextserialbase.h \
    Mycom/ComThread.h
