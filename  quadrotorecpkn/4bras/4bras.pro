#-------------------------------------------------
#
# Project created by QtCreator 2010-12-30T01:25:44
#
#-------------------------------------------------

QT       += core gui
QT       += opengl

TARGET = 4bras
TEMPLATE = app
INCLUDEPATH += .

SOURCES += main.cpp\
        mainwindow.cpp \
    Mycom/Mycom.cpp \
    Mycom/win_qextserialport.cpp \
    Mycom/qextserialbase.cpp \
    3D/Module3D.cpp \
    Mycom/MachineThread.cpp

HEADERS  += mainwindow.h \
    Mycom/Mycom.h \
    Mycom/win_qextserialport.h \
    Mycom/qextserialbase.h \
    3D/Module3D.h \
    datafile/data_protocol.h \
    Mycom/MachineThread.h
