######################################################################
# Automatically generated by qmake (2.01a) Wed Nov 24 15:24:38 2010
######################################################################

TEMPLATE = app
TARGET = 
DEPENDPATH += .

win32-msvc2008{
}

macx-g++{
INCLUDEPATH += . /usr/local/Cellar/opencv/2.1.1-pre/include
LIBS += -L/usr/local/Cellar/glew/1.5.5/lib/ -lGLEW -L/usr/local/Cellar/opencv/2.1.1-pre/lib/ -lopencv_highgui -lopencv_imgproc -lopencv_core
}

QT += opengl
CONFIG += debug

# Input
HEADERS += ReprojectDepthWidget.h ShaderException.h
SOURCES += Application.cpp ReprojectDepthWidget.cpp
