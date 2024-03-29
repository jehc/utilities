######################################################################
# Automatically generated by qmake (2.01a) Wed Nov 24 15:24:38 2010
######################################################################

TEMPLATE = app
TARGET = 
CONFIG += debug
DEPENDPATH += .

win32-msvc2008{
INCLUDEPATH += "C:\\OpenCV-2.2.0\\release\\include"
LIBS += -L"C:\\OpenCV-2.2.0\\release\\lib" -lglew32
CONFIG (debug, debug|release){
LIBS += -lopencv_highgui220d -lopencv_imgproc220d -lopencv_core220d
}else{
LIBS += -lopencv_highgui220 -lopencv_imgproc220 -lopencv_core220
}
}

macx-g++{
INCLUDEPATH += . /usr/local/Cellar/opencv/2.1.1-pre/include
LIBS += -L/usr/local/Cellar/glew/1.5.5/lib/ -lGLEW -L/usr/local/Cellar/opencv/2.1.1-pre/lib/ -lopencv_highgui -lopencv_imgproc -lopencv_core
}

linux-g++-64{
LIBS += -lopencv_highgui -lopencv_imgproc -lopencv_core -lGLEW
}

linux-g++{
LIBS += -lopencv_highgui -lopencv_imgproc -lopencv_core -lGLEW
}

QT += opengl

# Input
HEADERS += ReprojectDepthWidget.h ShaderException.h
SOURCES += Application.cpp ReprojectDepthWidget.cpp
