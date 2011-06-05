######################################################################
# Automatically generated by qmake (2.01a) Sun May 29 15:29:50 2011
######################################################################

TEMPLATE = app
TARGET = 
DEPENDPATH += .
INCLUDEPATH += . /home/kmatzen/utilities/include /usr/local/include/eigen3 /home/kmatzen/build/qt/qt-everywhere-opensource-src-4.7.1/include/Qt3D /home/kmatzen/build/qt/qt-everywhere-opensource-src-4.7.1/demos/shared/
LIBS += -L/home/kmatzen/utilities/lib -lkvo -lGLEW -L/home/kmatzen/build/qt/qt-everywhere-opensource-src-4.7.1/lib -lQt3D -L/home/kmatzen/build/qt/qt-everywhere-opensource-src-4.7.1/demos/shared -ldemo_shared

QMAKE_CXXFLAGS += -fopenmp
QMAKE_LFLAGS += -fopenmp

CONFIG -= release
CONFIG += debug

QT += opengl

# Input
HEADERS += KVOViewerWidget.h \
           TransferFunctionWidget.h \
           GraphicsView.h \
           OpenGLScene.h

SOURCES += KVOViewerWidget.cpp \
           TransferFunctionWidget.cpp \
           OpenGLScene.cpp \
           main.cpp