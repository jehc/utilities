######################################################################
# Automatically generated by qmake (2.01a) Sun May 29 15:29:50 2011
######################################################################

TEMPLATE = app
TARGET = 
DEPENDPATH += .
INCLUDEPATH += . /usr/local/include/pcl-1.0 /Users/kmatzen/utilities/include /home/kmatzen/utilities/include /usr/include/pcl-1.0 /home/kmatzen/utilities/include /usr/local/include/eigen3
LIBS += -L/usr/local/lib -L/Users/kmatzen/utilities/lib -L/home/kmatzen/utilities/lib -lBundleFile -lGLEW -lpcl_io -lply -lopencv_highgui -lopencv_core -lpcl_common 

QMAKE_CXXFLAGS += -fopenmp -DNDEBUG
QMAKE_LFLAGS += -fopenmp

CONFIG -= release
CONFIG += debug

QT += opengl

# Input
HEADERS += HighResViewerWidget.h \

SOURCES += HighResViewerWidget.cpp \
           main.cpp
