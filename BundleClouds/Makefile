CXXFLAGS = -O3
LIBS     = -lopencv_core -lopencv_highgui 

all:	BundleClouds

BundleClouds:	BundlePoint.h BundlePoint.o BundleView.h BundleView.o BundleCamera.h BundleCamera.o BundleFile.h BundleFile.o main.cpp
	g++ $(CXXFLAGS) $(LIBS) BundlePoint.o BundleView.o BundleCamera.o BundleFile.o main.cpp -o BundleClouds

BundlePoint.o:	BundlePoint.h BundlePoint.cpp BundleView.h
	g++ $(CXXFLAGS) BundlePoint.cpp -c -o BundlePoint.o

BundleView.o:	BundleView.h BundleView.cpp
	g++ $(CXXFLAGS) BundleView.cpp -c -o BundleView.o

BundleCamera.o:	BundleCamera.h BundleCamera.cpp
	g++ $(CXXFLAGS) BundleCamera.cpp -c -o BundleCamera.o

clean:
	rm *.o BundleClouds
