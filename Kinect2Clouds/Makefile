CXXFLAGS     = -g -fopenmp -Wall -Wno-non-virtual-dtor
INCLUDE	     = -I/home/kmatzen/local/include/pcl-1.0 -I/home/kmatzen/local/include/eigen3
LIBS         = -lbundlefile -lopencv_core -lopencv_highgui -lpcl_features -lpcl_kdtree -lpcl_surface -lpcl_io -lboost_system -lboost_filesystem -lpcl_filters -lboost_thread -lplyfile -lm
 
all:	Kinect2Clouds

install:	Kinect2Clouds
		mkdir -p ~/utilities/bin
		cp Kinect2Clouds ~/utilities/bin

Kinect2Clouds:	main.cpp
	g++ main.cpp $(INCLUDE) -L/home/kmatzen/local/lib $(CXXFLAGS) $(LIBS) -o Kinect2Clouds

clean:
	rm -rf Kinect2Clouds
