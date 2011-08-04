CXXFLAGS     = -g -fopenmp -Wall -Werror -Wno-non-virtual-dtor
INCLUDE	     = -I/home/kmatzen/local/include/pcl-1.0 -I/home/kmatzen/local/include/eigen3
LIBS         = -lBundleFile -lopencv_core -lopencv_highgui -lpcl_features -lpcl_kdtree -lpcl_surface -lpcl_io -lboost_system -lboost_filesystem -lpcl_filters -lboost_thread -lply -lkvo
 
all:	flipcolor

install:	flipcolor
		mkdir -p ~/utilities/bin
		cp flipcolor ~/utilities/bin

flipcolor:	main.cpp
	g++ main.cpp $(INCLUDE) -L/home/kmatzen/local/lib $(CXXFLAGS) $(LIBS) -o flipcolor

clean:
	rm -rf flipcolor
