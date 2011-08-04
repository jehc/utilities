CXXFLAGS     = -g -fopenmp -Wall -Wno-non-virtual-dtor
INCLUDE	     = -I/home/kmatzen/local/include/pcl-1.0 -I/home/kmatzen/local/include/eigen3
LIBS         = -lbundlefile -lopencv_core -lopencv_highgui -lpcl_features -lpcl_kdtree -lpcl_surface -lpcl_io -lboost_system -lboost_filesystem -lpcl_filters -lboost_thread -lply -lkvo -lm
 
all:	ICPAugment

install:	ICPAugment
		mkdir -p ~/utilities/bin
		cp ICPAugment ~/utilities/bin

ICPAugment:	main.cpp
	g++ main.cpp $(INCLUDE) -L/home/kmatzen/local/lib $(CXXFLAGS) $(LIBS) -o ICPAugment

clean:
	rm -rf ICPAugment
