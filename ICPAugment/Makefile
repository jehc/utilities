CXXFLAGS     = -g -fopenmp -Wall -Wno-non-virtual-dtor
INCLUDE	     = -I/home/kmatzen/utilities/include -I/home/kmatzen/local/include -I/home/kmatzen/local/include/pcl-1.2 -I/home/kmatzen/prefix/usr/include/eigen3
LIBS         = -L/home/kmatzen/utilities/lib -L/home/kmatzen/prefix/usr/lib -L/home/kmatzen/prefix/usr/lib64 -L/home/kmatzen/local/lib -lbundlefile -lopencv_core -lopencv_highgui -lpcl_features -lpcl_kdtree -lpcl_surface -lpcl_io -lboost_system -lboost_filesystem -lpcl_filters -lboost_thread -lplyfile -lm -lboost_iostreams -lpcl_octree -ltorofile -lflann_cpp -lqhull -lcminpack
 
all:	ICPAugment

install:	ICPAugment
		mkdir -p ~/utilities/bin
		cp $< ~/utilities/bin

ICPAugment:	main.cpp
	g++ $^ $(INCLUDE) $(CXXFLAGS) $(LIBS) -o $@

clean:
	rm -rf ICPAugment
