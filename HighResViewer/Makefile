CXXFLAGS     = -g -fopenmp -Wall -Werror -Wno-non-virtual-dtor
INCLUDE	     = -I/home/kmatzen/local/include/pcl-1.0 -I/home/kmatzen/local/include/eigen3
LIBS         = -lBundleFile -lopencv_core -lopencv_highgui -lpcl_features -lpcl_kdtree -lpcl_surface -lpcl_io -lboost_system -lboost_filesystem -lpcl_filters -lboost_thread -lply -lkvo
 
all:	HighResViewer

install:	HighResViewer
		mkdir -p ~/utilities/bin
		cp HighResViewer ~/utilities/bin

ply.o:	ply.c
	gcc ply.c -c -o ply.o

HighResViewer:	main.cpp ply.o
	g++ main.cpp ply.o $(INCLUDE) -L/home/kmatzen/local/lib $(CXXFLAGS) $(LIBS) -o HighResViewer

clean:
	rm -rf HighResViewer
