CXXFLAGS     = -O3
LIB_PATH     = -L../BundleFile
LIBS         = -lBundleFile -lopencv_core -lopencv_highgui 
INCLUDE_PATH = -I../BundleFile
 
all:	BundleClouds

BundleClouds:	../BundleFile/libBundleFile.a main.cpp
	g++ main.cpp $(CXXFLAGS) $(LIB_PATH) $(LIBS) $(INCLUDE_PATH) -o BundleClouds

../BundleFile/libBundleFile.a:
	cd ../BundleFile; make; cd ../BundleClouds

clean:
	rm *.o BundleClouds
