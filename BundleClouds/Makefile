CXXFLAGS     = -O3
LIB_PATH     = -L../BundleFile
LIBS         = -lBundleFile -lopencv_core -lopencv_highgui 
INCLUDE_PATH = -I../BundleFile
 
all:	BundleClouds

BundleClouds:	../BundleFile/libBundleFile.a main.cpp
	g++ main.cpp -L$(LD_LIBRARY_PATH) $(CXXFLAGS) $(LIB_PATH) $(LIBS) $(INCLUDE_PATH) -o BundleClouds

../BundleFile/libBundleFile.a:
	cd ../BundleFile; make; cd ../BundleClouds

clean:
	rm -rf BundleClouds
