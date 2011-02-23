CXXFLAGS     = -O3
LIBS         = -lBundleFile -lopencv_core -lopencv_highgui 
 
all:	BundleClouds

BundleClouds:	main.cpp
	g++ main.cpp -L/home/kmatzen/local/lib $(CXXFLAGS) $(LIBS) -o BundleClouds

clean:
	rm -rf BundleClouds
