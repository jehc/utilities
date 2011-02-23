CXXFLAGS     = -O3
LIBS         = -lBundleFile -lopencv_core -lopencv_highgui 
 
all:	BundleClouds

install:	BundleClouds
		if [ ! -d ~/utilities/bin ]; then mkdir ~/utilities/bin; fi
		cp BundleClouds ~/utilities/bin

BundleClouds:	main.cpp
	g++ main.cpp -L/home/kmatzen/local/lib $(CXXFLAGS) $(LIBS) -o BundleClouds

clean:
	rm -rf BundleClouds
