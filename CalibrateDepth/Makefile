CXXFLAGS     = -g# -fopenmp#O3
LIBS         = -L/home/kmatzen/local/lib -lbundlefile -lopencv_core -lopencv_highgui 
INCLUDE	     = -I/home/kmatzen/local/include/eigen3
 
all:	CalibrateDepth

CalibrateDepth:	main.cpp
	g++ main.cpp $(INCLUDE) $(CXXFLAGS) $(LIBS) -o CalibrateDepth

install:
	if [ ! -d ~/utilities/bin ]; then mkdir ~/utilities/bin; fi
	cp CalibrateDepth ~/utilities/bin

clean:
	rm -rf CalibrateDepth
