CXXFLAGS     = -g#O3
LIBS         = -L/home/kmatzen/local/lib -lBundleFile -lopencv_core -lopencv_highgui 
 
all:	CalibrateDepth

CalibrateDepth:	main.cpp
	g++ main.cpp $(CXXFLAGS) $(LIBS) -o CalibrateDepth

install:
	if [ ! -d ~/utilities/bin ]; then mkdir ~/utilities/bin; fi
	cp CalibrateDepth ~/utilities/bin

clean:
	rm -rf CalibrateDepth
