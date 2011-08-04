CXXFLAGS     = -g#O3
LIBS         = -L/home/kmatzen/local/lib -lBundleFile -lopencv_core -lopencv_highgui -lply
INCLUDE	     = -I/home/kmatzen/local/include/eigen3 -I/home/kmatzen/local/include/pcl-1.0
 
all:	SampleCalib

SampleCalib:	main.cpp
	g++ main.cpp $(INCLUDE) $(CXXFLAGS) $(LIBS) -o SampleCalib

install:
	if [ ! -d ~/utilities/bin ]; then mkdir ~/utilities/bin; fi
	cp SampleCalib ~/utilities/bin

clean:
	rm -rf SampleCalib
