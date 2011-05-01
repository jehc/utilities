CXXFLAGS=-g
INCLUDE=-I/home/kmatzen/local/include/pcl-2.0 -I/home/kmatzen/local/include/eigen3
LIBS=-L/home/kmatzen/local/lib -lply -lopencv_core -lopencv_imgproc -lpcl_io -lboost_filesystem

all: ConvolutionDetection

ConvolutionDetection: main.cpp
	g++ $(CXXFLAGS) $(INCLUDE) main.cpp $(LIBS) -o ConvolutionDetection

clean:
	rm -rf ConvolutionDetection
