CXXFLAGS=-fopenmp -g
INCLUDE=-I/home/kmatzen/local/include/pcl-1.0 -I/home/kmatzen/local/include/eigen3
LIBS=-L/home/kmatzen/local/lib -lply -lkvo -lopencv_core -lopencv_imgproc -lpcl_io -lboost_filesystem -lboost_thread -lm

all: ConvolutionDetection

ConvolutionDetection: main.cpp
	g++ $(CXXFLAGS) $(INCLUDE) main.cpp $(LIBS) -o ConvolutionDetection

clean:
	rm -rf ConvolutionDetection
