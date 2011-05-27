INCLUDE=-I/home/kmatzen/local/include/pcl-1.0 -I/home/kmatzen/local/include/eigen3
LIBS=-L/home/kmatzen/local/lib -lassimp -lply -lpcl_io -lpcl_kdtree -lboost_filesystem -lboost_thread

all:	sampleDAE

sampleDAE:	main.cpp
	g++ -fopenmp -g main.cpp $(INCLUDE) $(LIBS) -o sampleDAE

install:	all
	cp sampleDAE ../bin

clean:
	rm -rf sampleDAE
