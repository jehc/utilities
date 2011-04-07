INCLUDE=-I/home/kmatzen/local/include/pcl-2.0 -I/home/kmatzen/local/include/eigen3
LIBS=-L/home/kmatzen/local/lib -lassimp -lply -lpcl_io -lboost_filesystem

all:	sampleDAE

sampleDAE:	main.cpp
	g++ -g main.cpp $(INCLUDE) $(LIBS) -o sampleDAE

install:	all
	cp sampleDAE ../bin

clean:
	rm -rf sampleDAE
