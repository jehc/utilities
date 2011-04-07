INCLUDE=-I/home/kmatzen/local/include/pcl-2.0 -I/home/kmatzen/local/include/eigen3
LIBS=-L/home/kmatzen/local/lib -lassimp -lply

all:	dae2ply

dae2ply:	main.cpp
	g++ -g main.cpp $(INCLUDE) $(LIBS) -o dae2ply

clean:
	rm -rf dae2ply
