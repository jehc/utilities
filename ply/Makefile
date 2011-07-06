CXXFLAGS=-g
CFLAGS=-g
INCLUDE=-I. -I/usr/include/pcl-1.0 -I/home/kmatzen/local/include/pcl-1.0 -I/usr/local/include/eigen3 -I/home/kmatzen/local/include/eigen3

all: libply.a

libply.a:	ply.o ply_io.o
	ar rcs libply.a ply.o ply_io.o

ply.o:	ply.h ply.c
	gcc $(CFLAGS) $(INCLUDE) ply.c -c -o ply.o

ply_io.o:	ply_io.h ply_io.cpp
	g++ $(CXXFLAGS) $(INCLUDE) ply_io.cpp -c -o ply_io.o

install: libply.a
	cp libply.a ../lib
	cp ply_io.h ../include

clean:
	rm -rf *.o *.a
