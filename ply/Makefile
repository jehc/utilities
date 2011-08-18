MAJOR=0
MINOR=0
REVISION=1
CXXFLAGS=-g -fPIC
CFLAGS=-g -fPIC
INCLUDE=-I. -I/usr/include/pcl-1.1 -I/usr/local/include/pcl-1.0/ -I/usr/include/pcl-1.0 -I/home/kmatzen/local/include/pcl-1.0 -I/usr/local/include/eigen3 -I/home/kmatzen/local/include/eigen3

libplyfile.so.$(MAJOR).$(MINOR).$(REVISION):	ply.o ply_io.o
	gcc -shared -Wl,-soname,libplyfile.so.$(MAJOR) -o $@ $^ -lc

ply.o:	ply.h ply.c
	gcc $(CFLAGS) $(INCLUDE) ply.c -c -o ply.o

ply_io.o:	ply_io.h ply_io.cpp
	g++ $(CXXFLAGS) $(INCLUDE) ply_io.cpp -c -o ply_io.o

install: libplyfile.so.$(MAJOR).$(MINOR).$(REVISION)
	mkdir -p ~/utilities/lib
	cp $< ~/utilities/lib
	ln -s -f /home/kmatzen/utilities/lib/$< /home/kmatzen/utilities/lib/libplyfile.so
	ln -s -f /home/kmatzen/utilities/lib/$< /home/kmatzen/utilities/lib/libplyfile.so.$(MAJOR)
	mkdir -p ~/utilities/include
	cp ply_io.h ~/utilities/include	

clean:
	rm -rf *.o *.so*
