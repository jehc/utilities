CXXFLAGS = -O3
CXXFLAGS = -g
INCLUDE = -I. -I/home/kmatzen/local/include/eigen3 -I/usr/local/include/eigen3

all: libkvo.a

kvo.o: kvo.cpp kvo.h
	g++ kvo.cpp $(CXXFLAGS) $(INCLUDE) -c -o kvo.o

libkvo.a: kvo.o
	ar rcs libkvo.a kvo.o

install: libkvo.a
	mkdir -p ~/utilities/lib
	cp libkvo.a ~/utilities/lib
	mkdir -p ~/utilities/include
	cp kvo.h ~/utilities/include

clean:
	rm *.o libkvo.a
