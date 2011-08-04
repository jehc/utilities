MAJOR=0
MINOR=0
REVISION=1
CXXFLAGS = -g -fPIC -Wall
INCLUDE = -I/usr/local/include/eigen3 -I/home/kmatzen/local/include/eigen3

libbundlefile.so.$(MAJOR).$(MINOR).$(REVISION): BundleFile.o BundlePoint.o BundleView.o BundleCamera.o
	gcc -shared -Wl,-soname,libbundlefile.so.$(MAJOR) -o libbundlefile.so.$(MAJOR).$(MINOR).$(REVISION) BundleFile.o BundlePoint.o BundleView.o BundleCamera.o -lc

install: libbundlefile.so.$(MAJOR).$(MINOR).$(REVISION)
	mkdir -p ~/utilities/lib
	cp libbundlefile.so.$(MAJOR).$(MINOR).$(REVISION) ~/utilities/lib
	ln -s -f /home/kmatzen/utilities/lib/libbundlefile.so.$(MAJOR).$(MINOR).$(REVISION) /home/kmatzen/utilities/lib/libbundlefile.so
	ln -s -f /home/kmatzen/utilities/lib/libbundlefile.so.$(MAJOR).$(MINOR).$(REVISION) /home/kmatzen/utilities/lib/libbundlefile.so.$(MAJOR)
	mkdir -p ~/utilities/include
	cp BundleFile.h ~/utilities/include
	cp BundlePoint.h ~/utilities/include
	cp BundleView.h ~/utilities/include
	cp BundleCamera.h ~/utilities/include

BundleFile.o:	BundleFile.cpp BundleFile.h BundlePoint.h BundleView.h BundleCamera.h
	g++ $(CXXFLAGS) $(INCLUDE) BundleFile.cpp -c -o BundleFile.o

BundlePoint.o:	BundlePoint.h BundlePoint.cpp BundleView.h
	g++ $(CXXFLAGS) $(INCLUDE) BundlePoint.cpp -c -o BundlePoint.o

BundleView.o:	BundleView.h BundleView.cpp
	g++ $(CXXFLAGS) $(INCLUDE) BundleView.cpp -c -o BundleView.o

BundleCamera.o:	BundleCamera.h BundleCamera.cpp
	g++ $(CXXFLAGS) $(INCLUDE) BundleCamera.cpp -c -o BundleCamera.o

clean:
	rm *.o libbundlefile.so*
