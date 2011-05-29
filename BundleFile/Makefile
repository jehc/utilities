CXXFLAGS = -g
INCLUDE = -I/home/kmatzen/local/include/eigen3

libBundleFile.a:	BundleFile.o BundlePoint.o BundleView.o BundleCamera.o
	ar rcs libBundleFile.a BundleFile.o BundlePoint.o BundleView.o BundleCamera.o

install:	libBundleFile.a
	mkdir -p ~/utilities/lib
	cp libBundleFile.a ~/utilities/lib
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
	rm *.o libBundleFile.a
