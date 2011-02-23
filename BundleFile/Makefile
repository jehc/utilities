CXXFLAGS = -O3

libBundleFile.a:	BundleFile.o BundlePoint.o BundleView.o BundleCamera.o
	ar rcs libBundleFile.a BundleFile.o BundlePoint.o BundleView.o BundleCamera.o

install:	libBundleFile.a
	if [ ! -d ~/utilities/lib ]; then mkdir ~/utilities/lib; fi
	cp libBundleFile.a ~/utilities/lib
	if [ ! -d ~/utilities/include ]; then mkdir ~/utilities/include; fi
	cp BundleFile.h ~/utilities/include
	cp BundlePoint.h ~/utilities/include
	cp BundleView.h ~/utilities/include
	cp BundleCamera.h ~/utilities/include

BundleFile.o:	BundleFile.cpp BundleFile.h BundlePoint.h BundleView.h BundleCamera.h
	g++ $(CXXFLAGS) BundleFile.cpp -c -o BundleFile.o

BundlePoint.o:	BundlePoint.h BundlePoint.cpp BundleView.h
	g++ $(CXXFLAGS) BundlePoint.cpp -c -o BundlePoint.o

BundleView.o:	BundleView.h BundleView.cpp
	g++ $(CXXFLAGS) BundleView.cpp -c -o BundleView.o

BundleCamera.o:	BundleCamera.h BundleCamera.cpp
	g++ $(CXXFLAGS) BundleCamera.cpp -c -o BundleCamera.o

clean:
	rm *.o libBundleFile.a
