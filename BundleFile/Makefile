CXXFLAGS = -O3

libBundleFile.a:	BundleFile.o BundlePoint.o BundleView.o BundleCamera.o
	ar rcs libBundleFile.a BundleFile.o BundlePoint.o BundleView.o BundleCamera.o

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
