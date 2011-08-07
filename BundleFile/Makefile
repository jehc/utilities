MAJOR=0
MINOR=0
REVISION=1
CXXFLAGS = -g -fPIC -Wall
INCLUDE = -I/usr/local/include/eigen3 -I/home/kmatzen/local/include/eigen3

libbundlefile.so.$(MAJOR).$(MINOR).$(REVISION): BundleFile.o BundlePoint.o BundleView.o BundleCamera.o CoordsFile.o TracksFile.o
	gcc -shared -Wl,-soname,libbundlefile.so.$(MAJOR) -o $@ $^ -lc

install: libbundlefile.so.$(MAJOR).$(MINOR).$(REVISION)
	mkdir -p ~/utilities/lib
	cp $< ~/utilities/lib
	ln -s -f /home/kmatzen/utilities/lib/$< /home/kmatzen/utilities/lib/libbundlefile.so
	ln -s -f /home/kmatzen/utilities/lib/$< /home/kmatzen/utilities/lib/libbundlefile.so.$(MAJOR)
	mkdir -p ~/utilities/include
	cp BundleFile.h ~/utilities/include
	cp BundlePoint.h ~/utilities/include
	cp BundleView.h ~/utilities/include
	cp BundleCamera.h ~/utilities/include
	cp CoordsFile.h ~/utilities/include
	cp TracksFile.h ~/utilities/include

%.o: %.cpp %.h
	g++ $(CXXFLAGS) $(INCLUDE) $< -c -o $@

clean:
	rm *.o libbundlefile.so*
