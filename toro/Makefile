MAJOR=0
MINOR=0
REVISION=1
CXXFLAGS = -g -fPIC -Wall
INCLUDE = -I/usr/local/include/eigen3 -I/home/kmatzen/local/include/eigen3

libtorofile.so.$(MAJOR).$(MINOR).$(REVISION): Toro3DFile.o
	gcc -shared -Wl,-soname,libtorofile.so.$(MAJOR) -o $@ $^ -lc

install: libtorofile.so.$(MAJOR).$(MINOR).$(REVISION)
	mkdir -p ~/utilities/lib
	cp $< ~/utilities/lib
	ln -s -f /home/kmatzen/utilities/lib/$< /home/kmatzen/utilities/lib/libtorofile.so
	ln -s -f /home/kmatzen/utilities/lib/$< /home/kmatzen/utilities/lib/libtorofile.so.$(MAJOR)
	mkdir -p ~/utilities/include
	cp Toro3DFile.h ~/utilities/include

%.o: %.cpp %.h
	g++ $(CXXFLAGS) $(INCLUDE) $< -c -o $@

clean:
	rm *.o libtorofile.so*
