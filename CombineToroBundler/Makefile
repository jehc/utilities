LIBS=-ltorofile -lbundlefile
LIB_DIR=-L/home/kmatzen/local/lib
INCLUDE=-I/home/kmatzen/local/include -I/home/kmatzen/local/include/eigen3
CXX_FLAGS=-g

CombineToroBundler: main.cpp
	g++ main.cpp $(CXX_FLAGS) $(LIBS) $(LIB_DIR) $(INCLUDE) -o CombineToroBundler

clean:
	rm -rf CombineToroBundler
