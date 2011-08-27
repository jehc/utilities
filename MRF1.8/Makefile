SRC =  mrf.cpp ICM.cpp GCoptimization.cpp graph.cpp maxflow.cpp \
       MaxProdBP.cpp LinkedBlockList.cpp regions-maxprod.cpp \
       TRW-S.cpp BP-S.cpp

CC = g++

#CPPFLAGS = -O2 -W -Wall
CPPFLAGS = -O2 -W -Wall -g -m32
#CPPFLAGS = -DUSE_64_BIT_PTR_CAST -O2 -W -Wall

OBJ = $(SRC:.cpp=.o)

all: libMRF.a example

libMRF.a: $(OBJ)
	rm -f libMRF.a
	ar ruc libMRF.a $(OBJ)
	ranlib libMRF.a

example: libMRF.a example.o
	$(CC) -m32 -o example example.o -L. -lMRF

clean: 
	rm -f $(OBJ) example.o

allclean: clean
	rm -f libMRF.a example
