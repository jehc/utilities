# PLY geometry file filters

COPTIONS = -g
CFLAGS = -I. $(COPTIONS)
CP_FLAGS = $(CFLAGS) -woff 3262
LIBS = -lm

.C.o:
	CC $(CP_FLAGS) -c $*.C

all:  GenNormalHistogram 

GenNormalHistogram: GenNormalHistogram.o ply.o
	cc -o GenNormalHistogram GenNormalHistogram.o ply.o $(LIBS)

clean:
	rm -rf *.o GenNormalHistogram
