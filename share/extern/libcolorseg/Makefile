# Makefile: Felzenszwalb's graph-based image segmentation

BASE  := $(shell cd ../../share; pwd;)

LIBNAME = lib/libcolorseg.a

INCDIR = -I. -I$(BASE)/include
CPATH := $(CPATH):$(LIBPATH)/pgrlibdcstereo/
CXXFLAGS += -fPIC -O3 -Wall -DMT_Linux -DMT_IMPLEMENT_TEMPLATES

DBG    = -g
OPT    = -O3
CPP    = g++
CFLAGS = $(OPT) $(INCDIR) -Wall -fno-default-inline
LINK   = -lm 
OBJ    = \
   convolve.o \
   disjoint-set.o \
   filter.o \
   image.o \
   imconv.o \
   imutil.o \
   pnmfile.o \
   segment-graph.o \
   segment-image.o

.cpp.o:
	$(CPP) $(CFLAGS) -c $< -o $@

libstatic: $(OBJ)
	ar rcs $(LIBNAME) $(OBJ)

clean:
	/bin/rm -f segment *.o *~ $(OBJ)

segment: segment.cpp segment-image.h segment-graph.h disjoint-set.h $(OBJ)
	$(CPP) $(CFLAGS) -o segment segment.cpp $(OBJ) $(LINK)

#all: segment

#segment: $(OBJ)
#	$(CPP) $(CFLAGS) -o segment $(OBJ) $(LINK)
	
#staticexample: static staticexample.o
#	$(CPP) $(LDFLAGS) staticexample.o -o segment -L. -linitseg

#clean-all: clean
#	/bin/rm -f *~ 

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCDIR) -o $@ -c $<



