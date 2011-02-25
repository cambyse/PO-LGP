CXXFLAGS += -O3 -fPIC -I../include -DNO_CYCLE_DETECTION #-Wall -Wno-deprecated

OBJS = fileio.o \
         lut.o \
         mesh.o \
         mesh_utils.o \
         object.o \
         pair.o \
         pqueue.o \
         scene.o

libSWIFT++.a: $(OBJS)
	ar ruvs ../lib/libSWIFT++.a $(OBJS)

clean::
	rm -f *.o ../lib/libSWIFT++.a
