OUTPUT    = liblewiner.a
CFLAGS += -O3 -fPIC
OBJS = ply.o MarchingCubes.o 


$(OUTPUT): $(OBJS)
	ar ruvs $(OUTPUT) $(OBJS)

clean::
	rm -f *.o $(OUTPUT)

depend:
