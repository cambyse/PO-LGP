OUTPUT    = ../../lib/liblewiner.a
CXXFLAGS += -O3 -fPIC -I../ply
OBJS = MarchingCubes.o 


$(OUTPUT): $(OBJS)
	ar ruvs $(OUTPUT) $(OBJS)

clean:
	rm -f *.o $(OUTPUT)

depend:
