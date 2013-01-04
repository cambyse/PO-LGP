OUTPUT    = ../../lib/libply.a
CXXFLAGS += -O3 -fPIC
OBJS = ply.o 


$(OUTPUT): $(OBJS)
	ar ruvs $(OUTPUT) $(OBJS)

clean::
	rm -f *.o $(OUTPUT)

depend: