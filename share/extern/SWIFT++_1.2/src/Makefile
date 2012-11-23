OUTPUT    = ../lib/libSWIFT++.a
CXXFLAGS += -O3 -fPIC -I../include -DNO_CYCLE_DETECTION -DSWIFT_DECOMP #-Wall -Wno-deprecated
OBJS      = $(shell find . \( -name '*.cpp' \) )
OBJS     := $(OBJS:%.cpp=%.o)

$(OUTPUT): $(OBJS)
	ar ruvs $(OUTPUT) $(OBJS)

clean::
	rm -f *.o $(OUTPUT)

depend: