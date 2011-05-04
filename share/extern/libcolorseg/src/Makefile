OUTPUT    = ../lib/libcolorseg.a
CXXFLAGS += -O3 -fPIC -I../include -Wno-deprecated
OBJS      = $(shell find . \( -name '*.cpp' \) )
OBJS     := $(OBJS:%.cpp=%.o)

$(OUTPUT): $(OBJS)
	ar ruvs $(OUTPUT) $(OBJS)

clean::
	rm -f *.o $(OUTPUT)
