#OUTPUT    = ../../bin/decomposer.exe
CXXFLAGS += -DSWIFT_DECOMP -g -fPIC -Wno-deprecated -I../RAPID -I../include -I../../include
OBJS      = $(shell find . \( -name '*.cpp' \) )
OBJS     := $(OBJS:%.cpp=%.o)
LIBS	= -L../RAPID -L../../lib -lRAPID -lSWIFT++ -lqhull -lm

#$(OUTPUT): $(OBJS)
#	$(CXX) $(LDFLAGS) -o $@ $(OBJS) $(LIBS)

OUTPUT    = ../../lib/libSWIFTdecomp.a
OBJS      = $(shell find . \( -name '*.cpp' -not -name 'main.cpp' \) )
OBJS     := $(OBJS:%.cpp=%.o)
$(OUTPUT): $(OBJS)
	ar ruvs $(OUTPUT) $(OBJS)

clean::
	rm -f *.o $(OUTPUT)
