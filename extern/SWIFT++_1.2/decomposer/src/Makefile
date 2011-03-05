OUTPUT    = ../../bin/decomposer.exe
CXXFLAGS += -DSWIFT_DECOMP -g -fPIC -Wno-deprecated -I../RAPID -I../include -I../../include
OBJS      = $(shell find . \( -name '*.cpp' \) )
OBJS     := $(OBJS:%.cpp=%.o)
LIBS	= -L../RAPID -L../../lib -lRAPID -lSWIFT++ -lqhull -lm

$(OUTPUT): $(OBJS)
	$(CXX) $(LDFLAGS) -o $@ $(OBJS) $(LIBS)

clean::
	rm -f *.o $(OUTPUT)
