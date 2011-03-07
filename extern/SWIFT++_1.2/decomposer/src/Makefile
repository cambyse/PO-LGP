.PRECIOUS: %.o

OUTPUT    = ../../bin/decomposer.exe ../../lib/libSWIFTdecomp.a
CXXFLAGS += -DSWIFT_DECOMP -O3 -fPIC -Wno-deprecated -IRAPID -I../include -I../../include
OBJS      = $(shell find . \( -name '*.cpp' \) )
OBJS     := $(OBJS:%.cpp=%.o)
LIBS	= -L../../lib -lSWIFT++ -lqhull -lm

default: $(OUTPUT)

%.exe: $(OBJS) main.o
	$(CXX) $(LDFLAGS) -o $@ $(OBJS) main.o $(LIBS)

%.a: $(OBJS)
	ar ruvs $@ $(OBJS)

%.o: %.cxx
	$(CXX) $(CXXFLAGS) -o $@ -c $<

clean::
	rm -f *.o $(OBJS) $(OUTPUT)
