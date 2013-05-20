.PRECIOUS: %.o


OUTPUT    = ../../bin/decomposer.exe ../../../../lib/libSWIFTdecomp.a
CXXFLAGS += -DSWIFT_DECOMP -DDECOMP_GRAPHICS -O3 -fPIC -Wno-deprecated -Wno-write-strings -IRAPID -I../include -I../../include -I/usr/include/tcl8.5
OBJS      = $(shell find . \( -name '*.cpp' \) )
OBJS     := $(OBJS:%.cpp=%.o)
LIBS	= -L../../../../lib -lSWIFT++ -lqhull -lm -ltcl -ltk -lglut -lGLU -lXmu
# Fixes compiling problem (occurred only with Baisero), possibly
# due to Ubuntu 13.04
LIBS += -lX11 -lGL

default: $(OUTPUT)

%.exe: $(OBJS) main.o
	$(CXX) $(LDFLAGS) -o $@ $(OBJS) main.o $(LIBS)

%.a: $(OBJS)
	ar ruvs $@ $(OBJS)

%.o: %.cxx
	$(CXX) $(CXXFLAGS) -o $@ -c $<

clean::
	rm -f *.o $(OBJS) $(OUTPUT)
