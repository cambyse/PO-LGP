SVN_PATH     = $(shell cd ../../../../; pwd;)
SVNSRC_PATH  = $(SVN_PATH)/share/src
SRC_PATH  = ../../../src/
LIB_PATH2 = ../../../3rdparty

OBJS         =  $(SRC_PATH)/nputils.o \
                $(SRC_PATH)/cvutils.o \
                $(SRC_PATH)/imgproc.o \
                $(SRC_PATH)/bumblebee2.o \
                $(SRC_PATH)/cameracalibrator.o \
                $(SRC_PATH)/calibrationimage.o \
                $(SRC_PATH)/cvwindow.o \
                $(SRC_PATH)/pgr/pgr_conversions.o \
                $(SRC_PATH)/pgr/pgr_stereocam.o \
                $(SRC_PATH)/pgr/pgr_registers.o \
                $(SVNSRC_PATH)/MT/array.o \
                $(SVNSRC_PATH)/MT/util.o

OUTPUT       = x.exe
CXX	         = g++-4.3
LINK	       = g++-4.3

NP_DEBUG=1
NP_OPENCV=1
NP_DC1394=1
#FREEGLUT=1

include libs.mk

# export paths
CPATH	      := $(CPATH):$(SRC_PATH):$(SVNSRC_PATH)
LPATH	      := $(LPATH):$(LIB_PATH)
LD_RUN_PATH += $(LPATH)
export CPATH
export LPATH
export LD_RUN_PATH
export LIB

# additional compiler flags
CXXFLAGS_NP := -DNP_IMPLEMENT_TEMPLATES
CXXFLAGS_MT := -DMT_Linux -DMT_NOCHECK
CXXFLAGS_MT += -DMT_IMPLEMENT_TEMPLATES
CXXFLAGS    += $(CXXFLAGS_NP) $(CXXFLAGS_MT)
CXXFLAGS    += -Wall

ifdef NP_DEBUG
CXXFLAGS    += -g -fno-default-inline -pg -DNP_DEBUG
else
CXXFLAGS    += -O3 -fPIC
endif

# additional linker flags
LDFLAGS    += -lrt  $(LIBS)

all: $(OBJS) x

x: $(OBJS) main.o
	$(LINK) $(OBJS) main.o $(LDFLAGS) -o $@.exe

clean:
	rm -rf *~ *.o *.a *.so *.exe $(OBJS)

info:
	@echo "CPATH" "$(CPATH)"
	@echo "LPATH" "$(LPATH)"

%.exe: $(OBJS)
	$(LINK) $(OBJS) $<.o $(LDFLAGS) -o $@

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -o $@ -c $<

