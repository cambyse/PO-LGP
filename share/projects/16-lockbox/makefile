BASE = ../..

OBJS = main.o swig.o interface/myBaxter.o lockbox/lockbox.o

OUTPUT := x.exe swig_wrap.cxx _swig.so

ROS = 1
DEPEND = Core Geo Control Gui RosCom Ors Motion

default: $(OUTPUT)

mySWC_LDFLAGS:=$(SWC_FLAGS) $(OBJS) -shared $(SWC_LIB_PATH) $(SWC_INCLUDES) $(DEPEND:%=-l%)

#-l$(notdir $(CURDIR))

_swig.so: swig_wrap.o
	$(CXX) $< $(mySWC_LDFLAGS) -o $@


include $(BASE)/gofMake/generic.mk

