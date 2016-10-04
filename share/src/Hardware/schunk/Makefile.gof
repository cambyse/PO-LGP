BASE   = ../../..
NAME   = $(shell basename `pwd`)
OUTPUT = libHardware_$(NAME).so

LIBS += -lG4Track -lusb-1.0

CXXFLAGS += -DMLR_SCHUNK -DWITH_ESD_CAN=1 -DOSNAME_LINUX=1 -D__LINUX__

LIBS += -lSDHLibrary-CPP -lm5api -ldevice -lutil -lntcan -lpthread -lX11 #-lvs_can_ap

DEPEND = Core

SRCS = $(shell find . -maxdepth 1 -name '*.cpp' )
OBJS = $(SRCS:%.cpp=%.o)

include $(BASE)/build/generic.mk
