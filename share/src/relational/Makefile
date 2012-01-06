BASE   = ../..
NAME   = librelational
OUTPUT = $(BASE)/lib/$(NAME).so

#OPTIM = fast

CXXFLAGS += -fPIC -DMT_IMPLEMENT_TEMPLATES #causes all *.cpp to implement the used templates
#CXXFLAGS += -fPIC 
OBJS  = $(shell find . \( -name '*.cpp' \) )
OBJS := $(OBJS:%.cpp=%.o)
OBJS_C  = $(shell find . -name '*.c')
OBJS += $(OBJS_C:%.c=%.o)

#FREEGLUT = 1
ODE = 1
QHULL = 1
SWIFT	= 1
#SOIL = 1

include $(BASE)/make-generic

# CXXFLAGS += -DMT_Linux -DMT_IMPLEMENT_TEMPLATES -DMT_NO_GLOBALMEM
#LDFLAGS += -lrt $(LIBS)

