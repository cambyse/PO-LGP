BASE   = ../..
NAME   = libtl
OUTPUT = $(BASE)/lib/$(NAME).so

OBJS         = \
	relationalLogic.o \
        ors_actionInterface.o \
        logicEngine.o \
        ruleEngine.o \
        logic_world_interface.o \
        utilTL.o \
        plan.o \
        prada.o \
	bwLanguage.o \
	decisionMakingModule.o

#ODE = 1
QHULL = 1
SWIFT	= 1

CXXFLAGS += -fPIC
CXXFLAGS += -DMT_Linux -DMT_IMPLEMENT_TEMPLATES -DMT_NO_GLOBALMEM
#LDFLAGS += -lrt $(LIBS)

include $(BASE)/make-generic
