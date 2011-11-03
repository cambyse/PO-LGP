BASE = $(HOME)/code/share

TL_PATH = $(HOME)/code/tobias/src/
CXXFLAGS += -I $(TL_PATH)
CXXFLAGS += -DMT_IMPLEMENT_TEMPLATES

OPTIM = fast
# OPTIM = debug



# CXXFLAGS += -DENFORCED_SILENCE


TL_OBJ =  \
	$(TL_PATH)TL/logicDefinitions.o \
	$(TL_PATH)TL/logicObjectManager.o \
	$(TL_PATH)TL/logicObjectDatabase.o \
	$(TL_PATH)TL/logicReasoning.o \
	$(TL_PATH)TL/ruleReasoning.o \
	$(TL_PATH)TL/ruleLearner.o \
	$(TL_PATH)TL/ruleLearner_ground.o \
	$(TL_PATH)TL/ruleExplorer.o \
	$(TL_PATH)TL/plan.o \
	$(TL_PATH)TL/prada.o \
	$(TL_PATH)TL/utilTL.o \
	$(TL_PATH)TL/ppddl.o \
	$(TL_PATH)TL/experiments_fixedContexts.o

OBJS     = \
	main.o \
	$(TL_OBJ) \
	libMT.o

OUTPUT	= x.exe

CXXFLAGS += -DMT_IMPLEMENT_TEMPLATES
# LIBS += -lrt

include $(BASE)/make-generic

TLclean:
	rm -f $(OUTPUT) main.o $(TL_OBJ)



