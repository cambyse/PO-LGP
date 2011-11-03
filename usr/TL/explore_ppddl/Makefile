BASE = $(shell pwd)/../../../share
OBJS = main.o

# OPTIM = fast
OPTIM = debug

TOBIAS = 1

CXXFLAGS += -fPIC -DMT_IMPLEMENT_TEMPLATES #causes all *.cpp to implement the used templates
# CXXFLAGS += -DENFORCED_SILENCE

include $(BASE)/make-generic

# Hand-selected files instead of library:
#
# TL_OBJ =  \
# 	$(TL_PATH)TL/logicDefinitions.o \
# 	$(TL_PATH)TL/logicObjectManager.o \
# 	$(TL_PATH)TL/logicObjectDatabase.o \
# 	$(TL_PATH)TL/logicReasoning.o \
# 	$(TL_PATH)TL/ruleReasoning.o \
# 	$(TL_PATH)TL/ruleLearner.o \
# 	$(TL_PATH)TL/ruleLearner_ground.o \
# 	$(TL_PATH)TL/ruleExplorer.o \
# 	$(TL_PATH)TL/plan.o \
# 	$(TL_PATH)TL/prada.o \
# 	$(TL_PATH)TL/utilTL.o \
# 	$(TL_PATH)TL/ppddl.o \
# 	$(TL_PATH)TL/experiments_fixedContexts.o
#
# OBJS     = \
# 	main.o \
# 	$(TL_OBJ) \
# 	libMT.o
