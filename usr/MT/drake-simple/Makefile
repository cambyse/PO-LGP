BASE = ../../../share

OBJS = main.o drake.o

DEPEND = Core Algo

MLR_NO_CXX11 = 1
CXXFLAGS += -DBT_USE_DOUBLE_PRECISION -DHAVE_SPDLOG -DSPDLOG_FMT_EXTERNAL -std=gnu++14

CPATH := $(CPATH):/opt/drake/include:/opt/drake/include/drake/lcmtypes:/opt/drake/include/lcmtypes:/opt/drake/include/bullet:/opt/drake/include/eigen3:/opt/drake/include/nlopt
LPATHS += /opt/drake/lib

LIBS += -ldrake -lBulletCollision -lLinearMath -lfcl -lccd -loctomap -llcm -lglib-2.0 -lpthread -lnlopt -lprotobuf -lsdformat -lignition_math -ltinyxml -lfmt -lyaml_cpp -lz


include $(BASE)/build/generic.mk
