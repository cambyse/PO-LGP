# SILLY makefile to wrap ors with swig.
#
# Experimental as of <2013-04-02 Tue>
#
# in python you can use ors simply by importing ors:
# >>> import corpy
# >>> help(corepy)
#
# Also take a look at share/test/orspy/

ifndef MLR_PATH
	MLR_PATH=~/git/mlr
endif

main: python

FLAGS=-std=c++0x -shared
INCLUDES=-I$(MLR_PATH)/share/src -I/usr/include/python2.7 
LIB_PATH=-L$(MLR_PATH)/share/lib -Xlinker -rpath $(MLR_PATH)/share/lib
CXXFLAGS+=-c $(FLAGS) -fPIC $(INCLUDES) -DSWIG_TYPE_TABLE=mlr
LDFLAGS+=$(FLAGS) $(LIB_PATH) $(INCLUDES) $(LIBS)
SWIG_INCLUDE=-I$(MLR_PATH)/share -I$(MLR_PATH)/share/include/numpy

python:
	swig2.0 -c++ -python $(SWIG_INCLUDE) $(MODULE_NAME).i
	install -D $(MODULE_NAME)py.py ~/.local/lib/python2.7/site-packages/$(MODULE_NAME)py.py
	$(CXX) $(CXXFLAGS) $(MODULE_NAME)_wrap.cxx 
	$(CXX) $(MODULE_NAME)_wrap.o  $(LDFLAGS) -o ~/.local/lib/python2.7/site-packages/_$(MODULE_NAME)py.so
	rm $(MODULE_NAME)py.py

clean:
	rm -f _$(MODULE_NAME)py.so $(MODULE_NAME)_wrap.*
