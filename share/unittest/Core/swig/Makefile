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

LDFLAGS+=-L../../../lib -Xlinker -rpath $(MLR_PATH)/share/lib

python:
	swig2.0 -c++ -python core_test.i
	install -D core_test.py ~/.local/lib/python2.7/site-packages/core_test.py
	$(CXX) $(EXTRA_CXXFLAGS) -c -std=c++0x -fPIC -shared core_test_wrap.cxx -I../../../src -I/usr/include/python2.7/ -DSWIG_TYPE_TABLE=mlr
	$(CXX) -std=c++0x -shared core_test_wrap.o -I../../../src -I/usr/include/python2.7/ $(LDFLAGS) -lCore -lGui -o ~/.local/lib/python2.7/site-packages/_core_test.so
	rm core_test.py

clean:
	rm -f _core_test.so core_test_wrap.*
