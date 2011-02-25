BASE         = ../..

OBJS         = \
	bumblebee2.o \
	uvccamera.o \
	camera.o \
	pgr/pgr_conversions.o \
	pgr/pgr_registers.o \
	pgr/pgr_stereocam.o \
	calibrationimage.o \
	cvwindow.o \
	cvutils.o \
	libcolorseg.o \
	../MT/threads.o
#	cameracalibrator.o \
#	camera.o \
#	desc.o \
#	efd.o \
#	imgproc.o \
#	libmt.o \
#	npors.o \
#	nputils.o \
#	rtree.o \
#	ucm.o \

OUTPUT       = libnp.so

OPENCV=1
FELZ = 1
#ANN=1

#CXX	     = g++-4.3
#LINK	     = g++-4.3

# standard linker and compiler settings
CXXFLAGS += -O3 -Wall -fPIC -DNP_DC1394 
CXXFLAGS += -DMT_Linux -DMT_IMPLEMENT_TEMPLATES -DMT_BUMBLE -DMT_NO_GLOBALMEM
CXXFLAGS += -DNP_IMPLEMENT_TEMPLATES
LDFLAGS += -lrt $(LIBS)

include $(BASE)/make-generic
