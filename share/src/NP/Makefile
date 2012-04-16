BASE   = ../..
NAME   = libnp
OUTPUT = $(BASE)/lib/$(NAME).so

MAKEMODE = mlrlib

OBJS         = \
	bumblebee2.o \
	uvccamera.o \
	camera.o \
	pgr_conversions.o \
	pgr_registers.o \
	pgr_stereocam.o
#	../MT/process.o
#	libcolorseg.o \
#	cvwindow.o \
#	cvutils.o \
#	calibrationimage.o \
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

CXXFLAGS += -DNP_IMPLEMENT_TEMPLATES

include $(BASE)/make-generic
