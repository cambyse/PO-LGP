BASE=../../../../share

DEPEND = Core Gui
QT = 1

OBJS = main.o VideoWriter_x264.o

CXXFLAGS += -DMT_IMPLEMENT_TEMPLATES -D__STDC_CONSTANT_MACROS
LPATHS += /usr/local/lib
LIBS += -lueye_api -lavformat -lavcodec -lavutil -lswscale -lx264 -lz

include $(BASE)/make-generic

# to make api-example
# gcc api-example.c -o api-example -lavcodec -lavutil -lx264 -lz
# -L/usr/local/lib -D__STDC_CONSTANT_MACROS
