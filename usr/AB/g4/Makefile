BASE = ../../../share

OPENCV=1

CXXFLAGS += -I/opt/XIMEA/include -DMT_IMPLEMENT_TEMPLATES
LPATHS += /opt/XIMEA/lib

LIBS += -lXIMEA_GenTL

# ffmpeg stuff
#CXXFLAGS += -D__STDC_CONSTANT_MACROS
#LIBS += -lavutil -lavcodec -lavformat -lavdedice -lswscale

# G4 stuff
CXXFLAGS += -I/usr/local/include
LIBS += -lG4Track

include $(BASE)/make-generic
