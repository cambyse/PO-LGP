
ifdef NP_OPENCV
	CXXFLAGS      += -DNP_OPENCV
	ifdef NP_DEBUG
		CPATH       := $(CPATH):$(LIB_PATH2)/opencv/debug/include/
		LPATH       := $(LPATH):$(LIB_PATH2)/opencv/debug/lib/
	else
		CPATH       := $(CPATH):$(LIB_PATH2)/opencv/release/include/
		LPATH       := $(LPATH):$(LIB_PATH2)/opencv/release/lib/
	endif
	LIBS          += -lcv -lhighgui -lml -lcvaux -lcxcore
endif

ifdef NP_DC1394
	CXXFLAGS      += -DNP_DC1394
	CPATH       := $(CPATH)
	LPATH       := $(LPATH)
	LIBS        += -ldc1394
endif

###

ifdef FREEGLUT
	CXXFLAGS      += -DMT_FREEGLUT
	CPATH         := $(CPATH)
	LPATH         := $(LPATH)
	LIBS          += -lglut -lGLU -lGL -lX11
endif
