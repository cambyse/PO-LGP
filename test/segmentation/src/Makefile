
SRC_PATH     = $(shell pwd;)/../../../src/
LIB_PATH     = $(shell pwd;)/../../../../lib/
INC_PATH     = $(shell pwd;)/../../../src/NP/

OUTPUT       = x.exe
CXX	         = g++-4.3
LINK	       = g++-4.3

# adjust these paths to fit your installations
# (and please make sure not to 'svn commit' this file again)
OPENCV_PATH   = /home/nils/uni/code/3rdparty/opencv
OPENSURF_PATH = /home/nils/uni/code/3rdparty/opensurf/
ANN_PATH      = /home/nils/uni/mlr/lib/ann-1.1/

# export paths
CPATH	      += $(INC_PATH):$(SRC_PATH):$(OPENCV_PATH)/include:$(OPENSURF_PATH):$(ANN_PATH)/include/
LPATH	      := $(LPATH):$(LIB_PATH):$(OPENCV_PATH)/lib:$(OPENSURF_PATH):$(ANN_PATH)/lib/
LD_RUN_PATH += $(LPATH)
export CPATH
export LPATH
export LD_RUN_PATH
export LIB

OBJS         = $(SRC_PATH)/NP/ucm.o $(SRC_PATH)/NP/imgproc.o $(SRC_PATH)/NP/nputils.o $(SRC_PATH)/NP/libmt.o \
               $(SRC_PATH)/NP/cvutils.o $(SRC_PATH)/NP/efd.o $(SRC_PATH)/NP/desc.o $(SRC_PATH)/NP/vocutils.o \
               $(SRC_PATH)/NP/wrappers/surf.o $(SRC_PATH)/NP/rtree.o $(SRC_PATH)/MT/infer.o


CXXFLAGS    += -O3 #-g
# CXXFLAGS    += -g -O3 -pg -fPIC -fno-default-inline -DMT_Linux -DMT_NOCHECK -DMT_IMPLEMENT_TEMPLATES
CXXFLAGS    += -Wall -fPIC -DMT_Linux -DMT_IMPLEMENT_TEMPLATES -DMT_ANN -DMT_NOCHECK
CXXFLAGS    += -DSEG_INCLUDE_HEADERS
#LDFLAGS     += -L$(LIB_PATH) -lseg_$(shell uname -m)
LDFLAGS    += -lrt -lcv -lhighgui -lml -lopensurf_x86_64 -lann11_x86_64
#-lann11-gcc432-x86_64

all: $(OBJS) img2ucm img2texturedesc ucm2tree tree_shapedesc \
 block2codebook texturedesc2xysc texturedesc2block xysc2tree_texturehist \
 tree_colordesc tree_labels tree_final_features rttrain rtpred

img2ucm: $(OBJS) img2ucm.o
	$(LINK) $(OBJS) img2ucm.o $(LDFLAGS) -o $@.exe

img2texturedesc: $(OBJS) img2texturedesc.o
	$(LINK) $(OBJS) img2texturedesc.o $(LDFLAGS) -o $@.exe

ucm2tree: $(OBJS) ucm2tree.o
	$(LINK) $(OBJS) ucm2tree.o $(LDFLAGS) -o $@.exe

tree_shapedesc: $(OBJS) tree_shapedesc.o
	$(LINK) $(OBJS) tree_shapedesc.o $(LDFLAGS) -o $@.exe

tree_colordesc: $(OBJS) tree_colordesc.o
	$(LINK) $(OBJS) tree_colordesc.o $(LDFLAGS) -o $@.exe

tree_labels: $(OBJS) tree_labels.o
	$(LINK) $(OBJS) tree_labels.o $(LDFLAGS) -o $@.exe

tree_labels2block: $(OBJS) tree_labels2block.o
	$(LINK) $(OBJS) tree_labels2block.o $(LDFLAGS) -o $@.exe

tree_final_features: $(OBJS) tree_final_features.o
	$(LINK) $(OBJS) tree_final_features.o $(LDFLAGS) -o $@.exe

tree_final_features2block: $(OBJS) tree_final_features2block.o
	$(LINK) $(OBJS) tree_final_features2block.o $(LDFLAGS) -o $@.exe

block2codebook: $(OBJS) block2codebook.o
	$(LINK) $(OBJS) block2codebook.o $(LDFLAGS) -o $@.exe

texturedesc2block: $(OBJS) texturedesc2block.o
	$(LINK) $(OBJS) texturedesc2block.o $(LDFLAGS) -o $@.exe

texturedesc2xysc: $(OBJS) texturedesc2xysc.o
	$(LINK) $(OBJS) texturedesc2xysc.o $(LDFLAGS) -o $@.exe

xysc2tree_texturehist: $(OBJS) xysc2tree_texturehist.o
	$(LINK) $(OBJS) xysc2tree_texturehist.o $(LDFLAGS) -o $@.exe

train_random_tree: $(OBJS) train_random_tree.o
	$(LINK) $(OBJS) train_random_tree.o $(LDFLAGS) -o $@.exe
	
rttrain: $(OBJS) rttrain.o
	$(LINK) $(OBJS) rttrain.o $(LDFLAGS) -o $@.exe

rtpred: $(OBJS) rtpred.o
	$(LINK) $(OBJS) rtpred.o $(LDFLAGS) -o $@.exe

segment: $(OBJS) segment.o
	$(LINK) $(OBJS) segment.o $(LDFLAGS) -o $@.exe

boostdtree: $(OBJS) boostdtree.o
	$(LINK) $(OBJS) boostdtree.o $(LDFLAGS) -o $@.exe

clean:
	rm -rf *~ *.o *.a *.so *.exe $(OBJS)

info:
	@echo "INC_PATH" "$(INC_PATH)"
	@echo "LIB_PATH" "$(LIB_PATH)"

%.exe: $(OBJS)
	$(LINK) $(OBJS) $<.o $(LDFLAGS) -o $@

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -o $@ -c $<

