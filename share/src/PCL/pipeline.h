#include <Core/thread.h>
#include "conv.h"

struct Percept;
typedef mlr::Array<Percept*> PerceptL;

struct PclPipeline : Thread{
  Access<Pcl> inputPcl;
  Access<Pcl> processedPcl;
  Access<PerceptL> percepts_input;
  PclPipeline(const char* input_name);
  ~PclPipeline();
  void open(){}
  void step();
  void close(){}
};


PerceptL PclScript_Z_plane_cluster_planes_boxes(const Pcl* newInput, bool verbosePercepts=false);
