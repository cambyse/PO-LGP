#include <Core/thread.h>
#include "conv.h"

namespace mlr { struct Mesh; }
struct Percept;
typedef mlr::Array<mlr::Mesh> MeshA;
typedef mlr::Array<Percept*> PerceptL;

struct PclPipeline : Thread{
  Access_typed<Pcl> inputPcl;
  Access_typed<Pcl> processedPcl;
  Access_typed<PerceptL> percepts_input;
  struct sPclPipeline *s;
  PclPipeline(const char* input_name);
  ~PclPipeline();
  void open(){}
  void step();
  void close(){}
};


PerceptL PclScript_Z_plane_cluster_planes_boxes(const Pcl* newInput, bool verbosePercepts=false);