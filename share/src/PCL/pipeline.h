#include <Core/thread.h>
#include "conv.h"

namespace mlr { struct Mesh; }
typedef mlr::Array<mlr::Mesh> MeshA;

struct PclPipeline : Thread{
  Access_typed<Pcl> inputPcl;
  Access_typed<Pcl> processedPcl;
  Access_typed<MeshA> visionDisplay;
  struct sPclPipeline *s;
  PclPipeline(const char* input_name);
  ~PclPipeline(){ threadClose(); }
  void open(){}
  void step();
  void close(){}
};

