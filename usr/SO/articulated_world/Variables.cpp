#include "Variable.h"


// ============================================================================
// BirosGraphVar
BirosGraphVar::BirosGraphVar(const char* name, const char* file)
  : Variable(name)
  , glMy()
  , glPh("PhysX")
{
  reg_graph();
  graph.init(file);

  // add simple robot to graph
  ors::Body* robot = createRobot(graph);
  get_graph().calcBodyFramesFromJoints();
}
