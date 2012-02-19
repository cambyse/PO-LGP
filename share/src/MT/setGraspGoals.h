#include "socSystem_ors.h"

void threeStepGraspHeuristic(soc::SocSystem_Ors& sys, arr& q, const arr& q0, uint shapeId, uint verbose);

void setGraspGoals(soc::SocSystem_Ors& sys, uint T, uint shapeId, uint side, uint phase);

double KeyframeOptimizer(arr& x, soc::SocSystemAbstraction& sys, double stopTolerance, bool x_is_initialized, uint verbose);
