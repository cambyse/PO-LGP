#ifndef PR2_HEURISTICS_H
#define PR2_HEURISTICS_H

#include <Core/array.h>

namespace ors {
  struct Graph;
};

uint pr2_q_dim(){ return 7; } //10; } //34; }
//arr pr2_zero_pose();
arr pr2_reasonable_W();
uintA pr2_get_shapes(ors::KinematicWorld& G);

#endif // PR2_HEURISTICS_H
