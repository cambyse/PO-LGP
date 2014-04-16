#pragma once

#include <Core/array.h>

namespace ors {
  struct KinematicWorld;
};

inline uint pr2_q_dim(){ MT_MSG("WHAT IS THIS? :-)"); return 7; } //10; } //34; }
//arr pr2_zero_pose();
arr pr2_reasonable_W(ors::KinematicWorld& world);
uintA pr2_get_shapes(ors::KinematicWorld& world);
