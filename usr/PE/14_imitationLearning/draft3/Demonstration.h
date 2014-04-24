#ifndef DEMONSTRATION_H
#define DEMONSTRATION_H

#include <Ors/ors.h>

struct Demonstration{

//  Demonstration(arr _qTraj, arr _q0, ors::KinematicWorld _world);
  /// trajectory in joint space
  arr qTraj;
  /// start position
  arr q0;
  /// kinematic world for obstacle and goal position
  ors::KinematicWorld world;

  ~Demonstration(){};
};

#endif // DEMONSTRATION_H
