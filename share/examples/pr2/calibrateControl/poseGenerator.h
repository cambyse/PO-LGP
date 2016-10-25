#ifndef PoseGenerator_H
#define PoseGenerator_H

#include <Core/array.h>
#include <Ors/ors.h>

struct PoseGenerator {
  ors::KinematicWorld W;
  arr q0;

  arr getRandomPose(const arr& qInit = NoArr);

  PoseGenerator(const ors::KinematicWorld& W);
};

#endif // PoseGenerator_H
