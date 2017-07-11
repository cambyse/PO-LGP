#ifndef PoseGenerator_H
#define PoseGenerator_H

#include <Core/array.h>
#include <Kin/kin.h>

struct PoseGenerator {
  mlr::KinematicWorld W;
  arr q0;

  arr getRandomPose(const arr& qInit = NoArr);

  PoseGenerator(const mlr::KinematicWorld& W);
};

#endif // PoseGenerator_H
