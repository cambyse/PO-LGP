#pragma once

#include "act.h"

struct Act_ComPR2 : Act {
  struct sAct_ComPR2 *s;

  Act_ComPR2(Roopi *r);
  virtual ~Act_ComPR2();

  void stopSendingMotionToRobot(bool stop);

  typedef std::shared_ptr<Act_ComPR2> Ptr;
};

struct Act_RosSubKinect : Act {
  struct SubscribeRosKinect *sub;
  Act_RosSubKinect(Roopi *r);
  virtual ~Act_RosSubKinect();
};
