#pragma once

#include "act.h"

struct Act_FollowPath : Act {
  struct sAct_FollowPath *s;

  Act_FollowPath(struct Roopi* r, const char* name, const arr& path, struct TaskMap* map, double executionTime);
  ~Act_FollowPath();

  void start();
  void stop();
};
