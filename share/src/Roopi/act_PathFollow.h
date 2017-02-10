#pragma once

#include "act.h"
#include "act_CtrlTask.h"

struct Act_FollowPath : Act_CtrlTask {
  Act_FollowPath(struct Roopi* r, const char* name, const arr& path, struct TaskMap* map, double executionTime);
};
