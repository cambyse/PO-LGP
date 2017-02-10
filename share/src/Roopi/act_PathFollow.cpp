#include "act_PathFollow.h"
#include "act_CtrlTask.h"

#include <Control/taskController.h>

Act_FollowPath::Act_FollowPath(Roopi *r, const char* name, const arr& path, TaskMap* map, double executionTime)
  : Act_CtrlTask(r){

  setMap(map);

  arr path2 = path;
  uint qdim = y0.N;
  path2.reshape(path.N/qdim, qdim);

  task->ref = new MotionProfile_Path(path2, executionTime);
}
