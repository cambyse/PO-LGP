#include "motion.h"

void MotionPlan::setClear(const arr& frame0_pose, Process *p){
  writeAccess(p);
  converged = false;
  frame0->set_x_estimate(frame0_pose, p);
  frame1->set_converged(false, p);
  deAccess(p);
}

void Action::setNewAction(const ActionPredicate _action, char *ref1, char *ref2, char *ref3, Process *p){
  writeAccess(p);
  action = _action;
  objectRef1 = ref1;
  objectRef2 = ref2;
  objectRef3 = ref3;
  executed = false;
  deAccess(p);
}

void ControllerTask::setFeedbackTask(FeedbackControlTaskAbstraction& task, bool _forceColLimTVs, bool _fixFingers, Process* p){
  writeAccess(NULL);
  mode = ControllerTask::feedback;
  feedbackControlTask = &task;
  forceColLimTVs=_forceColLimTVs;
  fixFingers=_fixFingers;
  deAccess(NULL);
}
