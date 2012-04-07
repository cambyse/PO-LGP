#include "motion.h"

void Action::setNewAction(const ActionPredicate _action, const char *ref1, const char *ref2, Process *p){
  writeAccess(p);
  action = _action;
  objectRef1 = (char*)ref1;
  objectRef2 = (char*)ref2;
  executed = false;
  deAccess(p);
}

void MotionPrimitive::setClearPlanTask(const arr& frame0_pose, Process *p){
  writeAccess(p);
  planConverged = false;
  mode = MotionPrimitive::followPlan;
  frame0->set_x_estimate(frame0_pose, p);
  frame1->set_converged(false, p);
  deAccess(p);
}

void MotionPrimitive::setFeedbackTask(FeedbackControlTaskAbstraction& task, bool _forceColLimTVs, bool _fixFingers, Process* p){
  writeAccess(NULL);
  mode = MotionPrimitive::feedback;
  feedbackControlTask = &task;
  forceColLimTVs=_forceColLimTVs;
  fixFingers=_fixFingers;
  deAccess(NULL);
}
