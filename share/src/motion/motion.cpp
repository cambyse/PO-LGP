#include "motion.h"

#define VAR(Type) \
  Type *_##Type;  birosInfo.getVariable<Type>(_##Type, #Type, NULL);

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
  writeAccess(p);
  mode = MotionPrimitive::feedback;
  feedbackControlTask = &task;
  forceColLimTVs=_forceColLimTVs;
  fixFingers=_fixFingers;
  deAccess(p);
}

void MotionFuture::appendNewAction(const Action::ActionPredicate _action, const char *ref1, const char *ref2, Process *p){
  writeAccess(p);
  
  //check if at least 1 keyframe exists
  if (!frames.N){
    frames.append(new MotionKeyframe);
    VAR(HardwareReference);
    arr x0 =  _HardwareReference->get_q_reference(NULL);
    x0.append(_HardwareReference->get_v_reference(NULL));
    frames(0)->set_x_estimate(x0, p);
    frames(0)->set_converged(true, p);
  }
  
  //create new Variables
  Action *a = actions.append(new Action);
  MotionKeyframe *f0 = frames.last();
  MotionKeyframe *f1 = frames.append(new MotionKeyframe);
  MotionPrimitive *m = motions.append(new MotionPrimitive);

  //assign Variables
  a->setNewAction(_action, ref1, ref2, p);
  a->set_frameCount(actions.N-1, p);
  f1->set_frameCount(frames.N-1, p);
  m->set_frameCount(motions.N-1, p);
  
  //create new Processes
  ActionToMotionPrimitive *planner = planners.append(new ActionToMotionPrimitive(*a, *f0, *f1, *m));

  //loop the process
  planner -> threadLoopWithBeat(0.01);
  
  deAccess(p);
}
