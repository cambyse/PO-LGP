#include "motion.h"

#define VAR(Type) \
  Type *_##Type;  birosInfo.getVariable<Type>(_##Type, #Type, NULL);

void reattachShape(const char* objShape, const char* toBody);

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

void waitForDoneMotionPrimitive2(MotionPrimitive *motionPrimitive){
  int rev = 0;
  while(motionPrimitive->get_mode(NULL)!=MotionPrimitive::done){
    rev = motionPrimitive->waitForRevisionGreaterThan(rev);
  }
}

void MotionFuture::appendNewAction(const Action::ActionPredicate _action, const char *ref1, const char *ref2, Process *p){
  
#if 0 //this is the version that should actually work - but has problems with the optimization
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
  done = false;
  
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

#else //this is the strictly sequential (old) version

  writeAccess(p);
  
  if(!frames.N){ //first call -- build new variables and processes
    frames.append(new MotionKeyframe);
    frames.append(new MotionKeyframe);
    actions.append(new Action);
    motions.append(new MotionPrimitive);
    planners.append(new ActionToMotionPrimitive(*actions(0), *frames(0), *frames(1), *motions(0)));
    planners(0) -> threadLoopWithBeat(0.01);
  }
  
  Action *a = actions(0);
  MotionKeyframe *f0 = frames(0);
  MotionKeyframe *f1 = frames(1);
  MotionPrimitive *m = motions(0);
  ActionToMotionPrimitive *planner = planners(0);
  
  VAR(HardwareReference);
  arr x0 =  _HardwareReference->get_q_reference(p);
  x0.append(_HardwareReference->get_v_reference(p));
  f0->set_x_estimate(x0, p);
  f0->set_converged(true, p);
  f1->set_converged(false, p);
  m->set_planConverged(false, p);
  m->set_mode(MotionPrimitive::stop, p);
  
  a->setNewAction(_action, ref1, ref2, p);
  
  deAccess(p);
  
  if(_action == Action::grasp || _action==Action::place)
    waitForDoneMotionPrimitive2(m);
  if(_action == Action::grasp) reattachShape(a->get_objectRef1(p), "m9");
  if(_action == Action::place) reattachShape(a->get_objectRef1(p), "OBJECTS");

  if(_action == Action::openHand || _action==Action::closeHand)
    MT::wait(3.);
  
#endif
  
}
