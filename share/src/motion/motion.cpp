#include "motion.h"
#include "motion_internal.h"

void reattachShape(const char* objShape, const char* toBody);

void Action::setNewAction(const ActionPredicate _action, const char *ref1, const char *ref2, const arr& locref, Process *p){
  writeAccess(p);
  action = _action;
  objectRef1 = (char*)ref1;
  objectRef2 = (char*)ref2;
  if(&locref) locationRef = locref;  
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


void MotionFuture::appendNewAction(const Action::ActionPredicate _action, const char *ref1, const char *ref2, const arr& locref, Process *p){
  
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
  MotionKeyframe *f1 = frames.append(new MotionKeyframe);   //append a new frame
  MotionPrimitive *m = motions.append(new MotionPrimitive); //append a new motion primitive
  done = false;
  
  //assign Variables
  a->setNewAction(_action, ref1, ref2, locref, p);
  a->set_frameCount(actions.N-1, p);
  f1->set_frameCount(frames.N-1, p);
  m->set_frameCount(motions.N-1, p);
  
  //create new Processes
  MotionPlanner *planner = planners.append(new MotionPlanner(*a, *f0, *f1, *m));
  
  //start the process
  planner -> threadStep();
  
  deAccess(p);

  
}
