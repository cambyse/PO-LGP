#include "motion_internal.h"

#define VAR(Type) \
  Type *_##Type;  birosInfo().getVariable<Type>(_##Type, #Type, NULL);

void reattachShape(const char* objShape, const char* toBody);

Process* newActionProgressor(MotionFuture& a){
  return new ActionProgressor(a);
}

ActionProgressor::ActionProgressor(MotionFuture& a)
:Process("ActionProgressor"), motionFuture(&a) {
  if(!motionFuture) birosInfo().getVariable(motionFuture, "MotionFuture", this);
  threadListenTo(motionFuture);
  MotionPrimitive *motionPrimitive = motionFuture->getCurrentMotionPrimitive(this);
  if(motionPrimitive) threadListenTo(motionPrimitive);
}

void ActionProgressor::step(){
  //TODO: the progressor shouldn't wait itself - it should be triggered listening to the primitive and future
  
  if(motionFuture->getTodoFrames(this) <= 1) return; //no future to progress to
  MotionPrimitive *motionPrimitive = motionFuture->getCurrentMotionPrimitive(this);
  threadListenTo(motionPrimitive);
  Action *action = motionFuture->getCurrentAction(this);
  
  if(motionPrimitive->get_mode(this) != MotionPrimitive::done) return;
  switch(action->get_action(this)){ //wait, depending on current action
    case Action::grasp: 
      reattachShape(action->get_objectRef1(this), "m9");
      break;
    case Action::place_location: 
    case Action::place:
      reattachShape(action->get_objectRef1(this), "OBJECTS");
      break;
  default: NIY;
  }

  motionFuture->incrementFrame(this);
  MotionPrimitive *mp = motionFuture->getCurrentMotionPrimitive(this);
  threadListenTo(mp);
  
  //reset the frame0 of the motion primitive to the real hardware pose! -> triggers the MotionPlanner to refine!
  MotionFuture *f = motionFuture;
  f->writeAccess(this);
  VAR(HardwareReference);
  arr x0 =  _HardwareReference->get_q_reference(this);
  x0.append(_HardwareReference->get_v_reference(this));
  x0.subRange(x0.N/2,-1) = 0.;
  //cout <<"0-state! in motion progressor\n" <<x0 <<"\n ...frame=" <<f->currentFrame <<endl;
  f->frames(f->currentFrame)->set_x_estimate(x0, this);
  //f->frames(f->currentFrame)->set_converged(true, this);
  f->frames(f->currentFrame+1)->set_converged(false, this);
  f->motions(f->currentFrame)->set_planConverged(false, this);
  _HardwareReference->set_motionPrimitiveRelativeTime(0., this);
  f->deAccess(this);
}

void reattachShape(ors::Graph& ors, SwiftInterface *swift, const char* objShape, const char* toBody){
  ors::Shape *obj  = ors.getShapeByName(objShape);
  obj->body->shapes.removeValue(obj);
  obj->body = ors.getBodyByName(toBody);
  obj->ibody = obj->body->index;
  obj->body->shapes.append(obj);
  obj->rel.setDifference(obj->body->X, obj->X);
  if(swift) swift->initActivations(ors);
}

#define VAR(Type) \
  Type *_##Type;  birosInfo().getVariable<Type>(_##Type, #Type, NULL);

void reattachShape(const char* objShape, const char* toBody){
  VAR(GeometricState);
  VAR(HardwareReference);
  _GeometricState->writeAccess(NULL);
  arr q=_HardwareReference->get_q_reference(NULL);
  arr v=_HardwareReference->get_v_reference(NULL);
  _GeometricState->ors.setJointState(q,v);
  _GeometricState->ors.calcBodyFramesFromJoints();
  reattachShape(_GeometricState->ors, NULL, objShape, toBody);
  _GeometricState->deAccess(NULL);
}
