#include "motion.h"

#define VAR(Type) \
  Type *_##Type;  birosInfo.getVariable<Type>(_##Type, #Type, NULL);

void reattachShape(const char* objShape, const char* toBody);
void waitForDoneMotionPrimitive(MotionPrimitive *motionPrimitive);

struct sActionProgressor {
  MotionFuture *motionFuture;
};

ActionProgressor::ActionProgressor():Process("ActionProgressor") {
  s = new sActionProgressor();
  birosInfo.getVariable(s->motionFuture, "MotionFuture", this);
}

ActionProgressor::~ActionProgressor() {
  delete s;
}

void ActionProgressor::open(){
}

void ActionProgressor::step(){
  if(s->motionFuture->getTodoFrames(this) == 0) return;
  MotionPrimitive *motionPrimitive = s->motionFuture->getCurrentMotionPrimitive(this);
  Action *action = s->motionFuture->getCurrentAction(this);
  
  switch(action->get_action(this)){
    case Action::grasp: {
      waitForDoneMotionPrimitive(motionPrimitive);
      reattachShape(action->get_objectRef1(this), "m9");
    } break;
    case Action::place: {
      waitForDoneMotionPrimitive(motionPrimitive);
      reattachShape(action->get_objectRef1(this), "OBJECTS");
    } break;
    case Action::closeHand:
    case Action::openHand: {
      MT::wait(3.);
      motionPrimitive->set_mode(MotionPrimitive::done, this);
    } break;
    default:
      HALT("");
  }
  
  int rev = 0;
  while(s->motionFuture->getTodoFrames(NULL)<=1){
    rev = s->motionFuture->waitForRevisionGreaterThan(rev);
  }
  
  s->motionFuture->writeAccess(this);
  //reset the frame0 of the motion primitive!
  VAR(HardwareReference);
  arr x0 =  _HardwareReference->get_q_reference(NULL);
  x0.append(_HardwareReference->get_v_reference(NULL));
  s->motionFuture->currentFrame++;
  //s->motionFuture->frames(s->motionFuture->currentFrame)->set_x_estimate(x0, this);
  //Process *p = new PoseViewer<MotionKeyframe> (*s->motionFuture->frames(s->motionFuture->currentFrame));
  //p->threadStep();
  //s->motionFuture->frames(s->motionFuture->currentFrame)->set_converged(true, this);
  //s->motionFuture->frames(s->motionFuture->currentFrame+1)->set_converged(true, this);
  s->motionFuture->motions(s->motionFuture->currentFrame)->set_planConverged(false, this);
  s->motionFuture->deAccess(this);

}

void ActionProgressor::close(){
}


void waitForDoneMotionPrimitive(MotionPrimitive *motionPrimitive){
  int rev = 0;
  while(motionPrimitive->get_mode(NULL)!=MotionPrimitive::done){
    rev = motionPrimitive->waitForRevisionGreaterThan(rev);
  }
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
  Type *_##Type;  birosInfo.getVariable<Type>(_##Type, #Type, NULL);

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
