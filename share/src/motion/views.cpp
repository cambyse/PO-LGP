#include "motion.h"

REGISTER_VIEW(PoseView, arr/*, fieldVT*/);

PoseView::PoseView():View() {
  geo.init("GeometricState", NULL); //the pose view gets itself a copy of the central ors
}

PoseView::PoseView(FieldRegistration* field, GtkWidget *container):View(field) {
  geo.init("GeometricState", NULL); //the pose view gets itself a copy of the central ors
  gtkNewGl(container);
}

void PoseView::glInit() {
  gl->setClearColors(1.,1.,1.,1.);
  gl->camera.setPosition(10.,-15.,8.);
  gl->camera.focus(0,0,1.);
  gl->camera.upright();
  gl->update();
}

void PoseView::glDraw() {
  arr q = *(arr*) ((FieldRegistration*)object)->p; //copy!
  geo.pull();
  uint n=geo().ors.getJointStateDimension();
  if(q.nd==1){
    if (q.N==2*n) q = q.sub(0,q.N/2-1); //check dynamic state
    if (q.N!=n){ MT_MSG("pose view on wrong dimension");  return; }
    geo().ors.setJointState(q); //it's using the ors copy to interpret the pose array (field->p)
    geo().ors.calcBodyFramesFromJoints(); //it's using the ors copy to interpret the pose array (field->p)
  }
  if(q.nd==2){
    t++;
    if(t>=q.d0) t=0;
    geo().ors.setJointState(q[t]);
    geo().ors.calcBodyFramesFromJoints();
  }
  glStandardScene(NULL);
  geo().ors.glDraw(); //..and draws it
}



