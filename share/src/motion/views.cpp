#include "motion.h"

ViewInfo_typed<PoseView, arr> PoseView::staticInfo("PoseView", ViewInfo::fieldVT);

PoseView::PoseView():View(staticInfo) {
  geo.init("GeometricState", NULL); //the pose view gets itself a copy of the central ors
}

void PoseView::glInit() {
  gl->setClearColors(1.,1.,1.,1.);
  gl->camera.setPosition(10.,-15.,8.);
  gl->camera.focus(0,0,1.);
  gl->camera.upright();
  gl->update();
}

void PoseView::glDraw() {
  arr *q = (arr*)field->p;
  glStandardScene(NULL);
  if(q->N){
    geo().ors.setJointState(*q); //it's using the ors copy to interpret the pose array (field->p)
    geo().ors.calcBodyFramesFromJoints(); //it's using the ors copy to interpret the pose array (field->p)
  }
  geo().ors.glDraw(); //..and draws it
}



