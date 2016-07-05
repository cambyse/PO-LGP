#include "taskMap_AboveBox.h"

TaskMap_AboveBox::TaskMap_AboveBox(int iShape, int jShape)
  : i(iShape), j(jShape){
}


TaskMap_AboveBox::TaskMap_AboveBox(const ors::KinematicWorld& G, const char* iShapeName, const char* jShapeName)
  :i(-1), j(-1){
  ors::Shape *a = iShapeName ? G.getShapeByName(iShapeName):NULL;
  ors::Shape *b = jShapeName ? G.getShapeByName(jShapeName):NULL;
  if(a) i=a->index;
  if(b) j=b->index;
}

void TaskMap_AboveBox::phi(arr& y, arr& J, const ors::KinematicWorld& G, int t){
  ors::Shape *s1=G.shapes(i);
  ors::Shape *s2=G.shapes(j);
  if(s2->type!=ors::ssBoxST){ //switch roles
    ors::Shape *z=s1;
    s1=s2; s2=z;
  }
  CHECK(s2->type==ors::ssBoxST,"");//s1 should be the board
  arr pos,posJ;
  G.kinematicsRelPos(pos, posJ, s1->body, s1->rel.pos, s2->body, s2->rel.pos);
  arr range(3);
  double d1 = .5*s1->size[0] + s1->size[3];
  d1 =.05; //TODO: fixed! support size/radius of object on top
  double d2 = .5*s2->size[0] + s2->size[3];
  range(0) = fabs(d1 - d2);
  d1 = .5*s1->size[1] + s1->size[3];
  d1 =.05; //TODO: fixed! support size/radius of object on top
  d2 = .5*s2->size[1] + s2->size[3];
  range(1) = fabs(d1 - d2);
  range(2)=0.;
//  if(verbose>2) cout <<pos <<range
//                    <<pos-range <<-pos-range
//                   <<"\n 10=" <<s1->size[0]
//                  <<" 20=" <<s2->size[0]
//                 <<" 11=" <<s1->size[1]
//                <<" 21=" <<s2->size[1]
//               <<endl;
  y.resize(4);
  y(0) =  pos(0) - range(0);
  y(1) = -pos(0) - range(0);
  y(2) =  pos(1) - range(1);
  y(3) = -pos(1) - range(1);
  if(&J){
    J.resize(4, posJ.d1);
    J[0] =  posJ[0];
    J[1] = -posJ[0];
    J[2] =  posJ[1];
    J[3] = -posJ[1];
  }
}
