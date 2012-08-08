#define MT_IMPLEMENT_TEMPLATES
#include <MT/opengl.h>
#include <MT/ors.h>

void getGraspConfiguration(ors::Graph& grasp,
			   const ors::Graph& all,
			   const char* palmBodyName,
			   const char* objShapeName){
  uint i,j;
  ors::Shape *s;
  ors::Body *b;
  ors::Joint *l;

  grasp.clear();
  ors::Body *_hand = new ors::Body(grasp);
  ors::Body *_obj  = new ors::Body(grasp);
  
  ors::Body *palm = all.getBodyByName(palmBodyName);
  ors::Shape *obj   = all.getShapeByName(objShapeName);

  MT::Array<ors::Body*> handBodies;
  handBodies.append(palm);
  for_list(i,b,handBodies) for_list(j,l,b->outLinks) handBodies.append(l->to);
  cout <<"hand bodies:";  listWrite(handBodies, cout);
  cout <<endl;

  //copy hand
  _hand->X = palm->X;
  _hand->name = "hand";
  ors::Shape *_s;
  for_list(i,b,handBodies) for_list(j,s,b->shapes){
    _s = new ors::Shape(grasp, _hand, s);
    _s->rel.setDifference(_hand->X, _s->X);
  }

  //copy obj
  _obj->X = obj->X;
  _obj->name = "object";
  _s = new ors::Shape(grasp, _obj, obj);
  _s->rel.setZero();
};

void evaluate(){
  ors::Graph ors;
  OpenGL gl,gl2;
  init(ors, gl, "test.ors");
  gl.update();

  ors::Graph grasp;
  getGraspConfiguration(grasp, ors, "m9", "box");
  MT::save(grasp,"grasp.ors");
  init(grasp,gl2,NULL);
  gl2.watch();
}

int main(int argn, char** argv){
  evaluate();

  return 0;
}
