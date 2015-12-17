#include "towerProblem.h"
#include "LGP.h"

void TowerProblem::setRandom(){
  symbols.checkConsistency();
  Node *CYLIN = symbols["Cylin"];
  Node *BOARD = symbols["Board"];
  Node *DEPTH = symbols["depth"];
  Graph& state = symbols["STATE"]->graph();

  uint n = 10+rnd(20);
  double x=-1.6, y=-1.;
  for(uint i=0;i<n;i++){
    //add an object to the geometry
    ors::Body *b = new ors::Body(world);
    ors::Shape *s = new ors::Shape(world, *b);
    s->cont=true;
    b->X.addRelativeTranslation(x,y,.62);
    //randomize type and size
    if(rnd.uni()<.6){
      s->type = ors::cylinderST;
      s->size[0]=s->size[1]=0.;
      s->size[2]=.2;
      s->size[3]=.05;
      s->name <<"cyl_" <<i;
    }else{
      s->type = ors::boxST;
      s->size[0]=.1 + .3*rnd.uni();
      s->size[1]=.1 + .6*rnd.uni();
      s->size[2]=.02;
      s->size[3]=0.;
      s->name <<"boa_" <<i;
    }
    b->name = s->name;
    //position on grid
    b->X.addRelativeTranslation(0, .5*s->size[1], .5*s->size[2]);
    y += .1 + s->size[1]+s->size[3];
    if(y>1.){ x+=.4; y=-1.; }

    //add symbols
    Node *o = symbols.append<bool>({"Object", s->name}, {}, new bool(true), true);
    if(s->type==ors::cylinderST){
      state.append<bool>({}, {CYLIN ,o}, new bool(true), true);
    }else{
      state.append<bool>({}, {BOARD, o}, new bool(true), true);
    }
    state.append<double>({}, {DEPTH, o}, new double(0.), true);
  }

  symbols.checkConsistency();

  //HACK: move the actionSequence item to the end...
  Node *ss = symbols["STATE"];
  symbols.NodeL::append(ss);
  symbols.NodeL::remove(ss->index);
  symbols.index();

  Node *as = symbols["actionSequence"];
  symbols.NodeL::append(as);
  symbols.NodeL::remove(as->index);
  symbols.index();

  world.calc_fwdPropagateShapeFrames();
}

double TowerProblem::reward(const ors::KinematicWorld& world, const Graph& symbols){
  //-- find max depth
  double depth=0.;
  Node *depthSymbol=symbols["depth"];
  Graph& state =symbols["STATE"]->graph();

  for(Node *dep:depthSymbol->parentOf) if(&dep->container==&state){
    double *d = dep->getValue<double>();
    CHECK(d,"");
    if(*d>depth) depth=*d;
  }

  //-- count supports below
  double supp=0.;
  Node *supportSymbol=symbols["supports"];
  NodeL objs=symbols.getNodes("Object");
  for(Node *obj:objs){
    NodeL supporters;
    for(Node *constraint:obj->parentOf){
      if(constraint->parents.N==3 && constraint->parents(0)==supportSymbol && constraint->parents(2)==obj){
        supporters.append(constraint->parents(1));
      }
    }
    supp += .2 * (supporters.N * supporters.N);
  }

  return 10.*depth + supp;
}

void TowerProblem_new::setRandom(){
  fol_root.KB.checkConsistency();
  Node *CYLIN = fol_root.KB["Cylin"];
  Node *BOARD = fol_root.KB["Board"];
  Node *OBJECT = fol_root.KB["Object"];
  Graph& state = *fol_root.state;

  uint n = 2; //10+rnd(20);
  double x=-1.6, y=-1.;
  for(uint i=0;i<n;i++){
    //add an object to the geometry
    ors::Body *b = new ors::Body(world_root);
    ors::Shape *s = new ors::Shape(world_root, *b);
    s->cont=true;
    b->X.addRelativeTranslation(x,y,.62);
    //randomize type and size
    if(rnd.uni()<.6){
      s->type = ors::ssBoxST;
      s->size[0]=s->size[1]=0.;
      s->size[2]=.2;
      s->size[3]=.05;
      s->name <<"cyl_" <<i;
    }else{
      s->type = ors::ssBoxST;
      s->size[0]=.1 + .3*rnd.uni();
      s->size[1]=.1 + .6*rnd.uni();
      s->size[2]=.02;
      s->size[3]=.01;
      s->name <<"boa_" <<i;
    }
    s->sscCore.setBox();
    s->sscCore.scale(s->size[0], s->size[1], s->size[2]);
    s->mesh.setSSCvx(s->sscCore, s->size[3]);
    b->name = s->name;
    //position on grid
    b->X.addRelativeTranslation(0, .5*s->size[1], .5*s->size[2]);
    y += .1 + s->size[1]+s->size[3];
    if(y>1.){ x+=.4; y=-1.; }

    //add symbols
    Node *o = fol_root.KB.append<bool>({s->name}, {}, new bool(true), true);
    //add predicates
    state.append<bool>({}, {OBJECT, o}, new bool(true), true);
    if(!s->size[0]){
      state.append<bool>({}, {CYLIN ,o}, new bool(true), true);
    }else{
      state.append<bool>({}, {BOARD, o}, new bool(true), true);
    }
  }

  fol_root.KB.checkConsistency();

  world_root.calc_fwdPropagateShapeFrames();
}
