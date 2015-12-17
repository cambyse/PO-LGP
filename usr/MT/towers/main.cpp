#include <stdlib.h>
#include <GL/gl.h>

#include <Geo/mesh.h>
#include <Gui/opengl.h>
#include <Ors/ors.h>
#include <Motion/taskMaps.h>


struct TowerProgram:ConstrainedProblem{
  ors::KinematicWorld& world;
  Graph& logicState;
  arr x0;
  int verbose;
  TowerProgram(ors::KinematicWorld& world, Graph& symbolicState, const arr& x0, int verbose)
    : world(world), logicState(symbolicState), x0(x0), verbose(verbose){
    ConstrainedProblem::operator=(
          [this](arr& phi, arr& J, arr& H, TermTypeA& tt, const arr& x) -> void {
      return this -> phi(phi, J, H, tt, x);
    }
    );
  }

  void phi(arr& phi, arr& phiJ, TermTypeA& tt, const arr& x){
    world.setJointState(x);
    if(verbose>1) world.gl().timedupdate(.1);
    if(verbose>2) world.gl().watch();

    phi.clear();
    if(&phiJ) phiJ.clear();
    if(&tt) tt.clear();

    arr y,J;

    //-- regularization
    double prec=1e-4;
    phi.append(prec*(x-x0));
    if(&phiJ) phiJ.append(prec*eye(x.N));
    if(&tt) tt.append(sumOfSqrTT, x.N);

    //-- height -> last z
    ors::Body *last = world.bodies.last();
    world.kinematicsPos(y, (&phiJ?J:NoArr), last);
    phi.append(y(2)-6.);
    if(&phiJ) phiJ.append(J[2]);
    if(&tt) tt.append(sumOfSqrTT, 1);

    //-- height -> first z
    ors::Body *first = world.bodies(1);
    world.kinematicsPos(y, (&phiJ?J:NoArr), first);
    phi.append(y(2));
    if(&phiJ) phiJ.append(J[2]);
    if(&tt) tt.append(eqTT, 1);

    //-- touch -> GJK is zero
    if(true){
    Node *touch=logicState["touch"];
    Graph& state =logicState["STATE"]->graph();
    for(Node *constraint:touch->parentOf) if(&constraint->container==&state){
      ors::Shape *s1=world.getShapeByName(constraint->parents(1)->keys(0));
      ors::Shape *s2=world.getShapeByName(constraint->parents(2)->keys(0));

      TaskMap_GJK gjk(s1, s2, true);

      gjk.phi(y, (&phiJ?J:NoArr), world);
      phi.append(y);
      if(&phiJ) phiJ.append(J);
      if(&tt) tt.append(eqTT, y.N);
    }
    }

    if(&phiJ) phiJ.reshape(phi.N, x.N);
  }
  virtual uint dim_x(){ return world.getJointStateDimension(); }
  virtual uint dim_g(){ NIY; return 0; }
};


void optTowers() {
  ors::KinematicWorld W;
  Graph logicState;
  Node *touch = new Node_typed<bool>(logicState, {"touch"}, {}, new bool(true), true);
  Graph& state = (new Node_typed<Graph>(logicState, {"STATE"}, {}, new Graph(), true))->graph();
  //-- add random objects
  uint K=10;
  ors::Body base(W);
  for(uint k=0;k<K;k++){
    ors::Body *b = new ors::Body(W);
    ors::Joint *j = new ors::Joint(W, &base, b);
    j->type = ors::JT_free;
    j->A.addRelativeTranslation(0,0,1);
    j->Q.setRandom();
    ors::Shape *s = new ors::Shape(W, *b);
    s->name <<k;
    s->type = ors::ssCvxST;
    s->size[3] = .1;
    s->sscCore.setRandom();
    s->sscCore.scale(rnd.uni(.1,.3), rnd.uni(.1,.3), rnd.uni(.1,.3));

    new Node_typed<bool>(logicState, {s->name}, {}, new bool(true), true);
    if(k) new Node_typed<bool>(state, {}, {touch, logicState(k+1), logicState(k+2)}, new bool(true), true);
  }
  W.shapes.last()->color[0]=1.;
  W.calc_fwdPropagateFrames();
  arr x0 = W.getJointState();


  cout <<logicState <<endl;
  W.gl().update();

  arr x=x0;
  rndGauss(x, .1, true);

  //  checkJacobianCP(f, x, 1e-4);
  TowerProgram f(W, logicState, x0, 0);
  checkJacobianCP(f, x, 1e-4);
//  return;
  OptConstrained opt(x, NoArr, f, OPT(verbose=2, stopTolerance=1e-5));
  opt.run();
  //  checkJacobianCP(f, x, 1e-4);
  W.setJointState(x);
  W.gl().watch();
  cout <<opt.UCP.get_costs();
}


int MAIN(int argc, char** argv){

  optTowers();

  return 1;
}
