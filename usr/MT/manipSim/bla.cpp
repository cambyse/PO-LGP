#include <Optim/optimization.h>
#include <Kin/kin.h>
#include <Gui/opengl.h>

//===========================================================================

struct EndStateProgram:ConstrainedProblem{
  mlr::KinematicWorld& world;
  Graph& symbolicState;
  int verbose;
  EndStateProgram(mlr::KinematicWorld& world, Graph& symbolicState, int verbose)
    : world(world), symbolicState(symbolicState), verbose(verbose){
    ConstrainedProblem::operator=(
      [this](arr& phi, arr& J, arr& H, ObjectiveTypeA& tt, const arr& x) -> void {
        return this -> phi(phi, J, H, tt, x);
      }
    );
  }

  void phi(arr& phi, arr& phiJ, ObjectiveTypeA& tt, const arr& x){
    world.setJointState(x);
    if(verbose>1) world.gl().update();
    if(verbose>2) world.gl().watch();

    phi.clear();
    if(&phiJ) phiJ.clear();
    if(&tt) tt.clear();

    //-- support symbols -> constraints of being inside!
    Node *support=symbolicState["supports"];
    for(Node *constraint:support->parentOf){
      mlr::Body *b1=world.getBodyByName(constraint->parents(1)->keys(1));
      mlr::Body *b2=world.getBodyByName(constraint->parents(2)->keys(1));
      arr y,J;
      world.kinematicsRelPos(y, J, b1, NULL, b2, NULL);
      arr range(3);
      double d1 = .5*b1->shapes(0)->size[0] + b1->shapes(0)->size[3];
      double d2 = .5*b2->shapes(0)->size[0] + b2->shapes(0)->size[3];
      range(0) = fabs(d1 - d2);
      d1 = .5*b1->shapes(0)->size[1] + b1->shapes(0)->size[3];
      d2 = .5*b2->shapes(0)->size[1] + b2->shapes(0)->size[3];
      range(1) = fabs(d1 - d2);
      range(2)=0.;
      if(verbose>2) cout <<y <<range
          <<y-range <<-y-range
         <<"\n 10=" <<b1->shapes(0)->size[0]
        <<" 20=" <<b2->shapes(0)->size[0]
       <<" 11=" <<b1->shapes(0)->size[1]
      <<" 21=" <<b2->shapes(0)->size[1]
        <<endl;
      phi.append(  y(0) - range(0) );
      phi.append( -y(0) - range(0) );
      phi.append(  y(1) - range(1) );
      phi.append( -y(1) - range(1) );
      if(&phiJ){
        phiJ.append( J[0]);
        phiJ.append(-J[0]);
        phiJ.append( J[1]);
        phiJ.append(-J[1]);
      }
      if(&tt) tt.append(OT_ineq, 4);
    }

    //-- supporters below object -> maximize their distances
    NodeL objs=symbolicState.getNodes("Object");
    for(Node *obj:objs){
      NodeL supporters;
      for(Node *constraint:obj->parentOf){
        if(constraint->parents.N==3 && constraint->parents(0)==support && constraint->parents(2)==obj){
          supporters.append(constraint->parents(1));
        }
      }
      if(verbose>1){ cout <<"Object" <<*obj <<" is supported by "; listWrite(supporters, cout); cout <<endl; }
      if(supporters.N>=2){
        //-- compute center
        uint n=world.getJointStateDimension();
        arr cen(3),cenJ(3,n);  cen.setZero(); cenJ.setZero();
        mlr::Body *b;
        arr y,J;
        for(Node *s:supporters){
          b=world.getBodyByName(s->keys(1));
          world.kinematicsPos(y, J, b);
          cen += y;
          cenJ += J;
        }
        cen  /= (double)supporters.N;
        cenJ /= (double)supporters.N;
        //-- max distance to center
        for(Node *s:supporters){
          b=world.getBodyByName(s->keys(1));
          world.kinematicsPos(y, J, b);
          y -= cen;
          double d = length(y);
          arr normal = y/d;
          phi.append( 1.-d );
          if(&phiJ) phiJ.append( ~normal*(-J+cenJ) );
          if(&tt) tt.append(OT_sumOfSqr, 1);
        }
      }
    }

    //-- supporter above object -> center
//    objs=symbolicState.getNodes("Object");
    for(Node *obj:objs){
      NodeL supporters;
      for(Node *constraint:obj->parentOf){
        if(constraint->parents.N==3 && constraint->parents(0)==support && constraint->parents(1)==obj){
          supporters.append(constraint->parents(2));
        }
      }
      if(verbose>1){ cout <<"Object" <<*obj <<" is supported by "; listWrite(supporters, cout); cout <<endl; }

      if(false && supporters.N>=1){
        //-- compute center
        uint n=world.getJointStateDimension();
        arr cen(3),cenJ(3,n);  cen.setZero(); cenJ.setZero();
        mlr::Body *b;
        arr y,J;
        for(Node *s:supporters){
          b=world.getBodyByName(s->keys(1));
          world.kinematicsPos(y, J, b);
          cen += y;
          J += cenJ;
        }
        cen  /= (double)supporters.N;
        cenJ /= (double)supporters.N;

        //-- compare to object center
        b=world.getBodyByName(obj->keys(1));
        world.kinematicsPos(y, J, b);
        phi.append( 1e-3*(y-cen) );
        if(&phiJ) phiJ.append( 1e-3*(J-cenJ) );
        if(&tt) tt.append(OT_sumOfSqr, 3);
      }
    }

    if(&phiJ) phiJ.reshape(phi.N, x.N);
  }
  virtual uint dim_x(){ return world.getJointStateDimension(); }
  virtual uint dim_g(){ return 4; }
};

//===========================================================================

double endStateOptim(mlr::KinematicWorld& world, Graph& symbolicState){
  EndStateProgram f(world, symbolicState, 0);

  arr x = world.getJointState();
  rndGauss(x, .1, true);

  checkJacobianCP(f, x, 1e-4);
  OptConstrained opt(x, NoArr, f, OPT(verbose=1));
  opt.run();
  checkJacobianCP(f, x, 1e-4);
  world.setJointState(x);
  return opt.UCP.get_costs();
}

//===========================================================================

void createEndState(mlr::KinematicWorld& world, Graph& symbolicState){
  Node *actionSequence = symbolicState["actionSequence"];
  Node *supportSymbol  = symbolicState["supports"];
  Graph& actions = actionSequence->graph();

  for(Node *a:actions){

    //-- create a symbol that says A-on-B
    symbolicState.newNode<bool>( {}, {supportSymbol, a->parents(2), a->parents(1)}, new bool(true), true);

    //-- create a joint between the object and the target
    mlr::Shape *object= world.getShapeByName(a->parents(1)->keys(1));
    mlr::Shape *target = world.getShapeByName(a->parents(2)->keys(1));

    if(!object->body->inLinks.N){ //object does not yet have a support -> add one; otherwise NOT!
        mlr::Joint *j = new mlr::Joint(world, target->body, object->body);
        j->type = mlr::JT_transXYPhi;
        j->A.addRelativeTranslation(0, 0, .5*target->size[2]);
        j->B.addRelativeTranslation(0, 0, .5*object->size[2]);
        j->Q.addRelativeTranslation(rnd.uni(-.1,.1), rnd.uni(-.1,.1), 0.);
        j->Q.addRelativeRotationDeg(rnd.uni(-180,180), 0, 0, 1);
    }
  }

  world.topSort();
  world.checkConsistency();
}

//===========================================================================
