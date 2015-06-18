#include <Optim/optimization.h>
#include <Ors/ors.h>
#include <Gui/opengl.h>

//===========================================================================

struct EndStateProgram:ConstrainedProblemMix{
  ors::KinematicWorld& world;
  Graph& symbolicState;
  int verbose;
  EndStateProgram(ors::KinematicWorld& world, Graph& symbolicState, int verbose)
    : world(world), symbolicState(symbolicState), verbose(verbose){
    ConstrainedProblemMix::operator=(
          [this](arr& phi, arr& J, TermTypeA& tt, const arr& x) -> void {
      return this -> phi(phi, J, tt, x);
    }
    );
  }

  void phi(arr& phi, arr& phiJ, TermTypeA& tt, const arr& x){
    world.setJointState(x);
    if(verbose>1) world.gl().timedupdate(.1);
    if(verbose>2) world.gl().watch();
    double prec=0.;

    phi.clear();
    if(&phiJ) phiJ.clear();
    if(&tt) tt.clear();

    //-- support symbols -> constraints of being inside!
    Node *support=symbolicState["supports"];
    Graph& state =symbolicState["STATE"]->graph();
    for(Node *constraint:support->parentOf) if(&constraint->container==&state){
      ors::Body *b1=world.getBodyByName(constraint->parents(1)->keys(1));
      ors::Body *b2=world.getBodyByName(constraint->parents(2)->keys(1));
      if(b2->shapes(0)->type==ors::cylinderST){
        ors::Body *z=b1;
        b1=b2; b2=z;
      }//b2 should be the board
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
      prec = 1e1;
      phi.append(prec*(  y(0) - range(0) ));
      phi.append(prec*( -y(0) - range(0) ));
      phi.append(prec*(  y(1) - range(1) ));
      phi.append(prec*( -y(1) - range(1) ));
      if(&phiJ){
        phiJ.append(prec*( J[0]));
        phiJ.append(prec*(-J[0]));
        phiJ.append(prec*( J[1]));
        phiJ.append(prec*(-J[1]));
      }
      if(&tt) tt.append(ineqTT, 4);
    }

    //-- supporters below object -> maximize their distances and center
    NodeL objs=symbolicState.getNodes("Object");
    for(Node *obj:objs){
      NodeL supporters;
      for(Node *constraint:obj->parentOf){
        if(constraint->parents.N==3 && constraint->parents(0)==support && constraint->parents(2)==obj){
          supporters.append(constraint->parents(1));
        }
      }
      if(supporters.N>=2){
        if(verbose>1){ cout <<"Adding cost term Object" <<*obj <<" above "; listWrite(supporters, cout); cout <<endl; }
        //-- compute center
        uint n=world.getJointStateDimension();
        arr cen(3),cenJ(3,n);  cen.setZero(); cenJ.setZero();
        ors::Body *b;
        arr y,J;
        for(Node *s:supporters){
          b=world.getBodyByName(s->keys(1));
          world.kinematicsPos(y, J, b);
          cen += y;
          cenJ += J;
        }
        cen  /= (double)supporters.N;
        cenJ /= (double)supporters.N;

        //-- max distances to center
        prec=3e-1;
        for(Node *s:supporters){
          b=world.getBodyByName(s->keys(1));
          world.kinematicsPos(y, J, b);
          y -= cen;
          double d = length(y);
          arr normal = y/d;
          phi.append( prec*(1.-d) );
          if(&phiJ) phiJ.append( prec*(~normal*(-J+cenJ)) );
          if(&tt) tt.append(sumOfSqrTT, 1);
        }

        //-- align center with object center
        prec=1e-1;
        b=world.getBodyByName(obj->keys(1));
        world.kinematicsPos(y, J, b);
        phi.append( prec*(y-cen) );
        if(&phiJ) phiJ.append( prec*(J-cenJ) );
        if(&tt) tt.append(sumOfSqrTT, 3);
      }

      prec=1e-0;
      if(supporters.N==1){ // just one-on-one: align
        arr y1,J1,y2,J2;
        ors::Body *b1=world.getBodyByName(obj->keys(1));
        ors::Body *b2=world.getBodyByName(supporters(0)->keys(1));
        if(b1->shapes(0)->type==ors::boxST){
          if(verbose>1){ cout <<"Adding cost term Object" <<*obj <<" below "; listWrite(supporters, cout); cout <<endl; }
          world.kinematicsPos(y1, J1, b1);
          world.kinematicsPos(y2, J2, b2);
          phi.append( prec*(y1-y2) );
          if(&phiJ) phiJ.append( prec*(J1-J2) );
          if(&tt) tt.append(sumOfSqrTT, 3);
        }
      }
    }

    //-- supporters above object
    for(Node *obj:objs){
      NodeL supporters;
      for(Node *constraint:obj->parentOf){
        if(constraint->parents.N==3 && constraint->parents(0)==support && constraint->parents(1)==obj){
          supporters.append(constraint->parents(2));
        }
      }

      if(supporters.N>=2){
        if(verbose>1){ cout <<"Adding cost term Object" <<*obj <<" below "; listWrite(supporters, cout); cout <<endl; }
        //-- compute center
        uint n=world.getJointStateDimension();
        arr cen(3),cenJ(3,n);  cen.setZero(); cenJ.setZero();
        ors::Body *b;
        arr y,J;
        for(Node *s:supporters){
          b=world.getBodyByName(s->keys(1));
          world.kinematicsPos(y, J, b);
          cen += y;
          cenJ += J;
        }
        cen  /= (double)supporters.N;
        cenJ /= (double)supporters.N;

        //-- max distances to center
        prec=1e-1;
        for(Node *s:supporters){
          b=world.getBodyByName(s->keys(1));
          world.kinematicsPos(y, J, b);
          y -= cen;
          double d = length(y);
          arr normal = y/d;
          phi.append( prec*(1.-d) );
          if(&phiJ) phiJ.append( prec*(~normal*(-J+cenJ)) );
          if(&tt) tt.append(sumOfSqrTT, 1);
        }

        //-- align center with object center
        prec=1e-0;
        b=world.getBodyByName(obj->keys(1));
        world.kinematicsPos(y, J, b);
        phi.append( prec*(y-cen) );
        if(&phiJ) phiJ.append( prec*(J-cenJ) );
        if(&tt) tt.append(sumOfSqrTT, 3);
      }

      prec=1e-0;
      if(supporters.N==1){ // just one-on-one: align
        arr y1,J1,y2,J2;
        ors::Body *b1=world.getBodyByName(obj->keys(1));
        ors::Body *b2=world.getBodyByName(supporters(0)->keys(1));
        if(b1->shapes(0)->type==ors::boxST){
          if(verbose>1){ cout <<"Adding cost term Object" <<*obj <<" below "; listWrite(supporters, cout); cout <<endl; }
          world.kinematicsPos(y1, J1, b1);
          world.kinematicsPos(y2, J2, b2);
          phi.append( prec*(y1-y2) );
          if(&phiJ) phiJ.append( prec*(J1-J2) );
          if(&tt) tt.append(sumOfSqrTT, 3);
        }
      }
    }

    if(&phiJ) phiJ.reshape(phi.N, x.N);
  }
  virtual uint dim_x(){ return world.getJointStateDimension(); }
  virtual uint dim_g(){ return 4; }
};

//===========================================================================

double endStateOptim(ors::KinematicWorld& world, Graph& symbolicState){
  EndStateProgram f(world, symbolicState, 0);

  arr x = world.getJointState();
  rndGauss(x, .1, true);

  //  checkJacobianCP(f, x, 1e-4);
  OptConstrained opt(x, NoArr, f, OPT(verbose=0));
  opt.run();
  //  checkJacobianCP(f, x, 1e-4);
  world.setJointState(x);
  return opt.UCP.get_sumOfSquares();
}

//===========================================================================

void createEndState(ors::KinematicWorld& world, Graph& symbolicState){
  //  Node *actionSequence = symbolicState["actionSequence"];
  Node *supportSymbol  = symbolicState["supports"];
  Graph& state = symbolicState["STATE"]->graph();

  for(Node *s:supportSymbol->parentOf) if(&s->container==&state){

    //  }
    //  for(Node *a:actions){

    //    //-- create a symbol that says A-on-B
    //    symbolicState.append<bool>( {}, {supportSymbol, a->parents(2), a->parents(1)}, new bool(true), true);

    //-- create a joint between the object and the target
    ors::Shape *base = world.getShapeByName(s->parents(1)->keys(1));
    ors::Shape *object= world.getShapeByName(s->parents(2)->keys(1));

    if(!object->body->inLinks.N){ //object does not yet have a support -> add one; otherwise NOT!
      ors::Joint *j = new ors::Joint(world, base->body, object->body);
      j->type = ors::JT_transXYPhi;
      j->A.addRelativeTranslation(0, 0, .5*base->size[2]);
      j->B.addRelativeTranslation(0, 0, .5*object->size[2]);
      j->Q.addRelativeTranslation(rnd.uni(-.1,.1), rnd.uni(-.1,.1), 0.);
      j->Q.addRelativeRotationDeg(rnd.uni(-180,180), 0, 0, 1);
    }
  }

  world.topSort();
  world.checkConsistency();
}

//===========================================================================
