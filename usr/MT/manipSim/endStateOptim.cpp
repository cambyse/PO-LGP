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
    if(verbose>1) world.gl().update();
    if(verbose>2) world.gl().watch();

    phi.clear();
    if(&phiJ) phiJ.clear();
    if(&tt) tt.clear();

    //-- cost
//    arr y,J;
//    world.kinematicsPos(y, J, obj);
////    cout <<"QUERY: pos=" <<y <<endl;
//    phi.append(y-target);
//    if(&tt) tt.append(sumOfSqrTT, y.N);
//    if(&phiJ) phiJ.append(J);

    //-- support symbols -> overlap constraints
    Item *support=symbolicState["supports"];
    for(Item *constraint:support->parentOf){
      ors::Body *b1=world.getBodyByName(constraint->parents(1)->keys(1));
      ors::Body *b2=world.getBodyByName(constraint->parents(2)->keys(1));
      arr y,J;
      world.kinematicsRelPos(y, J, b1, NULL, b2, NULL);
      arr range(3);
      range(0) = .5*fabs(b1->shapes(0)->size[0] - b2->shapes(0)->size[0]);
      range(1) = .5*fabs(b1->shapes(0)->size[1] - b2->shapes(0)->size[1]);
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
      if(&tt) tt.append(ineqTT, 4);
    }

    //-- support -> maximize distances
    ItemL objs=symbolicState.getItems("Object");
    for(Item *obj:objs){
      ItemL supporters;
      for(Item *constraint:obj->parentOf){
        if(constraint->parents.N==3 && constraint->parents(0)==support && constraint->parents(2)==obj){
          supporters.append(constraint->parents(1));
        }
      }
      if(verbose>1){ cout <<"Object" <<*obj <<" is supported by "; listWrite(supporters, cout); cout <<endl; }
      if(supporters.N==2){
        ors::Body *b1=world.getBodyByName(supporters(0)->keys(1));
        ors::Body *b2=world.getBodyByName(supporters(1)->keys(1));
        arr y1,y2,J1,J2;
        world.kinematicsPos(y1, J1, b1);
        world.kinematicsPos(y2, J2, b2);
        arr y = y1-y2;
        double d = length(y);
        arr normal = y/d;
        phi.append( 1.-d );
        if(&phiJ){
          arr J = ~normal*(-J1+J2);
          phiJ.append( J );
        }
        if(&tt) tt.append(sumOfSqrTT, 1);
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

  OptConstrained opt(x, NoArr, f, OPT(verbose=1));
  opt.run();
  f.world.setJointState(x);
  return opt.UCP.get_sumOfSquares();
}

//===========================================================================

