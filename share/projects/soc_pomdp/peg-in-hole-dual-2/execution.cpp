#include "execution.h"
#include "pomdp.h"
#include <Motion/taskMap_proxy.h>
#include <Ors/ors_swift.h>
#include <Core/geo.h>
#include <vector>


using namespace std;


void getTrajectory(arr& x, arr& y, arr& dual, ors::KinematicWorld& world, arr x0, const arr& model, bool stickyness, uint horizon){
    /////////////

  //SET INITIAL STATE: Model parameter (height, position, size,...), and initial joint's position
    world.setJointState(x0);
    world.getBodyByName("hole")->X.pos.z   = model(0);
    world.getBodyByName("target")->X.pos.z = model(0);

    world.getBodyByName("hole")->X.pos.x   = model(1);
    world.getBodyByName("target")->X.pos.x = model(1);



  /////////////////////////////////////////////////////
  ////////////////////////////////////////////////////

  MotionProblem P(world, true); //true for using swift
  P.loadTransitionParameters(); // can change horizon hereP

  P.T = horizon;
  x = P.getInitialization();

  //-- setup the motion problem


  TaskCost *c;

  TaskCost *pos = P.addTask("position", new DefaultTaskMap(posTMT, world, "peg", NoVector, "target", NoVector));
  P.setInterpolatingCosts(pos, MotionProblem::finalOnly,ARRAY(0.,0.,0.), 2e5);

  TaskCost *vel = P.addTask("position_vel", new DefaultTaskMap(posTMT, world, "peg", NoVector));
  vel->map.order=1;
  P.setInterpolatingCosts(vel, MotionProblem::finalOnly, ARRAY(0.,0.,0.), 1e3);

  //see taskmap_default.cpp;
  TaskCost *vec = P.addTask("orientation", new DefaultTaskMap(vecTMT, world, "peg",ARRAY(0.,0.,1.)));
  //P.setInterpolatingCosts(vec, MotionProblem::finalOnly, ARRAY(0.,0.,-1.), 1e3, ARRAY(0.,0.,0.), 1e-3);
  P.setInterpolatingCosts(vec, MotionProblem::early_restConst, ARRAY(0.,0.,-1.), 1e3, NoArr, -1., 0.1);



#if 1  //CONSTRAINT
  TaskCost *collision = P.addTask("collisionConstraint", new CollisionConstraint());
  P.setInterpolatingCosts(collision, MotionProblem::constant, ARRAY(0.), 1e3);
#else
  c = P.addTask("collision", new ProxyTaskMap(allPTMT, {0}, .041));
  P.setInterpolatingCosts(c, MotionProblem::constant, ARRAY(0.), 1e1);
#endif

  if(stickyness){
    TaskCost *cons = P.addTask("planeConstraint", new PlaneConstraint(world, "peg", ARR(0,0,-1, world.getBodyByName("hole")->X.pos.z + 0.2 + .05))); //plane is 0.2 (table's width = 0.4) above the hole->z
    //P.setInterpolatingCosts(cons, MotionProblem::constant, ARRAY(0.), 1e1);
    cons->setCostSpecs(0, P.T, {0.}, 1.);
    TaskCost *sticky = P.addTask("planeStickiness", new ConstraintStickiness(cons->map));
    sticky->setCostSpecs(0, P.T, {0.}, 1.);

    P.makeContactsAttractive = true;
  }else{
    //stickyWeight = 0.;
    P.makeContactsAttractive = false;
  }


  MotionProblemFunction MF(P);
  Convert ConstrainedP(MF);

  UnconstrainedProblem UnConstrainedP(ConstrainedP);
  UnConstrainedP.mu = 10.;



  for(uint k=0;k<5;k++){
   optNewton(x, UnConstrainedP, OPT(verbose=0, stopIters=200, damping=1e-3, stopTolerance=1e-5, maxStep=.5));
    P.costReport(false);
//    displayTrajectory(x, 1, G, gl,"planned trajectory");
    UnConstrainedP.aulaUpdate(.9,x);
    P.dualMatrix = UnConstrainedP.lambda;
    UnConstrainedP.mu *= 2.;
 }

  //get the final optimal cost at each time slice
  P.costReport(false);

  if(&y){
    y.resize(x.d0, pos->map.dim_phi(world));

    for(uint t=0;t<x.d0;t++){
      world.setJointState(x[t]);
      pos->map.phi(y[t](), NoArr, world);
    }
  }

  ///cout<< UnConstrainedP.lambda<<endl;

  uint index = 0;
  dual.resize(x.d0);
  //extract dual information
  if(&dual) {
      for(int i=0;i<UnConstrainedP.lambda.d0;i=i+2){
          dual(index) = UnConstrainedP.lambda(i);

          index++;
      }
  }

  //cout<< dual <<endl;
}


// DUAL-EXECUTION
void POMDPExecution(const arr& x, const arr& y, const arr& dual, ors::KinematicWorld& world, int num){
  arr q, qdot;
  world.getJointState(q, qdot);

  ofstream data(STRING("data-"<<num<<".dat"));

  ors::Shape *endeff = world.getShapeByName("peg");
  ors::Shape *true_target = world.getShapeByName("target");
  ors::Body *est_target = world.getBodyByName("target");
  ors::Body *table = world.getBodyByName("hole");
  double mean_table_height = table->X.pos.z;

  double sin_jitter = MT::getParameter<double>("sin_jitter", 0.);

  FeedbackMotionControl MC(world);
  MC.qitselfPD.active=false;

  //position PD task:  decayTime = 0.1, dampingRatio = 0.8
  PDtask *pd_y =  MC.addPDTask("position", .1, .8, new DefaultTaskMap(posTMT, world, "peg", NoVector, "target"));
  pd_y->prec = 10.;


  //joint space PD task
  PDtask *pd_x = MC.addPDTask("pose", .1, .8, new DefaultTaskMap(qItselfTMT, world));
  pd_x->prec = .1;

  //plane constraint task
#define USE_DUAL
#ifdef USE_DUAL
  PlaneConstraint *plane_constraint = new PlaneConstraint(world, "peg", ARR(0,0,-1, table->X.pos.z + 0.2 + 0.05));
  ConstraintForceTask *pd_c = MC.addConstraintForceTask("planeConstraint", plane_constraint );

  MC.addPDTask("collisions", .2, .8, collTMT, NULL, NoVector, NULL, NoVector, {.1});

#endif

  double tau = 0.01;

  cout<< dual.d0 << "  "<<x.d0<<endl;

  // remaining 100 steps is for reaching to the target.
  for(uint t=0;t<x.d0 + 100;t++){
    MC.setState(q, qdot);

   //adapt the PD task references following the plan


   if(t<y.d0){
        pd_y->y_ref = y[t];
        pd_x->y_ref = x[t];
  #ifdef USE_DUAL
        pd_c->desiredForce = dual(t);
  #endif
    }

#ifdef USE_DUAL
    //recalibrate the target based on touch
    double d=0.;
    if(pd_c->desiredApproach.y.N){
      d = pd_c->desiredApproach.y(0); //d = distance measured by constraint task
      if(pd_c->desiredApproach.y_ref(0)==0. && d<1e-2){
        est_target->X.pos.z = endeff->X.pos.z - 0.2; //est_target position update
      }
    }
#endif


 //   //external sinus on the table height
 //   table->X.pos.z = mean_table_height+sin_jitter*::sin(double(t)/15);
#ifdef USE_DUAL
//    plane_constraint->planeParams(3) = table->X.pos.z + 0.2; //0.2 + 0.02
#endif

    //operational space loop
    for(uint tt=0;tt<10;tt++){
      MC.updateConstraintControllers();
      arr a = MC.operationalSpaceControl();
      q += .1*tau*qdot;
      qdot += .1*tau*a;
    }

    //display and record video
//    world.watch(false, STRING(t));
    world.gl().update(STRING(t), true, false, true);
    //    flip_image(world.gl().captureImage);
    //    vid->addFrame(world.gl().captureImage);

    //write data
    MT::arrayBrackets="  ";
    data <<t <<' ' <<(t<dual.N?dual(t):0.) <<' '
        <<table->X.pos.z <<' '
       <<endeff->X.pos.z <<' '
      <<endeff->X.pos.z-table->X.pos.z <<' '
      <<est_target->X.pos.z <<' '
     <<true_target->X.pos.z <<' '
    <<endl;
  }
  data.close();

  FILE(STRING("data-"<<num<<"-err.dat")) << ARRAY(true_target->X.pos)- ARRAY(endeff->X.pos);
}



  /// Online execution: Using POMDP policy (solve the POMDP online, using offline value functions from SOC)
void POMDPExecution(FSC fsc, ors::KinematicWorld& world, int num, double est){



    arr q, qdot;


    ofstream data(STRING("data-"<<num<<".dat"));

    ors::Shape *endeff = world.getShapeByName("peg");
    ors::Shape *true_target = world.getShapeByName("target");
    ors::Body *est_target = world.getBodyByName("target");
    ors::Body *table = world.getBodyByName("hole");


    //est_target->X.pos.z  = est;


    FeedbackMotionControl MC(world);
    MC.qitselfPD.active=false;

    //position PD task:  decayTime = 0.1, dampingRatio = 0.8
    PDtask *pd_y =  MC.addPDTask("position", .1, .8, new DefaultTaskMap(posTMT, world, "peg", NoVector, "target"));
    pd_y->prec = 10.;

    //joint space PD task
    PDtask *pd_x = MC.addPDTask("pose", .1, .8, new DefaultTaskMap(qItselfTMT, world));
    pd_x->prec = .1;

    //plane constraint task
  #define USE_DUAL
  #ifdef USE_DUAL
    PlaneConstraint *plane_constraint = new PlaneConstraint(world, "peg", ARR(0,0,-1,table->X.pos.z + 0.02 + 0.05));
    ConstraintForceTask *pd_c =
        MC.addConstraintForceTask("planeConstraint", plane_constraint );


    //collision avoidance
    MC.addPDTask("collisions", .2, .8, collTMT, NULL, NoVector, NULL, NoVector, {.1});
  #endif



    double tau = 0.01;

    NODE*Root = fsc.getRoot();
    arr x = Root->AllX();
    arr y = Root->AllY();
    arr dual = Root->AllDual();

    //world.setJointState(x[0]);
    world.getJointState(q, qdot);

    double estimate_height = Root->Model()(0);


    cout<<"the robot thinks the height is "<<estimate_height<<endl;

    cout<< "starting joint "<<q<<endl;


    arr y_p = y[0];


    // remaining 100 steps is for reaching to the target.
    cout<<endl<<"num "<<num<<endl;
    for(uint t=0;t<x.d0 + 100;t++){
      MC.setState(q, qdot);

     //double observation = pd_c->infraRed_obs;


      //scan all next node
       if(t<y.d0-2){
          double current_pos_eff = endeff->X.pos.z;
          int best_obs = 0;
          double best_distance = 10000.;
          int numchild = Root->Childrens.size();
          if(numchild>1){
              for(int no=0; no < Root->Childrens.size();no++){
                  //Observation obs = ;
                  double height_obs = Root->Obss[no].height;
                  //cout<< t <<" CHECKING  " <<height_obs <<endl;
                  double temp = sqrt((height_obs-current_pos_eff)*(height_obs-current_pos_eff));
                  if (temp < best_distance){
                      best_obs = no;
                      best_distance = temp;
                  }
              }
              cout<<"best ID "<<best_obs<<"   current end eff: "<<current_pos_eff <<" ; FSC thinks "<<Root->Obss[best_obs].height <<"  ; true is " <<table->X.pos.z<<endl;

          }

          Root = Root->Child(Root->Obss[best_obs]);

          //if(numchild>1) cout<<" Dual of the new table "<< Root->AllDual() <<endl;

       }

     //adapt the PD task references following the plan


        if(t<y.d0){
          pd_y->y_ref = Root->Y();
          pd_x->y_ref = Root->X();

          y_p = Root->Y();
    #ifdef USE_DUAL
          pd_c->desiredForce = Root->Dual();
    #endif
      }

  #ifdef USE_DUAL
      //recalibrate the target based on touch
      double d=0.;
      if(pd_c->desiredApproach.y.N){
        d = pd_c->desiredApproach.y(0); //d = distance measured by constraint task
        if(pd_c->desiredApproach.y_ref(0)==0. && d<1e-2){
          est_target->X.pos.z = endeff->X.pos.z - 0.2; //est_target position update

          //UPDATE height;
          estimate_height = est_target->X.pos.z;
        }
      }
  #endif

      //external sinus on the table height
     // table->X.pos.z = mean_table_height+sin_jitter*::sin(double(t)/15);
  #ifdef USE_DUAL
      //plane_constraint->planeParams(3) = table->X.pos.z + 0.02;
  #endif

      //operational space loop
      for(uint tt=0;tt<10;tt++){
        MC.updateConstraintControllers();
        arr a = MC.operationalSpaceControl();
        q += .1*tau*qdot;
        qdot += .1*tau*a;
      }

      //display and record video
  //    world.watch(false, STRING(t));
      world.gl().update(STRING(t), true, false, true);
      //    flip_image(world.gl().captureImage);
      //    vid->addFrame(world.gl().captureImage);

      //write data
      MT::arrayBrackets="  ";
      data <<t <<' ' <<(t<dual.N?dual(t):0.) <<' '
          <<table->X.pos.z <<' '
         <<endeff->X.pos.z <<' '
        <<endeff->X.pos.z-table->X.pos.z <<' '
        <<est_target->X.pos.z <<' '
       <<true_target->X.pos.z <<' '
      <<endl;
    }
    data.close();

    FILE(STRING("data-"<<num<<"-err.dat")) << ARRAY(true_target->X.pos)- ARRAY(endeff->X.pos);
  }


