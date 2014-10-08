#include "execution.h"
#include "pomdp.h"
#include <Motion/taskMap_proxy.h>
#include <Ors/ors_swift.h>
#include <Core/geo.h>
#include <vector>


using namespace std;


void getTrajectory(arr& x, arr& y, arr& dual, ors::KinematicWorld& world, arr x0, const arr& model, bool stickyness, uint horizon){
    /////////////


   // cout<<"model = =============== "<<model<<endl;
  //SET INITIAL STATE: Model parameter (height, position, size,...), and initial joint's position
    world.setJointState(x0);

    world.getBodyByName("table")->X.pos.z   = model(0);
    world.getBodyByName("target")->X.pos.z  = model(0) + 0.02;

    world.getBodyByName("table")->X.pos.x   = model(1);
    world.getBodyByName("target")->X.pos.x  = model(1);
    ors::Body* target = world.getBodyByName("target");



  /////////////////////////////////////////////////////
  ////////////////////////////////////////////////////

  MotionProblem P(world, true); //true for using swift
  P.loadTransitionParameters(); // can change horizon hereP

  P.T = horizon;
  x = P.getInitialization();

  //-- setup the motion problem 

  //TaskCost *pos = P.addTask("position", new DefaultTaskMap(posTMT, world, "endeff", NoVector, "target", NoVector));
  //P.setInterpolatingCosts(pos, MotionProblem::finalOnly,ARRAY(0.,0.,0.), 2e5);
  TaskCost *pos = P.addTask("position", new DefaultTaskMap(posTMT, world, "endeff", NoVector));
  P.setInterpolatingCosts(pos, MotionProblem::finalOnly,ARRAY(target->X.pos.x,target->X.pos.y,target->X.pos.z), 2e5);

  TaskCost *vel = P.addTask("position_vel", new DefaultTaskMap(posTMT, world, "endeff", NoVector));
  vel->map.order=1;
  P.setInterpolatingCosts(vel, MotionProblem::finalOnly, ARRAY(0.,0.,0.), 1e3);

  /*/
  //see taskmap_default.cpp;
  TaskCost *vec = P.addTask("orientation", new DefaultTaskMap(vecTMT, world, "peg",ARRAY(0.,0.,1.)));
  //P.setInterpolatingCosts(vec, MotionProblem::finalOnly, ARRAY(0.,0.,-1.), 1e3, ARRAY(0.,0.,0.), 1e-3);
  P.setInterpolatingCosts(vec, MotionProblem::early_restConst, ARRAY(0.,0.,-1.), 1e3, NoArr, -1., 0.1);
  /*/

  TaskCost *cons = P.addTask("planeConstraint", new PlaneConstraint(world, "endeff", ARR(0,0,-1, world.getBodyByName("table")->X.pos.z + 0.02)));//0.2 is table width  //0.05 above table surface to avoid slippery
  P.setInterpolatingCosts(cons, MotionProblem::constant, ARRAY(0.), 1e2);


#if 1  //CONSTRAINT
  TaskCost *collision = P.addTask("collisionConstraint", new CollisionConstraint(0.01));
  P.setInterpolatingCosts(collision, MotionProblem::constant, ARRAY(0.), 1e3);
#else
  c = P.addTask("collision", new ProxyTaskMap(allPTMT, {0}, .041));
  P.setInterpolatingCosts(c, MotionProblem::constant, ARRAY(0.), 1e1);
#endif


  if(stickyness){

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



  for(uint k=0;k<10;k++){
   optNewton(x, UnConstrainedP, OPT(verbose=0, stopIters=300, damping=1e-4, stopTolerance=1e-5, maxStep=.5));
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
  uint index = 0;
  dual.resize(x.d0);
  if(&dual) {
      for(int i=1;i<UnConstrainedP.lambda.d0;i=i+2){
          dual(index) = UnConstrainedP.lambda(i);

          index++;
      }
  }

 // cout<< dual <<endl;
}


  /// Online execution: Using POMDP policy (solve the POMDP online, using offline value functions from SOC)
void POMDPExecution(FSC fsc, ors::KinematicWorld& world, int num, double est){


    arr q, qdot;
    world.getJointState(q, qdot);


    ofstream data(STRING("data-"<<num<<".dat"));

    ors::Shape *endeff = world.getShapeByName("endeff");
    ors::Shape *true_target = world.getShapeByName("truetarget");
    ors::Body *est_target = world.getBodyByName("target");
    ors::Body *table = world.getBodyByName("table");


    FeedbackMotionControl MC(world);
    MC.qitselfPD.active=false;

    //position PD task:  decayTime = 0.1, dampingRatio = 0.8
    PDtask *pd_y =  MC.addPDTask("position", .1, .8, new DefaultTaskMap(posTMT, world, "endeff", NoVector));
    pd_y->prec = 10.;
    pd_y->setTarget(ARR(est_target->X.pos.x,est_target->X.pos.y,est_target->X.pos.z));
    //MC.setInterpolatingCosts();


    //joint space PD task
    PDtask *pd_x = MC.addPDTask("pose", .1, .8, new DefaultTaskMap(qItselfTMT, world));
    pd_x->prec = .1;

    //plane constraint task
  #define USE_DUAL
  #ifdef USE_DUAL
    PlaneConstraint *plane_constraint = new PlaneConstraint(world, "endeff", ARR(0,0,-1,table->X.pos.z + 0.02));
    ConstraintForceTask *pd_c =
        MC.addConstraintForceTask("planeConstraint", plane_constraint );


    //collision avoidance
    MC.addPDTask("collisions", .2, .8, collTMT, NULL, NoVector, NULL, NoVector, {.01});
  #endif


    double tau = 0.01;

    NODE*Root = fsc.getRoot();
    arr x = Root->AllX();
    arr y = Root->AllY();
    arr dual = Root->AllDual();

    //world.setJointState(x[0]);
    world.getJointState(q, qdot);

    double estimate_height = Root->Model()(0);

    arr y_p = y[0];


    // remaining 100 steps is for reaching to the target.
    cout<<endl<<"num "<<num<<endl;

    double observation_x = -1111110.0;
    bool   updated = false;
    bool change_force = false;




    world.setJointState(x[0]);

    for(uint t=0;t<x.d0 + 100;t++){
        //MT::wait(.1);


      MC.setState(q, qdot);


      //scan all next node
       if(t<y.d0-1){
          //getting observations munally
          if(endeff->X.pos.x < table->X.pos.x - 0.5){
              observation_x = endeff->X.pos.x;

          }
          /////////////////////////////////////////////////////////


          double current_pos_eff_z = endeff->X.pos.z;
          double current_pos_eff_x = endeff->X.pos.x;

          double best_distance = 10000.;
          int numchild = Root->Childrens.size();

          int best_obs = numchild-1;//always the last child

          if(numchild>1){
              cout<<" checking "<<endl;
              if(observation_x > -10.){ //already found the edge
                  for(int no=0; no < Root->Childrens.size();no++){
                      double pos_obs = Root->Obss[no].pos;
                      double temp = sqrt((pos_obs-observation_x)*(pos_obs-observation_x));
                      if ((temp < best_distance)&&(pos_obs > observation_x)){
                          best_obs = no;
                          best_distance = temp;
                      }
                  }
                  est_target->X.pos.x = observation_x + 0.5;
              }
              //cout<<"reference Y = "<<Root->Y()<<"   current end eff: "<<current_pos_eff_x <<" ; FSC thinks "<<Root->Obss[best_obs].pos <<"  ; true is " <<true_target->X.pos.x<<endl;


          }

          Root = Root->Child(Root->Obss[best_obs]);

          //if(numchild>1) cout<<" Dual of the new table "<< Root->AllDual() <<endl;

       }

       cout<<" model "<<Root->Model() << " Root->index "<<Root->getIndex() <<endl;

        if(t<y.d0){

          //world.setJointState(Root->X());


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


          //UPDATE height;
          if(!updated){
              updated = true;
              change_force = true;
          }


        }//else{
         //   if(change_force){
        //        observation_x = endeff->X.pos.x; //found the  change of edge
        //        est_target->X.pos.x = observation_x;

        //        change_force = false;
        //    }
        //}

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


