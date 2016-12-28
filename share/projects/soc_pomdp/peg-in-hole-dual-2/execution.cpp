#include "execution.h"
#include "pomdp.h"
#include <Motion/taskMaps.h>
#include <Ors/ors_swift.h>
#include <Geo/geo.h>
#include <vector>


using namespace std;


void getTrajectory(arr& x, arr& y, arr& dual, mlr::KinematicWorld& world, arr x0, const arr& model, bool stickyness, uint horizon){
    /////////////


    cout<<"model = =============== "<<model<<endl;
  //SET INITIAL STATE: Model parameter (height, position, size,...), and initial joint's position
    world.setJointState(x0);
    world.getBodyByName("hole")->X.pos.z   = model(0);
    world.getBodyByName("target")->X.pos.z = model(0) + 0.5;

    world.getBodyByName("hole")->X.pos.x   = model(1);
    world.getBodyByName("target")->X.pos.x = model(1);



  /////////////////////////////////////////////////////
  ////////////////////////////////////////////////////

  MotionProblem P(world, true); //true for using swift
  P.loadTransitionParameters(); // can change horizon hereP

  P.T = horizon;
  x = P.getInitialization();

  //-- setup the motion problem 

  Task *pos = P.addTask("position", new TaskMap_Default(posTMT, world, "peg", NoVector, "target", NoVector));
  pos->setCostSpecs(P.T, P.T,{0.,0.,0.}, 2e5);

  Task *vel = P.addTask("position_vel", new TaskMap_Default(posTMT, world, "peg", NoVector));
  vel->map.order=1;
  vel->setCostSpecs(P.T, P.T, {0.,0.,0.}, 1e3);

  //see taskmap_default.cpp;
  Task *vec = P.addTask("orientation", new TaskMap_Default(vecTMT, world, "peg",{0.,0.,1.}));
  //vec->setCostSpecs(P.T, P.T, {0.,0.,-1.}, 1e3, {0.,0.,0.}, 1e-3);
  P.setInterpolatingCosts(vec, MotionProblem::early_restConst, {0.,0.,-1.}, 1e3, NoArr, -1., 0.1);


  Task *cons = P.addTask("planeConstraint", new PlaneConstraint(world, "peg", ARR(0,0,-1, world.getBodyByName("hole")->X.pos.z + 0.5)));//0.2 is table width  //0.05 above table surface to avoid slippery
  cons->setCostSpecs(0, P.T, {0.}, 1e2);


#if 1  //CONSTRAINT
  Task *collision = P.addTask("collisionConstraint", new CollisionConstraint(0.05));
  collision->setCostSpecs(0, P.T, {0.}, 1.);
#else
  c = P.addTask("collision", new TaskMap_Proxy(allPTMT, {0}, .041));
  c->setCostSpecs(0, P.T, {0.}, 1e1);
#endif


  if(stickyness){

    Task *sticky = P.addTask("planeStickiness", new ConstraintStickiness(cons->map));
    sticky->setCostSpecs(0, P.T, {0.}, 1.);

    P.makeContactsAttractive = true;
  }else{
    //stickyWeight = 0.;
    P.makeContactsAttractive = false;
  }


  MotionProblemFunction MF(P);
  Convert ConstrainedP(MF);

  LagrangianProblem LagrangianP(ConstrainedP);
  LagrangianP.mu = 10.;



  for(uint k=0;k<20;k++){
   optNewton(x, LagrangianP, OPT(verbose=0, stopIters=300, damping=1e-4, stopTolerance=1e-5, maxStep=.5));
    P.costReport(false);
//    displayTrajectory(x, 1, G, gl,"planned trajectory");
    LagrangianP.aulaUpdate(.9,x);
    P.dualMatrix = LagrangianP.lambda;
    LagrangianP.mu *= 2.;
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
      for(int i=1;i<LagrangianP.lambda.d0;i=i+2){
          dual(index) = LagrangianP.lambda(i);

          index++;
      }
  }

  //cout<< dual <<endl;
}


  /// Online execution: Using POMDP policy (solve the POMDP online, using offline value functions from SOC)
void POMDPExecution(FSC fsc, mlr::KinematicWorld& world, int num, double est){



    arr q, qdot;
    world.getJointState(q, qdot);


    ofstream data(STRING("data-"<<num<<".dat"));

    mlr::Shape *endeff = world.getShapeByName("peg");
    mlr::Shape *true_target = world.getShapeByName("target");
    mlr::Body *est_target = world.getBodyByName("target");
    mlr::Body *table = world.getBodyByName("hole");


    //est_target->X.pos.z  = est;


    TaskController MC(world);
    MC.qitselfPD.active=false;

    //position PD task:  decayTime = 0.1, dampingRatio = 0.8
    CtrlTask *pd_y =  MC.addPDTask("position", .1, .8, new TaskMap_Default(posTMT, world, "peg", NoVector, "target"));
    pd_y->prec = 10.;

    //joint space PD task
    CtrlTask *pd_x = MC.addPDTask("pose", .1, .8, new TaskMap_qItself());
    pd_x->prec = .1;

    //plane constraint task
  #define USE_DUAL
  #ifdef USE_DUAL
    PlaneConstraint *plane_constraint = new PlaneConstraint(world, "peg", ARR(0,0,-1,table->X.pos.z + 0.5));
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
      //MC.setState(q, qdot);

     //double observation = pd_c->infraRed_obs;


      //scan all next node
       if(t<y.d0-2){
          double current_pos_eff_z = endeff->X.pos.z;
          double current_pos_eff_x = endeff->X.pos.x;
          int best_obs = 0;
          double best_distance = 10000.;
          int numchild = Root->Childrens.size();
          /*/
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
          }/*/

          if(numchild>1){
              for(int no=0; no < Root->Childrens.size();no++){
                  //Observation obs = ;
                  double height_obs = Root->Obss[no].pos;
                  //cout<< t <<" CHECKING  " <<height_obs <<endl;
                  double temp = sqrt((height_obs-current_pos_eff_x)*(height_obs-current_pos_eff_x));
                  if (temp < best_distance){
                      best_obs = no;
                      best_distance = temp;
                  }
              }
              cout<<"reference Y = "<<Root->Y()<<"   current end eff: "<<current_pos_eff_x <<" ; FSC thinks "<<Root->Obss[best_obs].pos <<"  ; true is " <<true_target->X.pos.x<<endl;

              best_obs = 1;
          }

          Root = Root->Child(Root->Obss[best_obs]);

          //if(numchild>1) cout<<" Dual of the new table "<< Root->AllDual() <<endl;

       }



     //adapt the PD task references following the plan

       cout<<"reference Y = "<<Root->Y() <<endl;

       cout<<"peg " <<endeff->X.pos<<endl;


        if(t<y.d0){

          world.setJointState(Root->X());


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
          est_target->X.pos.z = endeff->X.pos.z; //est_target position update

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
      /*/
      for(uint tt=0;tt<10;tt++){
        MC.updateConstraintControllers();
        arr a = MC.operationalSpaceControl();
        q += .1*tau*qdot;
        qdot += .1*tau*a;
      }
      /*/

      //display and record video
  //    world.watch(false, STRING(t));
      world.gl().update(STRING(t), true, false, true);
      //    flip_image(world.gl().captureImage);
      //    vid->addFrame(world.gl().captureImage);

      //write data
      mlr::arrayBrackets="  ";
      data <<t <<' ' <<(t<dual.N?dual(t):0.) <<' '
          <<table->X.pos.z <<' '
         <<endeff->X.pos.z <<' '
        <<endeff->X.pos.z-table->X.pos.z <<' '
        <<est_target->X.pos.z <<' '
       <<true_target->X.pos.z <<' '
      <<endl;
    }
    data.close();

    FILE(STRING("data-"<<num<<"-err.dat")) << conv_vec2arr(true_target->X.pos)- conv_vec2arr(endeff->X.pos);
  }


