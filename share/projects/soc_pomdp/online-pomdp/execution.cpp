

#include "execution.h"
#include "pomdp.h"
#include <vector>


using namespace std;




void getTrajectory(arr& x, arr& y, arr& dual, ors::KinematicWorld& world, arr x0, const double& height, bool stickyness, uint horizon){
    /////////////

  //set initial state
 world.setJointState(x0);
 world.getBodyByName("table")->X.pos.z = height;


  /////////////////////////////////////////////////////
  ////////////////////////////////////////////////////


  MotionProblem P(world, false);
  P.loadTransitionParameters(); // can change horizon hereP
  P.world.setJointState(x0);


  P.T = horizon;

  x = P.getInitialization();

  //-- setup the motion problem


  //TaskCost *pos = P.addTask("position", new DefaultTaskMap(posTMT, world, "endeff", NoVector, "target", NoVector));
  //P.setInterpolatingCosts(pos, MotionProblem::finalOnly,ARRAY(0.,0.,0.), 1e3);
  //MODIFY the target location relatively to the height +0.12 = 0.1 + 0.02 (0.02 is table width).
  world.getBodyByName("target")->X.pos.z = height + 0.12;

  TaskCost *pos = P.addTask("position", new DefaultTaskMap(posTMT, world, "endeff", NoVector, "target", NoVector));
  P.setInterpolatingCosts(pos, MotionProblem::finalOnly,ARRAY(0.,0.,0.), 1e3);



  // ARR(0,0,-1,.7): ax + by + cz + d: where n=(0,0,-1) is its normal vector; d = 0.7
  TaskCost *cons = P.addTask("planeConstraint", new PlaneConstraint(world, "endeff", ARR(0,0,-1, height+0.02)));
  P.setInterpolatingCosts(cons, MotionProblem::constant, ARRAY(0.), 1e3);

    if(stickyness){
        stickyWeight = .5;
        P.makeContactsAttractive = true;
    }else{
        stickyWeight = .0;
        P.makeContactsAttractive = false;
    }

  MotionProblemFunction MF(P);
  Convert ConstrainedP(MF);

  UnconstrainedProblem UnConstrainedP(ConstrainedP);
  UnConstrainedP.mu = 10.;

  for(uint k=0;k<5;k++){
    optNewton(x, UnConstrainedP, OPT(verbose=0, stopIters=100, damping=1e-3, stopTolerance=1e-4, maxStep=.5));
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

  if(&dual) dual = UnConstrainedP.lambda;

  //heuristic here (I don't want too stickyness)
  for(int index=0;index<dual.d0;index++)
      if(dual(index) > 0) dual(index) = 0.5;
  //cout<< " x " <<x[0]<<endl;
//  cout<< dual<<endl;
}





/// Online execution: Using POMDP policy (solve the POMDP online, using offline value functions from SOC)
void POMDPExecution(FSC fsc, ors::KinematicWorld& world, int num, double est){



  arr q, qdot;


  ofstream data(STRING("data-"<<num<<".dat"));

  ors::Shape *endeff = world.getShapeByName("endeff");
  ors::Shape *true_target = world.getShapeByName("truetarget");
  ors::Body *est_target = world.getBodyByName("target");
  ors::Body *table = world.getBodyByName("table");
  est_target->X.pos.z  = est;



  double mean_table_height = table->X.pos.z;

  double sin_jitter = MT::getParameter<double>("sin_jitter", 0.);

  FeedbackMotionControl MC(world);
  MC.qitselfPD.active=false;

  //position PD task:  decayTime = 0.1, dampingRatio = 0.8
  PDtask *pd_y =  MC.addPDTask("position", .1, .8, new DefaultTaskMap(posTMT, world, "endeff", NoVector, "target"));
  pd_y->prec = 10.;

  //joint space PD task
  PDtask *pd_x = MC.addPDTask("pose", .1, .8, new DefaultTaskMap(qItselfTMT, world));
  pd_x->prec = .1;

  //plane constraint task
#define USE_DUAL
#ifdef USE_DUAL
  PlaneConstraint *plane_constraint = new PlaneConstraint(world, "endeff", ARR(0,0,-1,table->X.pos.z+0.02));
  ConstraintForceTask *pd_c =
      MC.addConstraintForceTask("planeConstraint", plane_constraint );
 // pd_c->desiredApproach.prec = .1;
//      MC.addConstraintForceTask("touchTable",
//                                new PairCollisionConstraint(world, "endeff2", "table"));
#endif


  double tau = 0.01; 

  NODE*Root = fsc.getRoot();
  arr x = Root->AllX();
  arr y = Root->AllY();
  arr dual = Root->AllDual();

  //world.setJointState(x[0]);
  world.getJointState(q, qdot);

  double estimate_height = Root->Height();


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


       // if(pd_c->desiredForce>0)cout<<sqrt((y_p(0)-pd_y->y_ref(0))*(y_p(0)-pd_y->y_ref(0))+(y_p(1)-pd_y->y_ref(1))*(y_p(1)-pd_y->y_ref(1))+(y_p(2)-pd_y->y_ref(2))*(y_p(2)-pd_y->y_ref(2)))<<" " ;

        y_p = Root->Y();
  #ifdef USE_DUAL
        pd_c->desiredForce = Root->Dual();

        cout<< Root->Dual() <<"  ";
  #endif
    }

#ifdef USE_DUAL
    //recalibrate the target based on touch
    double d=0.;
    if(pd_c->desiredApproach.y.N){
      d = pd_c->desiredApproach.y(0); //d = distance measured by constraint task
      if(pd_c->desiredApproach.y_ref(0)==0. && d<1e-2){
        est_target->X.pos.z = endeff->X.pos.z + 0.1; //est_target position update

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

