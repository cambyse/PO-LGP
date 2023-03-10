

#include "execution.h"
#include "pomdp.h"
#include <vector>



using namespace std;




void getTrajectory(arr& x, arr& y, arr& dual, mlr::KinematicWorld& world, arr x0, const double& height, bool stickyness, uint horizon){
    /////////////

  //set initial state
 world.setJointState(x0);
 world.getBodyByName("table")->X.pos.z = height;
 world.getBodyByName("target")->X.pos.z = height + 0.12;
 mlr::Body* target = world.getBodyByName("target");



  KOMO P(world, false);
  P.loadTransitionParameters(); // can change horizon hereP
  P.T = horizon;
  x = P.getInitialization();

  Task *pos = P.addTask("position", new TaskMap_Default(posTMT, world, "endeffR", NoVector));//, "target", NoVector));
  pos->setCostSpecs(P.T, P.T,conv_vec2arr(target->X.pos), 1e3);



  // ARR(0,0,-1,.7): ax + by + cz + d: where n=(0,0,-1) is its normal vector; d = 0.7
  Task *cons = P.addTask("planeConstraint", new PlaneConstraint(world, "endeffR", ARR(0,0,-1, height+0.02)));
  cons->setCostSpecs(0, P.T, {0.}, 1.);


  if(stickyness){

        Task *sticky = P.addTask("planeStickiness", new ConstraintStickiness(cons->map));
        sticky->setCostSpecs(0, P.T, {0.}, 1.);

        P.makeContactsAttractive = true;
    }else{
       // stickyWeight = .0;
        P.makeContactsAttractive = false;
    }

  MotionProblemFunction MF(P);
  Convert ConstrainedP(MF);

  LagrangianProblem LagrangianP(ConstrainedP);
  LagrangianP.mu = 10.;

  for(uint k=0;k<5;k++){
    optNewton(x, LagrangianP, OPT(verbose=0, stopIters=500, damping=1e-3, stopTolerance=1e-4, maxStep=.5));
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

  if(&dual) dual = LagrangianP.lambda;

}




/// Online execution: Using POMDP policy (solve the POMDP online, using offline value functions from SOC)
void POMDPExecution(FSC fsc, mlr::KinematicWorld& world, int num, double est){



  arr q, qdot;


  ofstream data(STRING("data-"<<num<<".dat"));

  mlr::Shape *endeff = world.getShapeByName("endeffR");
  mlr::Shape *true_target = world.getShapeByName("truetarget");
  mlr::Body *est_target = world.getBodyByName("target");
  mlr::Body *table = world.getBodyByName("table");
  est_target->X.pos.z  = est;


  TaskControlMethods MC(world);
  MC.qitselfPD.active=false;

  //position PD task:  decayTime = 0.1, dampingRatio = 0.8
  CtrlTask *pd_y =  MC.addPDTask("position", .1, .8, new TaskMap_Default(posTMT, world, "endeffR", NoVector));//, "target"));
  pd_y->prec = 10.;
  pd_y->setTarget(ARR(est_target->X.pos.x,est_target->X.pos.y,est_target->X.pos.z));


  //joint space PD task
  CtrlTask *pd_x = MC.addPDTask("pose", .1, .8, new TaskMap_qItself());
  pd_x->prec = .1;

  //plane constraint task
#define USE_DUAL
#ifdef USE_DUAL
  PlaneConstraint *plane_constraint = new PlaneConstraint(world, "endeffR", ARR(0,0,-1,table->X.pos.z));
  ConstraintForceTask *pd_c =
      MC.addConstraintForceTask("planeConstraint", plane_constraint );
  pd_c->setPrecision(1e4);

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


  double observation_x = -1111110.0;
  bool   updated = false;
  bool change_force = false;
  // remaining 100 steps is for reaching to the target.



  world.setJointState(x[0]);
  uint counter = 0;
  cout<<endl<<"num "<<num<<endl;
  for(uint t=0;t<x.d0 + 100;t++){
    MC.setState(q, qdot);

   //double observation = pd_c->infraRed_obs;


    //scan all next node
     if(t<y.d0-2){
         //get observation (Heuristic)// CHANGE HERE FOR PR2
         if(fabs(endeff->X.pos.z - table->X.pos.z)<0.04){
             observation_x = endeff->X.pos.z;
         }





        double current_pos_eff = endeff->X.pos.z;

        double best_distance = 10000.;
        int numchild = Root->Childrens.size();
        int best_obs = numchild-1;//always the last child


        if(numchild>1){
            cout<<" checking "<<endl;
            if(observation_x > -10.){ //already found the edge
                for(int no=0; no < Root->Childrens.size();no++){
                    //Observation obs = ;
                    double height_obs = Root->Obss[no].height;
                    //cout<< t <<" CHECKING  " <<height_obs <<endl;
                    double temp = sqrt((height_obs-observation_x)*(height_obs-observation_x));
                    if (temp < best_distance){
                        best_obs = no;
                        best_distance = temp;
                    }
                }
            }

            //pd_c->setPrecision(0.0);
        }

        Root = Root->Child(Root->Obss[best_obs]);

     }

   //adapt the PD task references following the plan


      if(t<y.d0){

         // world.setJointState( Root->X());

        pd_y->y_ref = Root->Y();
        pd_x->y_ref = Root->X();
  #ifdef USE_DUAL
        /*/
        if((!updated)&&(t>5))
            pd_c->desiredForce = 0.5;// Root->Dual();
        else
            pd_c->desiredForce = 0.0;//Root->Dual();
        /*/

        pd_c->desiredForce = Root->Dual();

        if(observation_x>0)
            cout<<Root->AllDual() <<endl;
  #endif
    }

#ifdef USE_DUAL
    //recalibrate the target based on touch
    double d=0.;
    if(pd_c->desiredApproach.y.N){
      d = pd_c->desiredApproach.y(0); //d = distance measured by constraint task
      if(pd_c->desiredApproach.y_ref(0)==0. && d<1e-2){

          if(!updated){
            est_target->X.pos.z = endeff->X.pos.z + 0.1; //est_target position update

            //UPDATE height;
            estimate_height = est_target->X.pos.z;
            observation_x = endeff->X.pos.z;

            pd_y->setTarget(ARR(true_target->X.pos.x,true_target->X.pos.y,true_target->X.pos.z));

            cout<<"updated  "<<endl;


             updated = true;
          }

      }//else
       //    updated = true;
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

