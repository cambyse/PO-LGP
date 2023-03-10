#include <Control/gamepad2tasks.h>
#include <Control/taskControl.h>
#include <Hardware/joystick/joystick.h>
//#include <System/engine.h>
#include <Gui/opengl.h>

#include <RosCom/roscom.h>
#include <RosCom/actions.h>
#include <RosCom/actionMachine.h>

#include <KOMO/komo.h>
#include <Kin/taskMaps.h>

#include <Optim/optimization.h>
#include <Core/util.h>


#include "pomdp.h"
#include "execution.h"



//#include <Perception/videoEncoder.h>


//extern double stickyWeight;

//VideoEncoder_libav_simple *vid;


///////////////////////////////////////////////////////////////////////
//
///////////////////////////////////////////////////////////////////////
struct MySystem{
  ACCESS(CtrlMsg, ctrl_ref);
  ACCESS(CtrlMsg, ctrl_obs);
  ACCESS(arr, joystickState);
  ACCESS(arr, wrenchL)
  ACCESS(arr, wrenchR)
  MySystem(){
    addModule<JoystickInterface>(NULL, /*Module::loopWithBeat,*/ .01);
    if(mlr::getParameter<bool>("useRos", false)){
      new RosCom_Spinner();
      new SubscriberConvNoHeader<marc_controller_pkg::JointState, CtrlMsg, &conv_JointState2CtrlMsg>("/marc_rt_controller/jointState", ctrl_obs);
      new PublisherConv<marc_controller_pkg::JointState, CtrlMsg, &conv_CtrlMsg2JointState>("/marc_rt_controller/jointReference", ctrl_ref);
      addModule<RosCom_ForceSensorSync>(NULL, /*Module::loopWithBeat,*/ 1.);
    }
    //connect();
  }
};

/// Online execution: Using POMDP policy (solve the POMDP online, using offline value functions from SOC)
void PR2_ActionMachine(FSC fsc, mlr::KinematicWorld& world, int num){


     ofstream data(STRING("data-"<<num<<".dat"));

 // mlr::KinematicWorld& world = activity.machine->s->world;
  MySystem S;
  threadOpenModules(true);
  makeConvexHulls(world.shapes);
  world >>FILE("z.ors");
  arr q, qdot;
  world.getJointState(q, qdot);
  mlr::Joint *trans=world.getJointByName("worldTranslationRotation");
  mlr::Shape *ftL_shape=world.getShapeByName("endeffR");

  mlr::KinematicWorld worldCopy = world;

  //world.gl().add(mlr::glDrawGraph, &worldCopy);

  TaskControlMethods MP(world, true); // true means using swift
  //MP.qitselfPD.y_ref = q;
  MP.H_rate_diag = world.getHmetric();

  bool useRos = mlr::getParameter<bool>("useRos", false);
  if(useRos){
    //-- wait for first q observation!
    cout <<"** Waiting for ROS message on initial configuration.." <<endl;
    for(;;){
      S.ctrl_obs.data->waitForNextRevision();
      if(S.ctrl_obs.get()->q.N==MP.world.q.N
         && S.ctrl_obs.get()->qdot.N==MP.world.q.N)
        break;
    }

    //-- set current state
    cout <<"** GO!" <<endl;
    q = S.ctrl_obs.get()->q;
    qdot = S.ctrl_obs.get()->qdot;
    //arr fL_base = S.fL_obs.get();
    MP.setState(q, qdot);
  }

  arr zero_qdot(qdot.N);
  zero_qdot.setZero();
  CtrlMsg refs;



  /////////////////////////////////////////////////////////////////////////////////
  // FROM old code
  ////////////////////////////////////////////////////////////////////////////////
  MP.qitselfPD.active=true;


  mlr::Shape *endeff = world.getShapeByName("endeffR");
  mlr::Shape *true_target = world.getShapeByName("truetarget");
  mlr::Body *est_target = world.getBodyByName("target");
  mlr::Body *table = world.getBodyByName("table");



  CtrlTask *pd_y =  MP.addPDTask("position", .1, .8, new TaskMap_Default(posTMT, world, "endeffR", NoVector));//, "target"));
  pd_y->setTarget(ARR(est_target->X.pos.x,est_target->X.pos.y,est_target->X.pos.z));
  pd_y->prec = 10.;

  //joint space PD task
  CtrlTask *pd_x = MP.addPDTask("pose", .1, .8, new TaskMap_qItself());
  pd_x->prec = .1;


/*/

  CtrlTask* limits = MP.addPDTask("limits", .1, .8, new TaskMap_qLimits());
  // limits->setGains(10.,0.);
  limits->v_ref.setZero();
  limits->v_ref.setZero();
  limits->prec=100.;
  //tasks.append(limits);

  CtrlTask* coll = MP.addPDTask("collisions", .2, .8, collTMT, NULL, NoVector, NULL, NoVector, {.1});
  coll->y_ref.setZero();
  coll->v_ref.setZero();/*/

  cout<<"table->X.pos.z "<<table->X.pos.z<<"  "<< endeff->X.pos.z<< endl;

  //PlaneConstraint *plane_constraint = new PlaneConstraint(world, "endeffR", ARR(0,0,-1,table->X.pos.z+0.02));
  PlaneConstraint *plane_constraint = new PlaneConstraint(world, "endeffR", ARR(0,0,-1,0.2));
  ConstraintForceTask *pd_c = MP.addConstraintForceTask("planeConstraint", plane_constraint );


  ///////////////////////////////////////////
  NODE*Root = fsc.getRoot();
  arr x = Root->AllX();
  arr y = Root->AllY();
  arr dual = Root->AllDual();



  pd_c->setPrecision(0.0);


 double observation_x = -1111110.0;

  bool updated =false;
  for(uint t=0;t<x.d0 + 100;t++){
      mlr::wait(.1);

      MP.setState(q, qdot);

      //double d=0.;
      arr f_r = S.wrenchR.get();// ctrl_obs.get()->fR;

      cout<<"[f_r] = "<< f_r<<endl;

      if((!updated)&&(f_r(1)<6.0)&&(t>20)){
          est_target->X.pos.z = endeff->X.pos.z + 0.1;
          plane_constraint->planeParams(3) = endeff->X.pos.z;

          cout<<"updated "<<endeff->X.pos.z<<endl;
          updated = true;

          observation_x = endeff->X.pos.z;
      }

      //scan all next node: Running FSC
       if(t<y.d0-1){
           //get observation (Heuristic)// CHANGE HERE FOR PR2
           //if(fabs(endeff->X.pos.z - table->X.pos.z)<0.04){
           //    observation_x = endeff->X.pos.z;
          // }



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



    ////////////////////////////////////////////////////////////////////////////////
      if(t<y.d0){
        pd_y->y_ref = Root->Y();
        pd_x->y_ref = Root->X();
        pd_c->desiredForce = Root->Dual();
    }
    ////////////////////////////////////////////////////////////////////////////////


    // joint state
    if(useRos){
      worldCopy.setJointState(S.ctrl_obs.get()->q, S.ctrl_obs.get()->qdot);
    }

    //cout<< endeff->X.pos.z<< endl;



    if(pd_c->desiredApproach.y.N){

      //cout<<

      double d = pd_c->desiredApproach.y(0); //d = distance measured by constraint task
      if(pd_c->desiredApproach.y_ref(0)==0. && d<1e-2){
          if(!updated){
            est_target->X.pos.z = endeff->X.pos.z + 0.1; //est_target position update

            plane_constraint->planeParams(3) = endeff->X.pos.z;
            pd_y->setTarget(ARR(true_target->X.pos.x,true_target->X.pos.y,true_target->X.pos.z));

            //UPDATE height;

            observation_x = endeff->X.pos.z;



            cout<<"updated  "<<endl;


             updated = true;
          }
      }
    }
////////////////////////////////////////////////////////////////////////////////

    //compute control

    for(uint tt=0;tt<10;tt++){
      arr a = MP.operationalSpaceControl();
      q += .001*qdot;
      qdot += .001*a;
    }


 // MP.reportCurrentState();

    //MP.world.reportProxies();
    //if(!(t%4))
      MP.world.gl().update(STRING("local operational space controller state t="<<(double)t/100.), false, false, false);

    refs.q = q;
    refs.qdot = zeros(q.N);
    refs.u_bias = zeros(q.N);

    S.ctrl_ref.set() = refs;


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

  threadCloseModules();
  cout <<"bye bye" <<endl;


}

// ============================================================================
int main(int argc, char** argv)
{

  mlr::initCmdLine(argc, argv);

  ActionSystem activity;
  activity.machine->add(new CoreTasks());

  arr q,qdot,x0;
  activity.machine->s->world.getJointState(x0,qdot);
  double table_height = activity.machine->s->world.getBodyByName("table")->X.pos.z;
  cout<< "planning's table_height " <<table_height<<endl;

  uint T = 200; //time horizon
  uint numSamples = 8;

  mlr::timerStart(true);
  //rnd.seed(111001);


  mlr::Shape *endeff = activity.machine->s->world.getShapeByName("endeffR");

  arr y0;

  y0.resize(3);

  y0(0) = endeff->X.pos.x;
  y0(1) = endeff->X.pos.y;
  y0(2) = endeff->X.pos.z;
  double dual = 0.;

  FSC fsc;
  NODE*Root = fsc.getRoot();
  Root->X() = x0;
  Root->Y() = y0;
  //Root->Dual() = dual;


  arr heights;
  heights.resize(numSamples);
  arr xx, yy, ddual;


  xx.resize(T+1,x0.d0);
  yy.resize(T+1,y0.d0);
  ddual.resize(T+1,1);

  for(uint i=0;i<numSamples;i++){
      heights(i) = .48 + 0.05*rnd.uni();  //TABLE INFORMATION
  }
  cout<<heights  <<endl;
  for(uint i=0;i<T+1;i++){
      xx[i]() = x0;
      yy[i]() = y0;
      ddual[i]() = dual;
  }

  Root->AllX() = xx;
  Root->AllY() = yy;
  Root->AllDual() = ddual;
  Root->Heights() = heights;
  Root->Height() = heights(0);



  //2. trajectory optimization: return primal,dual trajectories, and value functions (at each time slice)
/*/
  arr x, y, dual1;
  getTrajectory(x, y, dual1, activity.machine->s->world, x0, 0.3, true, T);

/*/

  //building a FSC controller;
  FSC::Horizon = T;
  //OptimizeFSC_Test(world, Root, 0); //start optimize the FSC from level 0 (root node)
  OptimizeFSC(activity.machine->s->world, Root, 0);


  cout<<"Offline Computation Time = "<< mlr::realTime() <<" (s)"<<endl;


  //write the FSC controller to .dot file
  write_to_graphviz(fsc);


  //TESTING: Online POMDP planning
  //POMDP

  orsDrawJoints=orsDrawProxies=orsDrawMarkers=false;

  for(uint i=0;i<5;i++){
    activity.machine->s->world.setJointState(x0);
    //activity.machine->s->world.gl().update(STRING(i), true, false, true);

    activity.machine->s->world.getBodyByName("table")->X.pos.z = 0.5;
    //PR2_POMDPExecution(activity, x, y, dual, i);
    PR2_ActionMachine(fsc,activity.machine->s->world, i);
  }

  return 0;
}
