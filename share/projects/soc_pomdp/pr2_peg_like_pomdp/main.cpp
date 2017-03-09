#include <Control/gamepad2tasks.h>
#include <Control/taskControl.h>
#include <Hardware/joystick/joystick.h>
//#include <System/engine.h>
#include <Gui/opengl.h>

#include <RosCom/roscom.h>
#include <RosCom/actions.h>
#include <RosCom/actionMachine.h>

#include <Motion/motion.h>
#include <Motion/taskMaps.h>
#include <Motion/taskMaps.h>

#include <Optim/optimization.h>
#include <Core/util.h>


#include "pomdp.h"
#include "execution.h"


#define WIDTH 0.3



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
  bool updated_edge = false;
  bool branching = false;
  for(uint t=0;t<x.d0 + 100;t++){
      mlr::wait(.1);

      //cout<<"endeff->X.pos"<<endeff->X.pos <<endl;

      MP.setState(q, qdot);

      //recalibrate the target based on touch
      double d=0.;
      arr f_r = S.wrenchR.get();// ctrl_obs.get()->fR;

      cout<< f_r<<endl;

      if((!updated)&&(f_r(1)<6.0)&&(t>30)){
          est_target->X.pos.z = endeff->X.pos.z;
          //est_target->X.pos.x = table->X.pos.x;
          plane_constraint->planeParams(3) = endeff->X.pos.z - 0.01;
          //pd_y->setTarget(ARR(true_target->X.pos.x,true_target->X.pos.y,true_target->X.pos.z));

          cout<<"updated"<<endl;
          updated = true;
      }


      //scan all next node: Running FSC
       if(t<y.d0-2){
           //getting observations munally
          // if((endeff->X.pos.y < table->X.pos.y - 0.27) &&(fabs(endeff->X.pos.z < table->X.pos.z)<0.1)){
         //      observation_x = endeff->X.pos.y;
         //      est_target->X.pos.x =  endeff->X.pos.x + 0.27;
         //      pd_y->setTarget(ARR(est_target->X.pos.x,est_target->X.pos.y,est_target->X.pos.z));

         //  }
           if((!updated_edge) && (updated) &&(f_r(5)>0.0)&&(t>30)){
                      observation_x = endeff->X.pos.y;
                      est_target->X.pos.y =  endeff->X.pos.y + WIDTH;
                      pd_y->setTarget(ARR(est_target->X.pos.x,est_target->X.pos.y,est_target->X.pos.z));

                      updated_edge = true;
                      cout<<"edge"<<observation_x<<endl;
           }
           /////////////////////////////////////////////////////////


           double current_pos_eff_z = endeff->X.pos.z;
           double current_pos_eff_x = endeff->X.pos.y;

           double best_distance = 10000.;
           int numchild = Root->Childrens.size();

           int best_obs = numchild-1;//always the last child

           if(numchild>1){
               cout<<" checking "<<endl;
               double pos_obs;
               if(observation_x > -10.){ //already found the edge
                   for(int no=0; no < Root->Childrens.size();no++){
                       pos_obs = Root->Obss[no].pos;
                       double temp = sqrt((pos_obs-observation_x)*(pos_obs-observation_x));
                       if ((temp < best_distance)&&(pos_obs > observation_x)){
                           best_obs = no;
                           best_distance = temp;
                       }
                   }
                   est_target->X.pos.y = observation_x + WIDTH;

                   branching = true;
                   cout<< "branching at stored center = "<<pos_obs <<endl;
               }

               //cout<<"reference Y = "<<Root->Y()<<"   current end eff: "<<current_pos_eff_x <<" ; FSC thinks "<<Root->Obss[best_obs].pos <<"  ; true is " <<true_target->X.pos.x<<endl;


           }

           Root = Root->Child(Root->Obss[best_obs]);
       }



    ////////////////////////////////////////////////////////////////////////////////
      if(t<y.d0){
        pd_y->y_ref = Root->Y();
        pd_x->y_ref = Root->X();
        if(Root->Dual() > 0)
            pd_c->desiredForce = Root->Dual();
        else if(!branching)
            pd_c->desiredForce = 1000;
        else
            pd_c->desiredForce = Root->Dual();

        cout<<"pd_c->desiredForce "<< pd_c->desiredForce <<endl;
    }
    ////////////////////////////////////////////////////////////////////////////////


    // joint state
    if(useRos){
      worldCopy.setJointState(S.ctrl_obs.get()->q, S.ctrl_obs.get()->qdot);
    }

    //cout<< endeff->X.pos.z<< endl;

////////////////////////////////////////////////////////////////////////////////



    if(pd_c->desiredApproach.y.N){

      //cout<<

      d = pd_c->desiredApproach.y(0); //d = distance measured by constraint task
      if(pd_c->desiredApproach.y_ref(0)==0. && d<1e-2){
          if(!updated){
            est_target->X.pos.z = endeff->X.pos.z + 0.1; //est_target position update

            plane_constraint->planeParams(3) = endeff->X.pos.z;
           // pd_y->setTarget(ARR(true_target->X.pos.x,true_target->X.pos.y,true_target->X.pos.z));

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

  uint T = 240; //time horizon
  uint numSamples_height = 1; //sampling height
  uint numSamples_pos    = 5; //each height has 10 sampled pos. In Total, we have 100 samples (hierarchical).
  uint total = numSamples_height*numSamples_pos + numSamples_height; //each height we generate one pseudo-sample (that will help to find the best observation)


  mlr::timerStart(true);
  arr y0;
  double dual;
  //arr x0 = world.getJointState();
  mlr::Shape *endeff = activity.machine->s->world.getShapeByName("endeffR");

  y0.resize(3);

  y0(0) = endeff->X.pos.x;
  y0(1) = endeff->X.pos.y;
  y0(2) = endeff->X.pos.z;
  dual = 0.;

  FSC fsc;
  NODE*Root = fsc.getRoot();
  Root->X() = x0;
  Root->Y() = y0;
  Root->Dual() = dual;


  arr samples;
  arr xx, yy, ddual;
  samples.resize(total,2); //2 means: height and pos
  xx.resize(total,x0.d0);
  yy.resize(total,y0.d0);
  ddual.resize(total,1);

  //adding a pseudo sample to each height (the one with the largest abs(x)

  uint index = 0;
  uint best=index;
  for(uint i=0;i<numSamples_height;i++){
      double sampled_height = 0.5; //fix here
      //cout<< sampled_height <<endl;
      samples[index](0) = sampled_height;

      for(uint j=0;j<numSamples_pos;j++){
          samples[index](0) = sampled_height;

          samples[index](1) = -0.35 - .1*rnd.gauss();
          while ((samples[index](1)<-0.43) || (samples[index](1) >-0.3))
              samples[index](1) = -0.35 - .1*rnd.gauss();
          index ++;
      }
      samples[index](0) = sampled_height;
      samples[index](1) = - 0.8; //the farthest on the right
      best = index;
      index++;
  }



  cout<<"samples size = "<< samples.d0 <<endl;
  cout<<"samples = "<<samples<<endl;


  Root->AllX() = xx;
  Root->AllY() = yy;
  Root->AllDual() = ddual;
  Root->Samples() = samples;
  Root->Model() = samples[best];




  //building a FSC controller;
  FSC::Horizon = T;
  OptimizeFSC(activity.machine->s->world, Root, 0);
  //OptimizeFSC_Test(world, Root, 0);
  //write the FSC controller to .dot file
  write_to_graphviz(fsc);


  cout<<"Offline Computation Time = "<< mlr::realTime() <<" (s)"<<endl;


  //TESTING: Online POMDP planning
  //POMDP

  orsDrawJoints=orsDrawProxies=orsDrawMarkers=false;

  for(uint i=0;i<5;i++){
    activity.machine->s->world.setJointState(x0);
    //activity.machine->s->world.gl().update(STRING(i), true, false, true);

    activity.machine->s->world.getBodyByName("table")->X.pos.y = -0.2;
    //activity.machine->s->world.getBodyByName("table")->X.pos.x = 0.9;
    activity.machine->s->world.getBodyByName("table")->X.pos.z = 0.25;//samples[best](0);//table_height;// + 0.1*rnd.gauss();

    activity.machine->s->world.getBodyByName("target")->X.pos.y = -0.55;// samples[best](1);
    //activity.machine->s->world.getBodyByName("target")->X.pos.x = 0.5;
    activity.machine->s->world.getBodyByName("target")->X.pos.z = samples[best](0);//table_height;// + 0.1*rnd.gauss();
    //PR2_POMDPExecution(activity, x, y, dual, i);
    PR2_ActionMachine(fsc,activity.machine->s->world, i);
  }

  return 0;
}
