#include <Motion/gamepad2tasks.h>
#include <Control/taskController.h>
//#include <Hardware/joystick/joystick.h>
//#include <System/engine.h>
#include <Gui/opengl.h>
#include <Motion/pr2_heuristics.h>
#include <RosCom/roscom.h>
//#include <RosCom/actions.h>
//#include <RosCom/actionMachine.h>

#include <Motion/motion.h>
#include <Motion/taskMaps.h>
#include <Motion/taskMaps.h>

#include <Optim/optimization.h>
#include <Core/util.h>


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
  //ACCESS(arr, joystickState);
  ACCESS(arr, wrenchL)
  ACCESS(arr, wrenchR)
  MySystem(){
    //addModule<JoystickInterface>(NULL, Module_Thread::loopWithBeat, .01);
    if(mlr::getParameter<bool>("useRos", false)){
      new RosCom_Spinner();
      new SubscriberConvNoHeader<marc_controller_pkg::JointState, CtrlMsg, &conv_JointState2CtrlMsg>("/marc_rt_controller/jointState", ctrl_obs);
      new PublisherConv<marc_controller_pkg::JointState, CtrlMsg, &conv_CtrlMsg2JointState>("/marc_rt_controller/jointReference", ctrl_ref);
      addModule<RosCom_ForceSensorSync>(NULL, Module_Thread::loopWithBeat, 1.);
    }
    //connect();
  }
};

void switchToTransparent(void*){
    orsDrawAlpha = .3;
}

void switchToNormal(void*){
    orsDrawAlpha = 1.0;
}




/// Online execution: Using POMDP policy (solve the POMDP online, using offline value functions from SOC)
void OnlineSubmodularity(arr &q, arr &qdot,MySystem &S,  mlr::Array<mlr::String> active_joints,const double tableW, const double tableL, ors::KinematicWorld& world,ors::KinematicWorld& world_plan, int num, const arr target, int type,const arr &center){

    ofstream data(STRING("data-"<<num<<".dat"));
    TaskController MP(world, true); // true means using swift
    MP.H_rate_diag = pr2_reasonable_W(world);
  ////////////////////////////////////////////////////////////////////////////////////////
  // PLANNING
  ////////////////////////////////////////////////////////////////////////////////////////
  //optimize the trajectory w.r.t a selected particle (model)
  arr x, y, dual, temp; 

  getTrajectory(x, y, temp, world_plan, target, 200); //one example of best model
  dual.resize(x.d0);

  //cout<<"temp = "<<temp.d0 << " " <<temp<<endl;



  int tt=0;
  for(int i=0;i<temp.d0;i++){
      if((i+1)%9==0){
          dual(tt) = temp(i);
          tt++;
      }
      if(tt>=x.d0) break;

  }
  cout<<"dual = "<<dual<<endl;

  /// convert trajectory to full joint trajectory
  arr xP,xdP;

  for (uint i = 0;i<x.d0;i++){
    arr q_tmp = q;
    arr qd_tmp = qdot;
    transPlanPR2(active_joints,world_plan,world,x[i],q_tmp);
    //transPlanPR2(active_joints,world_plan,world,xd[i],qd_tmp);
    xP.append(~q_tmp);
    xdP.append(~qd_tmp);
  }

  CtrlMsg refs;


  ////////////////////////////////////////////////////////////////////////////////////////
  // END of PLanning
  ////////////////////////////////////////////////////////////////////////////////////////

  arr zero_qdot(qdot.N);
  zero_qdot.setZero();


  /////////////////////////////////////////////////////////////////////////////////
  // FROM old code
  ////////////////////////////////////////////////////////////////////////////////
  MP.qitselfPD.active=true;


  ors::Shape *endeff = world.getShapeByName("endeffR");
  ors::Shape *true_target = world.getShapeByName("truetarget");
  ors::Body *est_target = world.getBodyByName("target");
  ors::Body *table = world.getBodyByName("table");



  est_target->X.pos.z = target(2);
  est_target->X.pos.y = target(1);
  est_target->X.pos.x = target(0);


  PDtask *pd_y =  MP.addPDTask("position", .1, .8, new DefaultTaskMap(posTMT, world, "endeffR", NoVector, "target", NoVector));

  //pd_y->setTarget(ARR(target(0),target(1),target(2)));
  pd_y->prec = 10.;

  //joint space PD task
  PDtask *pd_x = MP.addPDTask("pose", .1, .8, new TaskMap_qItself());
  pd_x->prec = .1;


  ConstraintForceTask *pd_c ;
  PlaneConstraint *plane_constraint = new PlaneConstraint(world, "endeffR", ARR(0,0,-1, table->X.pos.z + 0.02));
  pd_c = MP.addConstraintForceTask("planeConstraint", plane_constraint);  

  bool replanning = false;

  for(uint t=0;t<x.d0-150 ;t++){
      mlr::wait(.3);
      MP.setState(q, qdot);
      // world.gl().add(switchToNormal);    

      arr f_r = S.wrenchR.get();// ctrl_obs.get()->fR;
      cout<< f_r<<endl;
      if((type==0)&&(f_r(0)>-7.7)&&(t>5)){ //detected height

      //if((type==0)&&(endeff->X.pos.z <= table->X.pos.z + 0.03 )){ //detected height

          // replanning
          if(!replanning){             
              est_target->X.pos.z = endeff->X.pos.z; // estimate table's height
              table->X.pos.z = endeff->X.pos.z - 0.02;
              //pd_y->setTarget(ARR(target(0),target(1),endeff->X.pos.z));
              //table->X.pos.z = est_target->X.pos.z+ 0.02;

              cout<<"height is: "<< table->X.pos.z <<"  " <<endeff->X.pos.z <<endl;

              replanning = true;
              break; //found the height
          }
      }
      if((type==1)&&(f_r(0)<-8.0)&&(t>4)){

          est_target->X.pos.z = endeff->X.pos.z;
          est_target->X.pos.y = endeff->X.pos.y;
          est_target->X.pos.x = endeff->X.pos.x;// estimate table's height
          //pd_y->setTarget(ARR(target(0),est_target->X.pos.z,target(2)));
          //table->X.pos.z = est_target->X.pos.z+ 0.02;

          cout<<" found " <<endeff->X.pos.y <<endl;

          break;

      }
      if((type==4)&&(f_r(0)<-7.0)&&(t>4)){

          est_target->X.pos.z = endeff->X.pos.z;
          est_target->X.pos.y = endeff->X.pos.y;
          est_target->X.pos.x = endeff->X.pos.x;// estimate table's height
          //pd_y->setTarget(ARR(target(0),est_target->X.pos.z,target(2)));
          //table->X.pos.z = est_target->X.pos.z+ 0.02;

          cout<<" found " <<endeff->X.pos.y <<endl;

          break;

      }
      if((type==2)&&(f_r(0)<-9.0)&&(t>4)){ //detected height

          est_target->X.pos.z = endeff->X.pos.z; // estimate table's height
          //pd_y->setTarget(ARR(target(0),est_target->X.pos.z,target(2)));
          //table->X.pos.z = est_target->X.pos.z+ 0.02;

          cout<<" found " <<endeff->X.pos.y <<endl;

          break;

      }
      if((type==3)&&(f_r(1)>4.0)&&(t>4)){ //detected height

          est_target->X.pos.z = endeff->X.pos.z; // estimate table's height
          //pd_y->setTarget(ARR(target(0),est_target->X.pos.z,target(2)));
          //table->X.pos.z = est_target->X.pos.z+ 0.02;

          cout<<" found " <<endeff->X.pos.y <<endl;

          break;

      }

      if(type==-2){
        arr temp;
        temp.resize(3); temp(0) =endeff->X.pos.x;temp(1) =endeff->X.pos.y;temp(2) =endeff->X.pos.z;
        double dis = sqrDistance(temp,center);
        if (dis<0.0001)  break;
      }

    ////////////////////////////////////////////////////////////////////////////////
      if(t<y.d0){
         // pd_y->y_ref   = y[t];
          pd_x->y_ref   = xP[t];
          pd_c->desiredForce = dual(t);
    }
    ////////////////////////////////////////////////////////////////////////////////



   ////////////////////////////////////////////////////////////////////////////////
    //compute control

    for(uint tt=0;tt<10;tt++){
      arr a = MP.operationalSpaceControl();
      q += .001*qdot;
      qdot += .001*a;
    }
    MP.world.gl().update(STRING("local operational space controller state t="<<(double)t/100.), false, false, false);
    refs.q = q;
    refs.qdot = zeros(q.N);
    refs.u_bias = zeros(q.N);
    S.ctrl_ref.set() = refs;


    ////////////////////////////////////////////////////////////////////////////////
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



}

// ============================================================================
int main(int argc, char** argv)
{


  mlr::initCmdLine(argc,argv);

  ors::KinematicWorld world("model.kvg");
  ////////////////////////////////////////////////////////////////////////////////////////
  ors::KinematicWorld world_plan("model_reduced.kvg");

  arr q, qdot; // joints states of pr2 world
  arr qP,qPdot; // joints states of planned world


  world.getJointState(q, qdot);
  world_plan.getJointState(qP,qPdot);

  /// set list of active joints for remapping between world_pr2 and world_plan
  mlr::Array<mlr::String> active_joints;
  for (uint i = 0;i<world_plan.joints.d0;i++) {
    if (world_plan.joints(i)->type != 10 && world_plan.joints(i)->name!="frame_door") {
      active_joints.append(world_plan.joints(i)->name);
      cout << world_plan.joints(i)->name << " " << world_plan.joints(i)->type << " "  << world_plan.joints(i)->qIndex<< endl;
    }
  }




  uint T = 200; //time horizon
  ors::Body *table = world.getBodyByName("table");
  double tableW = table->shapes(0)->size[1];
  double tableL = table->shapes(0)->size[0];
  double tableT = table->shapes(0)->size[2];
  double x,y,z;
  x = table->X.pos.x;
  y = table->X.pos.y;
  z = table->X.pos.z;


  mlr::timerStart(true);


  for(uint i=0;i<10;i++){
      MySystem S;
      threadOpenModules(true);
      //makeConvexHulls(world.shapes);

      world >>FILE("z.ors");

      //ors::Joint *trans=world.getJointByName("worldTranslationRotation");
      ors::Shape *ftL_shape=world.getShapeByName("endeffR");

      //world.gl().add(ors::glDrawGraph, &worldCopy);



      bool useRos = mlr::getParameter<bool>("useRos", false);
      if(useRos){
        //-- wait for first q observation!
        cout <<"** Waiting for ROS message on initial configuration.." <<endl;
        for(;;){
          S.ctrl_obs.var->waitForNextRevision();
          if(S.ctrl_obs.get()->q.N==world.q.N
             && S.ctrl_obs.get()->qdot.N==world.q.N)
            break;
        }
        //-- set current state
        cout <<"** GO!" <<endl;
        //q =S.ctrl_obs.get()->q;

        //cout<<q<<endl;
        arr a1= ARR(0.0984512, 0.139828, 0.045318, -0.702808, 1.2121);
        arr a2 = ARR(-0.386625, -0.188665, 0.15997, -1.701, 0.52572);
        arr a3 = ARR(-1.30566, -0.151259, 0.0181052, -0.244004, -0.735694);
        arr a4= ARR(-0.0731164, -1.3283, 0.0195418, 0.0302764, 0.0264636);
        arr a5=ARR(0.00514072, 0.00444588);
        arr temp;
        temp.append(a1);temp.append(a2);temp.append(a3);temp.append(a4);temp.append(a5);
        q=temp;
        /*/
        q(0)=0.0990116;
        q(1)=0.114905;q(2)= 0.0438641;q(3)= -1.30231;q(4)= 1.2189;q(5)= -0.386939;q(6)= 0.162319;
        q(7)=0.173336;q(8)= -1.38366; q(9)=0.527324; q(10)=-1.67975; q(11)=-0.186583;q(12)= -2.43866;
        q(12)=0.41742; q(13)=-0.24854;q(14)= -0.0804803;q(15)= 0.472149;q(16)= -1.44728;
        q(17)=0.0272069; q(18)=0.0137558;q(19)= 0.00458136; q(20)=0.00212836;
        /*/
        //arr fL_base = S.fL_obs.get();
        world.setJointState(q);
        transPR2Plan(active_joints,world,world_plan,q,qP);
        world_plan.setJointState(qP);
        //MP.setState(q, qdot);
      }else{
          transPR2Plan(active_joints,world,world_plan,q,qP);
          world_plan.setJointState(qP);
      }
      world.watch(false);
      world_plan.watch(false);     
      ////////////////////////////////////////////////////////////////////////////////////////
      //END OF INIT: reading first pose, then set to world_plan
      ////////////////////////////////////////////////////////////////////////////////////////

      mlr::wait(3.);




      cout<<" [TRIAL] "<<i<<"     ################################################# "<<endl;

      int numSamples = 1000;
      std::vector<std::vector<double> > samples;


      for(int i=0;i<numSamples;i++){
          std::vector<double> temp;
          temp.resize(6);
          temp[0] = x + 0.05*rnd.gauss() - tableL/2.;
          temp[1] = x + 0.05*rnd.gauss() + tableL/2.;
          temp[2] = z + 0.05*rnd.gauss();
          temp[3] = y + 0.05*rnd.gauss() - tableW/2.;
          temp[4] = y + 0.05*rnd.gauss() + tableW/2.;
          temp[5] = tableT; //thicknes
          samples.push_back(temp);
      }
      ///////// ENTROPY /////////////////////////////////////////////
      arr Sigma = diag(0.05*0.05,5);
      cout<< Sigma <<endl;
      double initialEntropy0 = log(pow(2.*3.14*exp(1.),5)*determinant(Sigma))/2.0;
      cout<<"initialEntropy =  "<< initialEntropy0 <<endl;

      ////////////////////////////////////////////////////////////////
      int numMeanTakenActs;//for computing the average cost for each ACTION

      /// entropy after detecting HEIGHT: margin of prunning hypothesis is 0.0001
      arr Sigma_h = diag(0.05*0.05,5); Sigma_h[4](4)= 0.001*0.001;

      double initialEntropy_h = log(pow(2.*3.14*exp(1.),5)*determinant(Sigma_h))/2.0;
      cout<<"initialEntropy of DETECT HEIGHT =  "<< initialEntropy_h <<endl;
      double gain = initialEntropy0 - initialEntropy_h; //expectation of the observation
      cout<< "gain of DETECT_HEIGHT "<<gain<<endl;
      numMeanTakenActs = 1;//need only action
      cout<< "gain AFTER COST of DETECT_HEIGHT "<<(double)gain/numMeanTakenActs<<endl;

      /// entropy after detecting EDGE: margin of prunning hypothesis is 0.001
      arr Sigma_e = diag(0.001*0.001,5);
      double initialEntropy_e = log(pow(2.*3.14*exp(1.),5)*determinant(Sigma_e))/2.0;
      cout<<"initialEntropy of DETECT EDGE =  "<< initialEntropy_e <<endl;
      double prob_o = pow((0.001/0.05),5);

      //double gain_e = initialEntropy0 - (prob_o*initialEntropy_e + (1-prob_o)*initialEntropy0); //expectation of the observation
      double gain_e = initialEntropy0 - initialEntropy_e; //expectation of the observation
      cout<< "gain of DETECT_EDGE "<<gain_e<<endl;
      numMeanTakenActs = numSamples*(numSamples+1)/2;//sum of 1+2+...numSamples (#taken actions until the task finishes)
      numMeanTakenActs /= numSamples;//uniform distribution

      cout<< "gain AFTER COST of DETECT_EDGE "<<(double)gain/numMeanTakenActs<<endl;
      //////////////////////////////////////////////////////////////////////////////////////////////////


    double HEIGHT, LEFT,RIGHT, REAR, FRONT;


     // DETECT_HEIGHT     
      arr target;
      target.resize(3);
      target(0) = ftL_shape->X.pos.x;
      target(1) = ftL_shape->X.pos.y;
      target(2) = 0.36; //lowest height of the sampled table //assuming having biggest IGvietna

      arr terminalPos;

      arr center;

      OnlineSubmodularity(q,qdot,S,active_joints,tableW, tableL, world,world_plan, i, target, 0, center);



      //remember the starting point for DETECT-EDGE

      center.resize(3);
      center(0) = ftL_shape->X.pos.x;
      center(1) = ftL_shape->X.pos.y;
      center(2) = ftL_shape->X.pos.z + 0.02;
      HEIGHT = ftL_shape->X.pos.z + 0.02;
      ///////////////////////////////////////////////////////////


      target.resize(3);
      target(0) = ftL_shape->X.pos.x;
      target(1) = ftL_shape->X.pos.y + 0.5;
      target(2) = ftL_shape->X.pos.z + 0.02;
      OnlineSubmodularity(q,qdot,S,active_joints,tableW, tableL, world,world_plan, i, target, 4, center);
      //return back to the starting point
      cout<<"right: " << ftL_shape->X.pos.y <<endl;
      RIGHT = ftL_shape->X.pos.y;
      OnlineSubmodularity(q,qdot,S,active_joints,tableW, tableL, world,world_plan, i, center, -2, center);

      target.resize(3);
      target(0) = ftL_shape->X.pos.x;
      target(1) = ftL_shape->X.pos.y - 0.3;
      target(2) = ftL_shape->X.pos.z + 0.02;
      OnlineSubmodularity(q,qdot,S,active_joints,tableW, tableL, world,world_plan, i, target, 1, center);
      //return back to the starting point
      cout<<"left: " << ftL_shape->X.pos.y <<endl;
      LEFT = ftL_shape->X.pos.y;
      OnlineSubmodularity(q,qdot,S,active_joints,tableW, tableL, world,world_plan, i, center, -2, center);

      target.resize(3);
      target(0) = ftL_shape->X.pos.x - 0.4;
      target(1) = ftL_shape->X.pos.y;
      target(2) = ftL_shape->X.pos.z + 0.02;
      OnlineSubmodularity(q,qdot,S,active_joints,tableW, tableL, world,world_plan, i, target, 2, center);
      //return back to the starting point
      //
      cout<<"rear: " << ftL_shape->X.pos.x <<endl;
      REAR = ftL_shape->X.pos.x;
      OnlineSubmodularity(q,qdot,S,active_joints,tableW, tableL, world,world_plan, i, center, -2, center);

      target.resize(3);
      target(0) = ftL_shape->X.pos.x + 0.4;
      target(1) = ftL_shape->X.pos.y;
      target(2) = ftL_shape->X.pos.z + 0.02;
      OnlineSubmodularity(q,qdot,S,active_joints,tableW, tableL, world,world_plan, i, target, 3, center);
      //return back to the starting point
      //
      cout<<"front: " << ftL_shape->X.pos.x <<endl;
      FRONT = ftL_shape->X.pos.x;
      //OnlineSubmodularity(q,qdot,S,active_joints,tableW, tableL, world,world_plan, i, center, -2, terminalPos);
/*/
      target.resize(3);
      target(0) = ftL_shape->X.pos.x + 0.4;
      target(1) = ftL_shape->X.pos.y;
      target(2) = ftL_shape->X.pos.z + 0.02;
      OnlineSubmodularity(q,qdot,S,active_joints,tableW, tableL, world,world_plan, i, target, 1, terminalPos);
      //return back to the starting point
      OnlineSubmodularity(q,qdot,S,active_joints,tableW, tableL, world,world_plan, i, center, -2, terminalPos);
      cout<<"front: " << ftL_shape->X.pos.x <<endl;
/*/
      //GO TO NEW CENTER
      double WID = fabs(RIGHT-LEFT);
      double LL = fabs(FRONT-REAR);

      cout<< "WID "<<WID <<" LEFT= "<<LEFT <<" RIGHT= "<<RIGHT<<" REAR = "<<REAR<<" LL ="<<LL<<" HEIGHT="<<HEIGHT<<endl;
      target.resize(3);
      target(0) = REAR + LL/2.;
      target(1) = LEFT + WID/2;
      target(2) = HEIGHT;
      OnlineSubmodularity(q,qdot,S,active_joints,tableW, tableL, world,world_plan, i, target, -2, center);

      cout<<"DONE: " <<endl;


       threadCloseModules();

    }



  cout <<"bye bye" <<endl;exit(0);



  return 0;
}
