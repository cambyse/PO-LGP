#pragma once

#include <Core/module.h>
#include <Motion/feedbackControl.h>
#include <pr2/roscom.h>
#include <pr2/rosalvar.h>
#include <FOL/relationalMachine.h>
#include <pr2/RTControllerSimulation.h>

#ifdef MLR_ROS
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#endif


struct SetOfDataFiles{
  mlr::Array<ofstream*> files;
  void open(const StringA& names){
    for(auto& name: names){
      files.append(new ofstream(STRING("z/" <<name)));
    }
  }
  void write(const arrL& data){
    CHECK_EQ(data.N, files.N, "");
    for(uint i=0; i<data.N; i++) *files(i) <<*data(i) <<endl;
  }
  ~SetOfDataFiles(){
    for(auto& file: files){
      file->close();
      delete file;
    }
  }
};


/// The task controller generates the message send to the RT_Controller
/// the problem is defined by the list of CtrlTasks
struct TaskControllerModule : Module {
  //protected access points
  ACCESSnew(CtrlMsg, ctrl_ref) //< the message send to the RTController
  ACCESSnew(CtrlMsg, ctrl_obs) //< the message received from the RTController
  ACCESSnew(mlr::Array<CtrlTask*>, ctrlTasks)
  ACCESSnew(mlr::String, effects)
  ACCESSnew(ors::KinematicWorld, modelWorld)
  ACCESSnew(AlvarMarkers, ar_pose_marker)
  ACCESSnew(bool, fixBase)
  ACCESSnew(arr, pr2_odom)

  //non-protected members
//private:
  ors::KinematicWorld realWorld;
  FeedbackMotionControl *feedbackController;
  arr q_real, qdot_real; //< real state
  arr q_model, qdot_model; //< model state
  const arr q0; //< homing pose
  bool oldfashioned;
  bool useRos;
  bool isInSyncWithRobot;
  bool syncModelStateWithReal; //< whether the step() should reinit the state from the ros message
  bool verbose;
  bool useDynSim;
  RTControllerSimulation* dynSim;
  CtrlTask *noTaskTask;

  arr q_history, qdot_last, a_last, q_lowPass, qdot_lowPass, qddot_lowPass, aErrorIntegral, u_lowPass;
  arr model_error_g;

  SetOfDataFiles dataFiles;

public:
  TaskControllerModule(const char* modelFile=NULL);
  ~TaskControllerModule();

  void open();
  void step();
  void close();
};
