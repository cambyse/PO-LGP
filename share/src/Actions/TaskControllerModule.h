#pragma once

#include <Core/module.h>
#include <Control/ctrlMsg.h>
#include <Control/taskController.h>
#include <Control/RTControllerSimulation.h>
//#include <pr2/roscom.h>

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
  ACCESS(arr, ctrl_q_real)
  ACCESS(arr, ctrl_q_ref)

  ACCESS(CtrlMsg, ctrl_ref) //< the message send to the RTController
  ACCESS(CtrlMsg, ctrl_obs) //< the message received from the RTController
  ACCESS(mlr::Array<CtrlTask*>, ctrlTasks)
  ACCESS(mlr::String, effects)
  ACCESS(ors::KinematicWorld, modelWorld)
//  ACCESS(AlvarMarkers, ar_pose_marker)
  ACCESS(bool, fixBase)
  ACCESS(arr, pr2_odom)

  //non-protected members
//private:
  ors::KinematicWorld realWorld;
  TaskController *taskController;
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
