#include "amex_controller.h"

AmexController(ros::NodeHandle &_nh) {
  nh = _nh;
  dtAmex = MT::getParameter<double>("dtAmex");
  folder = MT::getParameter<String>("evalName");



  setPosTargetClient = nh.serviceClient<tree_controller_pkg::SetPosTarget>("/tree_controller/set_pos_target",true);
  setVecTargetClient = nh.serviceClient<tree_controller_pkg::SetVecTarget>("/tree_controller/set_vec_target",true);
//  getJointStateClient = nh.serviceClient<tree_controller_pkg::SetPosTarget>("/tree_controller/get_joint_state",true);
  setJointGainsClient = nh.serviceClient<tree_controller_pkg::SetJointGains>("/tree_controller/set_joint_gains",true);

  setPosTargetClient.waitForExistence();
  setVecTargetClient.waitForExistence();
  setJointGainsClient.waitForExistence();

//  for (uint i = 0; i<rt_set_gains_clients.size() ;i++) {
//    rt_set_state_clients[i].waitForExistence();
//    rt_get_state_clients[i].waitForExistence();
//    rt_set_filter_clients[i].waitForExistence();
//  }

  get_goal_client = nh.serviceClient<goal_publisher::GetGoal>("/get_goal",true);
  if (get_goal_client.waitForExistence(ros::Duration(3.0))){
    useGoalPub=true;
    get_goal_client.call(goalSub);
    cout << "goal from ar tracker: " << endl;
    cout << goalSub.response.x << goalSub.response.y << goalSub.response.z << endl;
  } else {
    useGoalPub=false;
    cout << "goal from target.ors file used" << endl;
  }
}

// initialize ors
void initOrs(int argc, char** argv, String scene) {
  MT::initCmdLine(argc,argv);

  world.init(scene);
#if VISUALIZATION
  world.gl().resize(800, 800);
#endif
  makeConvexHulls(world.shapes);
  world.getJointState(q, qd);
}
