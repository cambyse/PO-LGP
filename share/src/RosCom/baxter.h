#include <Core/array.h>
#include <Core/module.h>

#ifdef MLR_ROS
#include <sensor_msgs/JointState.h>
#include <Ors/ors.h>
bool baxter_update_qReal(arr& qReal, const sensor_msgs::JointState& msg, const ors::KinematicWorld& baxterModel);
#endif

struct SendPositionCommandsToBaxter:Module{
  Access_typed<arr> q_ref;
  struct sSendPositionCommandsToBaxter *s;

  SendPositionCommandsToBaxter();
  ~SendPositionCommandsToBaxter(){}

  void open();
  void step();
  void close();
};
