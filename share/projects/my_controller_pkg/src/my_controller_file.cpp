#include "my_controller_pkg/my_controller_file.h"
#include <pluginlib/class_list_macros.h>

namespace my_controller_ns {


/// Controller initialization in non-realtime
bool MyControllerClass::init(pr2_mechanism_model::RobotState *robot,
                             ros::NodeHandle &n)
{
  std::string joint_name;
  if (!n.getParam("joint_name", joint_name))
  {
    ROS_ERROR("No joint given in namespace: '%s')",
              n.getNamespace().c_str());
    return false;
  }

  joint_state_ = robot->getJointState(joint_name);
  if (!joint_state_)
  {
    ROS_ERROR("MyController could not find joint named '%s'", joint_name.c_str());
    return false;
  }

  setStateSrv_ = n.advertiseService("set_state", &MyControllerClass::setState, this);
  setGainSrv_ = n.advertiseService("set_gains", &MyControllerClass::setGains, this);
  setFilterSrv_ = n.advertiseService("set_filter", &MyControllerClass::setFilter, this);

  getStateSrv_ = n.advertiseService("get_state", &MyControllerClass::getState, this);
  getGainSrv_ = n.advertiseService("get_gains", &MyControllerClass::getGains, this);
  getFilterSrv_ = n.advertiseService("get_filter", &MyControllerClass::getFilter, this);

  capture_srv_ = n.advertiseService("capture", &MyControllerClass::capture, this);
  mystate_pub_ = n.advertise<my_controller_pkg::MyStateMessage>("mystate_topic", StoreLen);
  storage_index_ = StoreLen;

  return true;
}


/// Controller startup in realtime
void MyControllerClass::starting()
{
  init_pos_ = joint_state_->position_;
  des_pos = init_pos_;
  des_pos_intern = init_pos_;
  des_vel = 0.;
  filter_vel = 0.;

  p_gain = 0.;
  d_gain = 0.;
  i_gain = 0.;
  integral = 0.;
  p_effort = 0.;
  d_effort = 0.;
  i_effort = 0.;

  i_filt_param = 0.1;
  vel_filt_param = 0.95;
  i_max_param = 0.;

}


/// Controller update loop in realtime
void MyControllerClass::update()
{
  double current_pos = joint_state_->position_;
  double current_vel = joint_state_->velocity_;

  filter_vel = (vel_filt_param)*filter_vel + (1-vel_filt_param)*current_vel;
  //  integral = i_filt_param*integral + (des_vel - filter_vel);

  integral = i_filt_param*integral + (des_pos_intern - current_pos);
  //  des_pos = des_pos+0.001*filter_vel;
  //  des_vel = filter_vel + 0.001*des_acc;
  //  des_pos = current_pos + 0.001*des_vel;
  des_pos_intern = current_pos + 0.001*des_vel;


  p_effort = (p_gain * (des_pos_intern - current_pos));
  d_effort = (d_gain * (des_vel - filter_vel));
  i_effort = i_gain * integral;

  //  if (i_effort) > fabs(i_max_param*d_effort)) {
  //      i_effort = i_max_param*d_effort;
  //  }
  if (i_effort > i_max_param) {
    i_effort = i_max_param;
  } else if (i_effort < (-1.*i_max_param)) {
    i_effort = -1.*i_max_param;
  }

  joint_state_->commanded_effort_ = p_effort + d_effort + i_effort;


  int index = storage_index_;
  if ((index >= 0) && (index < StoreLen))
  {
    storage_[index].dt               = ros::Time::now().toSec();
    storage_[index].position         = current_pos;
    storage_[index].desired_position = des_pos_intern;
    storage_[index].velocity         = current_vel;
    storage_[index].desired_velocity = des_vel;
    storage_[index].commanded_effort = joint_state_->commanded_effort_;
    storage_[index].measured_effort  = joint_state_->measured_effort_;
    storage_[index].filter_vel = filter_vel;
    storage_[index].p_effort = p_effort;
    storage_[index].d_effort = d_effort;
    storage_[index].i_effort = i_effort;

    // Increment for the next cycle.
    storage_index_ = index+1;
  }
}


/// Controller stopping in realtime
void MyControllerClass::stopping()
{}



bool MyControllerClass::setState(my_controller_pkg::SetState::Request &req, my_controller_pkg::SetState::Response &resp)
{
  des_pos = req.pos;
  des_vel = req.vel;
  return true;
}

bool MyControllerClass::setGains(my_controller_pkg::SetGains::Request &req, my_controller_pkg::SetGains::Response &resp)
{
  p_gain = req.pos_gain;
  d_gain = req.vel_gain;
  i_gain = req.i_gain;
  ROS_INFO("GAIN CHANGED");
  return true;
}

bool MyControllerClass::setFilter(my_controller_pkg::SetFilter::Request &req, my_controller_pkg::SetFilter::Response &resp)
{
  i_filt_param = req.i_filt_param;
  vel_filt_param = req.vel_filt_param;
  i_max_param = req.i_max_param;
  storage_index_ = 0;
  integral = 0.;
  ROS_INFO("FILTER CHANGED");
  return true;
}

bool MyControllerClass::getState(my_controller_pkg::GetState::Request& req,  my_controller_pkg::GetState::Response& resp)
{
  resp.pos = joint_state_->position_;
  resp.vel = filter_vel;
  return true;
}

bool MyControllerClass::getGains(my_controller_pkg::GetGains::Request& req,  my_controller_pkg::GetGains::Response& resp)
{
  resp.pos_gain = p_gain;
  resp.vel_gain = d_gain;
  resp.i_gain = i_gain;
  return true;
}

bool MyControllerClass::getFilter(my_controller_pkg::GetFilter::Request& req,  my_controller_pkg::GetFilter::Response& resp)
{
  resp.i_filt_param = i_filt_param;
  resp.vel_filt_param = vel_filt_param;
  resp.i_max_param = i_max_param;
  return true;
}

/// Service call to capture and extract the data
bool MyControllerClass::capture(std_srvs::Empty::Request& req,
                                std_srvs::Empty::Response& resp)
{
  /* Publish the buffer contents. */
  int i;
  int  index = storage_index_;
  for (i = 0 ; i < index; i++)
    mystate_pub_.publish(storage_[i]);

  return true;
}

} // namespace

/// Register controller to pluginlib
PLUGINLIB_DECLARE_CLASS(my_controller_pkg,MyControllerPlugin,
                        my_controller_ns::MyControllerClass,
                        pr2_controller_interface::Controller)


