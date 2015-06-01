#pragma once
#include <Core/module.h>


/** MACRO to create a MLR module that integrates data from ROS into the MLR
 *  module system.
 *
 *  This macro creates a class "ROS_#var_name" that subscribes the given
 *  topic_name with the given msg_type.  The ros msg is basically copied into
 *  the MLR module system.
 *
 *  You have to add the created class and the ACCESS variable to your System
 *  and you're good to go.
 *
 *  See ../../examples/pr2/generic_ros_sync/main.cpp for more.
 *
 *  TODO Refactor to avoid includes in header files. But we need the type of
 *  the msg. Is it even possible?
 */
#define ROSSUB(topic_name, msg_type, var_name) \
  class ROSSUB_##var_name : public Module { \
  public: \
    ACCESS(msg_type, var_name) \
    ROSSUB_##var_name() : Module(#var_name) {} \
    void open() { \
      rosCheckInit(); \
      this->_nh = new ros::NodeHandle; \
      this->_sub  = this->_nh->subscribe( \
        topic_name, 1, &ROSSUB_##var_name::callback, this); \
    } \
    void step() {} \
    void close() { \
      this->_nh->shutdown(); \
      delete _nh; \
    } \
    void callback(const msg_type::ConstPtr& msg) { \
      this->var_name.set() = *msg; \
    } \
  private: \
    ros::NodeHandle* _nh; \
    ros::Subscriber _sub; \
    \
  };
