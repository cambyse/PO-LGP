#pragma once

#include <ros/ros.h>
#include <Core/module.h>


/** MACRO to create a MLR module that integrates data from ROS into the MLR
 *  module system.
 *
 *  This macro creates a class "ROS_#var_name" that subscribes the given
 *  topic_name with the given msg_type.  Each received ros msg is copied into
 *  the MLR module system.
 *
 *  To use it you have to
 *  - add the created module "ROS_#var_name"  as module to your system
 *  - add the ACCESS variable with the ros msg type to your system
 *
 *  See ../../examples/pr2/generic_ros_sync/main.cpp for more.
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


#define BEGIN_ROSMODULE(topic_name, msg_type, var_name) \
  class ROSMODULE_##var_name : public Module { \
  public: \
    ROSMODULE_##var_name() : Module(#var_name) {} \
    void open() { \
      rosCheckInit(); \
      this->_nh = new ros::NodeHandle; \
      this->_sub  = this->_nh->subscribe( \
        topic_name, 1, &ROSMODULE_##var_name::callback, this); \
    } \
    void step(); \
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
  public: \
    ACCESS(msg_type, var_name)

#define END_ROSMODULE() \
  };
