#include "ros_import.h"

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <boost/bind.hpp>
#include <pcl/ros/conversions.h>

namespace MLR { namespace PCL {
    struct sFromROS {
        ros::NodeHandle n;
        ros::Subscriber pc_sub;
        cloud_cb *callback;

        sFromROS(const std::string& topic, cloud_cb *callback) :
            pc_sub(topic, 1, boost::bind(&sFromROS::msg_cb, this, _1)),
            callback(callback) {

        }

        void msg_cb(const sensor_msgs::PointCloud2& pc) {

        }
    };
}
FromROS::FromROS(const std::string& pc_topic, cloud_cb* callback) : s(new sFromROS(pc_topic, callback))
{
}

FromROS::~FromROS() {
    delete s;
}
