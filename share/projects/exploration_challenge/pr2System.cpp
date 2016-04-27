#include "pr2System.h"
#include "util.h"

#include <Core/array.h>
//#include <Core/array-eigen.h>
#include <Geo/geo.h>

pr2System::pr2System(ActionSwigInterface& interface) : ha::System(), interface(interface) {};

std::vector<double> eigen2std(::Eigen::MatrixXd mat) {
  return std::vector<double>(mat.data(), mat.data() + mat.size());  
}

::Eigen::MatrixXd std2eigen(std::vector<double> vec) {
  return ::Eigen::MatrixXd::Map(vec.data(), vec.size(), 1);
}

int pr2System::getDof() const {
  return const_cast<pr2System*>(this)->interface.getQDim();
}

/**
 * @brief Return the current joint configuration vector (dimx1)
 */
::Eigen::MatrixXd pr2System::getJointConfiguration() const {
    std::vector<double> q = const_cast<pr2System*>(this)->interface.getQ();
    return std2eigen(q);
}

/**
 * @brief Return the current joint velocity vector (dimx1)
 */
::Eigen::MatrixXd pr2System::getJointVelocity() const {
  std::vector<double> v = const_cast<pr2System*>(this)->interface.getV();
  return std2eigen(v);
}

/**
 * @brief Return the current Force-torque mieasurement of your sensor (6x1)
 */
::Eigen::MatrixXd pr2System::getForceTorqueMeasurement(const std::string& frame_id) const {
    //const_cast<pr2System*>(this)->ctrl_obs.waitForNextRevision();
    if(frame_id == "l_wrist_ft_sensor" or 
       frame_id == "ee") {
      return(std2eigen(interface.getForceLeft()));
    }
    else if(frame_id == "r_wrist_ft_sensor") {
      return(std2eigen(interface.getForceRight()));
    }
    else {
      HALT(frame_id << " is not a valid FT sensor frame.");
    }
}


::Eigen::MatrixXd pr2System::getFramePose(const std::string& frame_id) const {
    std::string frame = frame_id;
    if(frame == "EE") frame = "endeffR";
    ors::Transformation relative = const_cast<pr2System*>(this)->interface.getFramePose(frame);
    return mt2eigen(relative.getAffineMatrix());
}
