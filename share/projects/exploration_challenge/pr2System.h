#pragma once

#include <hybrid_automaton/System.h>
#include <Eigen/Dense>
#include <Ors/ors.h>
#include <pr2/roscom.h>
#include <Actions/swig.h>

class pr2System : public ha::System {
  public:
    typedef boost::shared_ptr<pr2System> Ptr;
    typedef boost::shared_ptr<const pr2System> ConstPtr;

    ActionSwigInterface& interface;

    pr2System(ActionSwigInterface& interface);

    /**
     * @brief Return the number degrees of freedom of your system
     */
		virtual int getDof() const;

    /**
     * @brief Return the current joint configuration vector (dimx1)
     */
		virtual ::Eigen::MatrixXd getJointConfiguration() const;

    /**
     * @brief Return the current joint velocity vector (dimx1)
     */
		virtual ::Eigen::MatrixXd getJointVelocity() const;

    /**
     * @brief Return the current Force-torque mieasurement of your sensor (6x1)
     */
		virtual ::Eigen::MatrixXd getForceTorqueMeasurement(const std::string& frame_id) const;

    /**
     * @brief Return the pose of a frame with id \a frame_id (4x4) wrt the
     * robot's base frame
     */
		virtual ::Eigen::MatrixXd getFramePose(const std::string& frame_id) const;

    virtual void step() {};
};

typedef boost::shared_ptr<const pr2System> pr2SystemConstPtr;
