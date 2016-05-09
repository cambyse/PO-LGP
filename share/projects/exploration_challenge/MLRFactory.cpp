#include "MLRFactory.h"
#include "pr2ControlSet.h"
#include "qItselfController.h"
#include "OpSpaceController.h"
#include <hybrid_automaton/ClockSensor.h>


ha::ControlSet::Ptr MLRFactory::createControlSet(const ha::HybridAutomatonAbstractParams& params,
                                                                   ha::Controller::Ptr ctrl) {
    ctrl->setSystem(system);
    ha::ControlSet::Ptr cs(new pr2ControlSet());
    return cs;;
}

ha::ControlSet::Ptr MLRFactory::createControlSet(const ha::HybridAutomatonAbstractParams& params,
                                                                   const std::vector<ha::Controller::Ptr>& ctrls) {
    ha::ControlSet::Ptr cs(new pr2ControlSet());
    for (auto ctrl : ctrls) {
        ctrl->setSystem(system);
        cs->appendController(ctrl);
    }

    return cs;;
}

ha::ControlSet::Ptr MLRFactory::createTaskSpaceControlSet(const ha::HybridAutomatonAbstractParams& params,
                                                                  ha::Controller::Ptr ctrl,
                                                                  bool move_base ) {
    ha::ControlSet::Ptr cs(new pr2ControlSet());
    ctrl->setSystem(system);
    cs->appendController(ctrl);
    return cs;;
}


ha::Controller::Ptr MLRFactory::createGraspController(const ha::HybridAutomatonAbstractParams& params,
                                                              std::string name) {

    MLRFactoryParams& p = (MLRFactoryParams&) params;
    qItselfController::Ptr close_gripper(new qItselfController());
    close_gripper->setSystem(system);
    close_gripper->setEndeff(p.gripper);
    close_gripper->setName(name);
    ::Eigen::MatrixXd goal(1, 1);
    goal(0, 0) = params.grasp_strength;
    close_gripper->setGoal(goal);
    return close_gripper;
}

ha::Controller::Ptr MLRFactory::createJointSpaceController(const ha::HybridAutomatonAbstractParams& params,
                                                       std::string name,
                                                       const Eigen::MatrixXd &goal_js,
                                                       double completion_time,
                                                       bool goal_relative) {
    ::Eigen::MatrixXd index_vec = ::Eigen::MatrixXd::Ones(system->getDof(), 1);
    auto ctrl = createSubjointSpaceController(params, name, goal_js, index_vec, goal_relative);
    ctrl->setSystem(system);
    return ctrl;
}

ha::Controller::Ptr MLRFactory::createSubjointSpaceController(const ha::HybridAutomatonAbstractParams& params,
                                                          std::string name,
                                                          const Eigen::MatrixXd& goal_js,
                                                          const Eigen::MatrixXd& index_vec,
                                                          bool is_relative) {
    qItselfController::Ptr ctrl(new qItselfController());
    ctrl->setSystem(system);
    //ctrl->setIndices(index_vec);
    ctrl->setEndeff("-");
    ctrl->setGoal(goal_js);
    ctrl->setName(name);
    return ctrl;
}

ha::Controller::Ptr MLRFactory::createBBSubjointSpaceController(const ha::HybridAutomatonAbstractParams& params,
                                                            std::string name,
                                                            bool use_tf,
                                                            const std::string& topic_name,
                                                            const std::string& tf_parent,
                                                            const Eigen::MatrixXd& index_vec,
                                                            bool is_relative) {
    NIY;
}

ha::Controller::Ptr MLRFactory::createBBSubjointSpaceControllerBase(const ha::HybridAutomatonAbstractParams& params,
                                                                const std::string name,
                                                                bool use_tf,
                                                                const std::string& topic_name,
                                                                const std::string& tf_parent,
                                                                bool is_relative) {
    NIY;
}

ha::Controller::Ptr MLRFactory::createOperationalSpaceController(const ha::HybridAutomatonAbstractParams& params,
                                                             std::string name,
                                                             const Eigen::MatrixXd &goal_op_translation,
                                                             const Eigen::MatrixXd &goal_op_rot_matrix,
                                                             double completion_time,
                                                             bool is_relative) {
    return createOperationalSpaceController(params, name, goal_op_translation, goal_op_rot_matrix, is_relative);
}

ha::Controller::Ptr MLRFactory::createOperationalSpaceController(const ha::HybridAutomatonAbstractParams& params,
                                                             std::string name,
                                                             const Eigen::MatrixXd &goal_op_translation,
                                                             const Eigen::MatrixXd &goal_op_rot_matrix,
                                                             bool is_relative) {
    MLRFactoryParams& p = (MLRFactoryParams&) params;
    OpSpaceController::Ptr ctrl(new OpSpaceController());
    Eigen::MatrixXd bin_home_frame;
    bin_home_frame.resize(4,4);
    bin_home_frame.setIdentity();
    if(goal_op_rot_matrix.size() != 0)
    {
        bin_home_frame.block(0,0,3,3) = goal_op_rot_matrix;
    }
    else {
        ctrl->setOnlyDisplacement(true);
    }
    bin_home_frame.block(0,3,3,1) = goal_op_translation;

    ctrl->setSystem(system);

    //Endeffector Frame Controller
    ctrl->setName(name);

    ctrl->setGoal(bin_home_frame);
    ctrl->setGoalIsRelative(is_relative);
    ctrl->setEndeff(p.endeff);

    return ctrl;
}

void MLRFactory::CreateGCCM(const ha::HybridAutomatonAbstractParams& p,
                            const ha::ControlMode::Ptr& cm_ptr,
                            const std::string& name){
    cout << "GCCM" << endl;
    cm_ptr->setName(name);
    ha::ControlSet::Ptr gravity_cs(new pr2ControlSet());
    cm_ptr->setControlSet(gravity_cs);
}
