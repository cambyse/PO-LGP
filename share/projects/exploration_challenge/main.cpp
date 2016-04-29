#include "util.h"
#include <Core/array.h>
#include <Eigen/Dense>
#include <iostream>

#include "pr2ControlSet.h"
#include "pr2System.h"
#include "qItselfController.h"
#include "OpSpaceController.h"
#include "MLRFactory.h"

#include <hybrid_automaton/JumpCondition.h>
#include <hybrid_automaton/SubjointConfigurationSensor.h>
#include <hybrid_automaton/HybridAutomaton.h>
#include <hybrid_automaton/HybridAutomatonAbstractFactory.h>

#include <Actions/swig.h>

void createPullAndPushAtPoseHA(MLRFactory::Ptr mlr_factory,
                               ha::HybridAutomaton::Ptr actuate_affordance_ha,
                               Eigen::Vector3d goal_op_pos,
                               Eigen::Matrix3d goal_op_rot_matrix,
                               Eigen::MatrixXd retreat_op_pos,
                               Eigen::MatrixXd retreat_op_rot_matrix,
                               Eigen::VectorXd approach_op_pos,
                               Eigen::MatrixXd approach_op_rot_matrix){
    MLRFactoryParams p;
    p.endeff = "endeffR";
    p.gripper = "r_gripper_joint";
    std::cout<<"generating pull push automaton"<<std::endl;

    //do we use an approach pose to initialize the interaction?
    bool with_approach = approach_op_pos.size() != 0 && approach_op_rot_matrix.size() != 0;

    std::string goto_home_name = "goto_home";

    /////////////////////////////////////////////////////////////
    //Create failure Mode as
    //gravity compensation control mode
    ha::ControlMode::Ptr failure_cm(new ha::ControlMode());
    mlr_factory->CreateGCCM(p, failure_cm, "failure");

    //add controlmode to HA
    actuate_affordance_ha->addControlMode(failure_cm);



    /////////////////////////////////////////////////////////////
    /// create pregrasp control mode that inflates the hand just a little bit
    ///
    ha::ControlMode::Ptr pregrasp_cm(new ha::ControlMode());
    ha::ControlSwitch::Ptr pregrasp_cs(new ha::ControlSwitch());
    mlr_factory->CreateGraspCMAndCS(p, pregrasp_cm, pregrasp_cs, "Pregrasp");
    actuate_affordance_ha->addControlMode(pregrasp_cm);
    actuate_affordance_ha->setCurrentControlMode(pregrasp_cm->getName());

    ha::ControlMode::Ptr move_towards_approach_cm;
    ha::ControlSwitch::Ptr move_towards_approach_convergence_cs;
    ha::ControlSwitch::Ptr move_towards_approach_force_exceeded_cs;


    //if we use an approach position, we have to generate the control mode
    //and add it to the ha
    if(with_approach){
        std::cout<<"WITH approach"<<std::endl;
        //Create first Mode
        //Move towards handle
        move_towards_approach_cm=ha::ControlMode::Ptr(new ha::ControlMode());
        move_towards_approach_convergence_cs=ha::ControlSwitch::Ptr(new ha::ControlSwitch());
        move_towards_approach_force_exceeded_cs=ha::ControlSwitch::Ptr(new ha::ControlSwitch());
        std::string move_towards_approach_name = "move_towards_approach";

        //Force-Torque sensor parameters
        Eigen::MatrixXd ft_weights = Eigen::MatrixXd::Constant(6,1,0.0);
        ft_weights(0) = 1;
        ft_weights(1) = 1;
        ft_weights(2) = 1;
        Eigen::MatrixXd ft_max_value = Eigen::MatrixXd::Constant(6,1,0.0);
        ft_max_value(0) = 0;
        ft_max_value(1) = 0;
        ft_max_value(2) = 0;
        double ft_epsilon = 7;

        //epsilon in operational space for the convergence condition
        p._pos_epsilon_os_linear = 0.02;
        p._pos_epsilon_os_angular = 0.04;

        mlr_factory->CreateGoToCMConvergenceCSAndMaxForceCS(p,
                                                            move_towards_approach_cm,
                                                            move_towards_approach_convergence_cs,
                                                            move_towards_approach_force_exceeded_cs,
                                                            move_towards_approach_name,
                                                            approach_op_pos,
                                                            approach_op_rot_matrix,
                                                            ft_weights,
                                                            ft_max_value,
                                                            ha::JumpCondition::NORM_L2,
                                                            false,
                                                            true,
                                                            ft_epsilon,
                                                            false );
        //also, go to failure mode if a maximum time of 120 sec is exceeded
        ha::ControlSwitch::Ptr move_towards_approach_time_cs = mlr_factory->CreateMaxTimeControlSwitch(p, move_towards_approach_name, 120);
        actuate_affordance_ha->addControlSwitchAndMode(pregrasp_cm->getName(),pregrasp_cs, move_towards_approach_cm);
        actuate_affordance_ha->addControlSwitch(move_towards_approach_cm->getName(), move_towards_approach_time_cs, failure_cm->getName());


    } else {
        std::cout<<"without approach"<<std::endl;
    }

    /////////////////////////////////////////////////////////////
    //Move towards operational space target
    ha::ControlMode::Ptr move_towards_target_cm(new ha::ControlMode());
    ha::ControlSwitch::Ptr move_towards_target_convergence_cs(new ha::ControlSwitch());
    ha::ControlSwitch::Ptr move_towards_target_force_exceeded_cs(new ha::ControlSwitch());
    std::string move_towards_target_name = "move_towards_handle";

    //ForceTorque
    Eigen::MatrixXd ft_weights = Eigen::MatrixXd::Constant(6,1,0.0);
    ft_weights(0) = 1;
    ft_weights(1) = 1;
    ft_weights(2) = 1;
    Eigen::MatrixXd ft_max_value = Eigen::MatrixXd::Constant(6,1,0.0);
    ft_max_value(0) = 0;
    ft_max_value(1) = 0;
    ft_max_value(2) = 0;
    double ft_epsilon = 7;

    mlr_factory->CreateGoToCMConvergenceCSAndMaxForceCS(p, move_towards_target_cm,
                                                        move_towards_target_convergence_cs,
                                                        move_towards_target_force_exceeded_cs,
                                                        move_towards_target_name,
                                                        goal_op_pos,
                                                        goal_op_rot_matrix,
                                                        ft_weights,
                                                        ft_max_value,
                                                        ha::JumpCondition::NORM_L2,
                                                        false,
                                                        true,
                                                        ft_epsilon, false);

    if(with_approach){
        actuate_affordance_ha->addControlSwitchAndMode(move_towards_approach_cm->getName(), move_towards_approach_convergence_cs, move_towards_target_cm);
        actuate_affordance_ha->addControlSwitch(move_towards_approach_cm->getName(), move_towards_approach_force_exceeded_cs, move_towards_target_cm->getName());
    }
    else
    {
        actuate_affordance_ha->addControlSwitchAndMode(pregrasp_cm->getName(), pregrasp_cs, move_towards_target_cm);
    }

    ha::ControlSwitch::Ptr move_towards_handle_time_cs = mlr_factory->CreateMaxTimeControlSwitch(p, move_towards_target_name, 120);
    actuate_affordance_ha->addControlSwitch(move_towards_target_cm->getName(), move_towards_handle_time_cs, failure_cm->getName());

    /////////////////////////////////////////////////////////////
    //Create grasp control mode (aka inflate the hand)
    //Grasp
    ha::ControlMode::Ptr grasp_cm(new ha::ControlMode());
    ha::ControlSwitch::Ptr grasp_cs(new ha::ControlSwitch());
    p.grasp_strength = .1;
    mlr_factory->CreateGraspCMAndCS(p, grasp_cm, grasp_cs, "Grasp");

    //add controlmode and switch to HA
    actuate_affordance_ha->addControlSwitchAndMode(move_towards_target_cm->getName(), move_towards_target_convergence_cs, grasp_cm);
    actuate_affordance_ha->addControlSwitch(move_towards_target_cm->getName(), move_towards_target_force_exceeded_cs, grasp_cm->getName());


    ha::ControlMode::Ptr unrotate_cm(new ha::ControlMode());
    ha::ControlSwitch::Ptr unrotate_convergence_cs(new ha::ControlSwitch());
    ha::ControlSwitch::Ptr unrotate_force_exceeded_cs(new ha::ControlSwitch());
    std::string unrotate_name = "unrotate";
    goal_op_pos << 0.0,0,0.0;
    //rotate hand_x 30 degrees around hand_y, because we want to approach in a certain way
    Eigen::Vector3d hand_y_ee = Eigen::Vector3d(0,1,0);
    Eigen::AngleAxis<double> rotation_back(-0.3, hand_y_ee);//-0.3
    goal_op_rot_matrix = rotation_back.toRotationMatrix();
    mlr_factory->CreateGoToCMConvergenceCSAndMaxForceCS(p,
                                                        unrotate_cm,
                                                        unrotate_convergence_cs,
                                                        unrotate_force_exceeded_cs,
                                                        unrotate_name,
                                                        goal_op_pos,
                                                        goal_op_rot_matrix,
                                                        ft_weights,
                                                        ft_max_value,
                                                        ha::JumpCondition::NORM_L2,
                                                        false,
                                                        true,
                                                        ft_epsilon,
                                                        true);


//    const ha::JumpCondition::JumpCriterion ft_criterion,
//    bool use_base,
//    bool negate_ft_condition,
//    double ft_epsilon,
//    double max_vel_os_linear,
//    double max_vel_os_angular,
//    double pos_epsilon_os_linear,
//    double pos_epsilon_os_angular,
//    bool is_relative,
//    const Eigen::MatrixXd &kp_os_linear,
//    const Eigen::MatrixXd &kp_os_angular,
//    const Eigen::MatrixXd &kv_os_linear,
//    const Eigen::MatrixXd &kv_os_angular)
//{



    actuate_affordance_ha->addControlSwitchAndMode(grasp_cm->getName(),grasp_cs, unrotate_cm);

        bool articulate_relative = true;

        /////////////////////////////////////////////////////////////
        //Pull (beginning of wobbling)
        ha::ControlMode::Ptr push_down_cm(new ha::ControlMode());
        ha::ControlSwitch::Ptr push_down_convergence_cs(new ha::ControlSwitch());
        ha::ControlSwitch::Ptr push_down_force_exceeded_cs(new ha::ControlSwitch());
        if(articulate_relative) goal_op_pos << 0.25,0,0.1;
        else goal_op_pos  += Eigen::Vector3d( 0,0.0,-0.1);
        Eigen::MatrixXd goal_op_rot_matrix2 = Eigen::MatrixXd();

        mlr_factory->CreateGoToCMConvergenceCSAndMaxForceCS(p,
                                                            push_down_cm,
                                                            push_down_convergence_cs,
                                                            push_down_force_exceeded_cs,
                                                            "push_down",
                                                            goal_op_pos,
                                                            goal_op_rot_matrix2,
                                                            ft_weights,
                                                            ft_max_value,
                                                            ha::JumpCondition::NORM_L2,
                                                            false,
                                                            true,
                                                            ft_epsilon,
                                                            articulate_relative);

        actuate_affordance_ha->addControlSwitchAndMode(unrotate_cm->getName(),unrotate_convergence_cs, push_down_cm);
        actuate_affordance_ha->addControlSwitch(unrotate_cm->getName(),unrotate_force_exceeded_cs, push_down_cm->getName());

    /////////////////////////////////////////////////////////////
    //Pull (beginning of wobbling)
    ha::ControlMode::Ptr pull_cm(new ha::ControlMode());
    ha::ControlSwitch::Ptr pull_convergence_cs(new ha::ControlSwitch());
    ha::ControlSwitch::Ptr pull_force_exceeded_cs(new ha::ControlSwitch());
    if(articulate_relative) goal_op_pos << 0,0,-0.1;
    else goal_op_pos  += Eigen::Vector3d( 0,-0.2,0);

    mlr_factory->CreateGoToCMConvergenceCSAndMaxForceCS(p,
                                                        pull_cm,
                                                        pull_convergence_cs,
                                                        pull_force_exceeded_cs,
                                                        "pull",
                                                        goal_op_pos,
                                                        goal_op_rot_matrix2,
                                                        ft_weights,
                                                        ft_max_value,
                                                        ha::JumpCondition::NORM_L2,
                                                        false,
                                                        true,
                                                        ft_epsilon,
                                                        articulate_relative);

    actuate_affordance_ha->addControlSwitchAndMode(push_down_cm->getName(),push_down_force_exceeded_cs, pull_cm);
    /////////////////////////////////////////////////////////////
    //Push forward (second motion of wobbling)
    ha::ControlMode::Ptr push_forward_cm(new ha::ControlMode());
    ha::ControlSwitch::Ptr push_forward_convergence_cs(new ha::ControlSwitch());
    ha::ControlSwitch::Ptr push_forward_force_exceeded_cs(new ha::ControlSwitch());

    if(articulate_relative) goal_op_pos << 0,0,0.2;
    else goal_op_pos += Eigen::Vector3d( 0,0.1,0);
//    ft_weights(2) = 1;
//    ft_max_value(2) = -20;

    mlr_factory->CreateGoToCMConvergenceCSAndMaxForceCS(p,
                                                        push_forward_cm,
                                                        push_forward_convergence_cs,
                                                        push_forward_force_exceeded_cs,
                                                        "push_forward",
                                                        goal_op_pos,
                                                        goal_op_rot_matrix2,
                                                        ft_weights,
                                                        ft_max_value,
                                                        ha::JumpCondition::NORM_L2,
                                                        false,
                                                        true,
                                                        ft_epsilon,
                                                        articulate_relative);

    //add controlmode and switch to HA
    actuate_affordance_ha->addControlSwitchAndMode(pull_cm->getName(), pull_force_exceeded_cs, push_forward_cm);


    /////////////////////////////////////////////////////////////
    //Create fifth Mode
    //Push right (third motion of wobbling)
    ha::ControlMode::Ptr push_right_cm(new ha::ControlMode());
    ha::ControlSwitch::Ptr push_right_convergence_cs(new ha::ControlSwitch());
    ha::ControlSwitch::Ptr push_right_force_exceeded_cs(new ha::ControlSwitch());

    if(articulate_relative) goal_op_pos << 0,-0.2,0;
    else goal_op_pos += Eigen::Vector3d( 0.2,0,0);
//    ft_weights(2) = 0;
//    ft_weights(1) = 1;
//    ft_max_value(2) = 0;
//    ft_max_value(1) = 20;

    mlr_factory->CreateGoToCMConvergenceCSAndMaxForceCS(p,
                                                        push_right_cm,
                                                        push_right_convergence_cs,
                                                        push_right_force_exceeded_cs,
                                                        "push_right",
                                                        goal_op_pos,
                                                        goal_op_rot_matrix2,
                                                        ft_weights,
                                                        ft_max_value,
                                                        ha::JumpCondition::NORM_L2,
                                                        false,
                                                        true,
                                                        ft_epsilon,
                                                        articulate_relative);

    //add controlmode and switch to HA
    actuate_affordance_ha->addControlSwitchAndMode(push_forward_cm->getName(), push_forward_force_exceeded_cs, push_right_cm);


    /////////////////////////////////////////////////////////////
    //Create sixth Mode
    //Push left (last motion of wobbling)
    ha::ControlMode::Ptr push_left_cm(new ha::ControlMode());
    ha::ControlSwitch::Ptr push_left_convergence_cs(new ha::ControlSwitch());
    ha::ControlSwitch::Ptr push_left_force_exceeded_cs(new ha::ControlSwitch());

    if(articulate_relative) goal_op_pos << 0,0.2,0;
    else goal_op_pos  += Eigen::Vector3d( -0.2,0.0,0.0);
//    ft_weights(1) = 1;
//    ft_max_value(1) = -20;

    mlr_factory->CreateGoToCMConvergenceCSAndMaxForceCS(p,
                                                        push_left_cm,
                                                        push_left_convergence_cs,
                                                        push_left_force_exceeded_cs,
                                                        "push_left",
                                                        goal_op_pos,
                                                        goal_op_rot_matrix2,
                                                        ft_weights,
                                                        ft_max_value,
                                                        ha::JumpCondition::NORM_L2,
                                                        false,
                                                        true,
                                                        ft_epsilon,
                                                        articulate_relative);


    //add controlmode and switch to HA
    actuate_affordance_ha->addControlSwitchAndMode(push_right_cm->getName(), push_right_force_exceeded_cs, push_left_cm);

    /////////////////////////////////////////////////////////////
    //Create  Mode
    //ungrasp
    ha::ControlMode::Ptr ungrasp_cm(new ha::ControlMode());
    //ha::ControlSwitch::Ptr ungrasp_pressure_cs(new ha::ControlSwitch());
    ha::ControlSwitch::Ptr ungrasp_cs(new ha::ControlSwitch());
    //mlr_factory->CreateUngraspCMAndCS(ungrasp_cm, ungrasp_pressure_cs, ungrasp_time_cs, "Ungrasp", _gripper_type);
    p.grasp_strength = 1;
    mlr_factory->CreateGraspCMAndCS(p, ungrasp_cm, ungrasp_cs, "Ungrasp");

    //add controlmode and switch to HA
    actuate_affordance_ha->addControlSwitchAndMode(push_left_cm->getName(), push_left_force_exceeded_cs, ungrasp_cm);


    actuate_affordance_ha->addControlSwitch(push_down_cm->getName(), push_down_convergence_cs, ungrasp_cm->getName());
    actuate_affordance_ha->addControlSwitch(pull_cm->getName(), pull_convergence_cs, ungrasp_cm->getName());
    actuate_affordance_ha->addControlSwitch(push_forward_cm->getName(), push_forward_convergence_cs, ungrasp_cm->getName());
    actuate_affordance_ha->addControlSwitch(push_right_cm->getName(), push_right_convergence_cs, ungrasp_cm->getName());
    actuate_affordance_ha->addControlSwitch(push_left_cm->getName(), push_left_convergence_cs, ungrasp_cm->getName());









    ////////////////////////////////////////////////////////////////////////////////////////////////////
    //Safe retreat
    ha::ControlMode::Ptr move_towards_retreat_cm;
    ha::ControlSwitch::Ptr move_towards_retreat_convergence_cs;
    ha::ControlSwitch::Ptr move_towards_retreat_force_exceeded_cs;

    move_towards_retreat_cm=ha::ControlMode::Ptr(new ha::ControlMode());
    move_towards_retreat_convergence_cs=ha::ControlSwitch::Ptr(new ha::ControlSwitch());
    move_towards_retreat_force_exceeded_cs=ha::ControlSwitch::Ptr(new ha::ControlSwitch());
    std::string move_towards_retreat_name = "move_towards_retreat";

    Eigen::MatrixXd ft_weights_retreat = Eigen::MatrixXd::Constant(6,1,0.0);
    ft_weights_retreat(0) = 1;
    ft_weights_retreat(1) = 1;
    ft_weights_retreat(2) = 1;

    Eigen::MatrixXd ft_max_value_retreat = Eigen::MatrixXd::Constant(6,1,0.0);
    ft_max_value_retreat(0) = 0;
    ft_max_value_retreat(1) = 0;
    ft_max_value_retreat(2) = 0;

    double ft_epsilon_retreat = 12;

    mlr_factory->CreateGoToCMConvergenceCSAndMaxForceCS(p,
                                                        move_towards_retreat_cm,
                                                        move_towards_retreat_convergence_cs,
                                                        move_towards_retreat_force_exceeded_cs,
                                                        move_towards_retreat_name,
                                                        retreat_op_pos,
                                                        retreat_op_rot_matrix,
                                                        ft_weights_retreat,
                                                        ft_max_value_retreat,
                                                        ha::JumpCondition::NORM_L2,
                                                        false,
                                                        true,
                                                        ft_epsilon_retreat,
                                                        false);


    //add controlmode and switch to HA
    actuate_affordance_ha->addControlSwitchAndMode(ungrasp_cm->getName(), ungrasp_cs, move_towards_retreat_cm);
    //actuate_affordance_ha->addControlSwitch(ungrasp_cm->getName(), ungrasp_cs, move_towards_retreat_cm->getName());

    /////////////////////////////////////////////////////////////
    // Create  Mode
    // Go to Home Position
    ha::ControlMode::Ptr goto_home_cm(new ha::ControlMode());
    ha::ControlSwitch::Ptr goto_home_convergence_cs(new ha::ControlSwitch());

    int num_dof_arm= 7;
    Eigen::MatrixXd home_config_js_arm = Eigen::MatrixXd(num_dof_arm, 1);
   //home_config_js_arm << 0.0, -0.447759, 0.01341, 2.79048, 1.0711, 0.485972, -0.119184;
    home_config_js_arm << 0.0, -0.14, 0.0, 2.18, 0.0, 0.2, -0.13;
    mlr_factory->CreateGoToHomeCMAndConvergenceCSArm(p, goto_home_cm, goto_home_convergence_cs, goto_home_name);

    //add controlmode and switch to HA
    actuate_affordance_ha->addControlSwitchAndMode(move_towards_retreat_cm->getName(), move_towards_retreat_convergence_cs, goto_home_cm);
    actuate_affordance_ha->addControlSwitch(move_towards_retreat_cm->getName(), move_towards_retreat_force_exceeded_cs, failure_cm->getName());

    ///////////////////////////////////////////!!!!!!!/////////////////////////////////DEBUG


    /////////////////////////////////////////////////////////////
    //Create last success Mode
    //gravity compensation
    ha::ControlMode::Ptr success_cm(new ha::ControlMode());
    mlr_factory->CreateGCCM(p, success_cm, "finished");

    //add controlmode and switch to HA
    actuate_affordance_ha->addControlSwitchAndMode(goto_home_cm->getName(), goto_home_convergence_cs, success_cm);
}

void createSimpleHA(MLRFactory::Ptr mlr_factory,
                    ha::HybridAutomaton::Ptr ha,
                    Eigen::MatrixXd goal_pos,
                    Eigen::MatrixXd goal_rot) {
    MLRFactoryParams p;
    p._pos_epsilon_os_linear = .1;
    p._pos_epsilon_os_angular = .1;
    p.endeff = "endeffR";

    /////////////////////////////////////////////////////////////
    //Create failure Mode as
    //gravity compensation control mode
    ha::ControlMode::Ptr failure_cm(new ha::ControlMode());
    mlr_factory->CreateGCCM(p, failure_cm, "failure");

    //add controlmode to HA
    ha->addControlMode(failure_cm);

    /////////////////////////////////////////////////////////////

    ha::ControlMode::Ptr move_cm(new ha::ControlMode());
    ha::ControlSwitch::Ptr move_cs(new ha::ControlSwitch());

    mlr_factory->CreateGoToCMAndConvergenceCS(p, move_cm, move_cs, "move", goal_pos, goal_rot, true, false);

    ha->addControlMode(move_cm);
    ha->addControlSwitch(move_cm->getName(), move_cs, failure_cm->getName());

    ha->setCurrentControlMode(move_cm->getName());

}

int main(int argc, char** argv) {
  mlr::initCmdLine(argc, argv);

  ActionSwigInterface interface;
  pr2System::Ptr system(new pr2System(interface));
  MLRFactory::Ptr mlr_factory = MLRFactory::Ptr(new MLRFactory(system));

  ha::HybridAutomaton::Ptr hybrid_automaton(new ha::HybridAutomaton());

  hybrid_automaton->setVerbose(false);

  Eigen::Vector3d goal_op_pos;
  goal_op_pos << .7, .0, .7;
  Eigen::Matrix3d goal_op_rot_matrix;
  goal_op_rot_matrix << 1, 0, 0, 
                        0, 1, 0, 
                        0, 0, 1;
  Eigen::MatrixXd retreat_op_pos(3, 1);
  retreat_op_pos << .3, .3, .7;
  Eigen::MatrixXd retreat_op_rot_matrix(3, 3);
  retreat_op_rot_matrix << 1, 0, 0,
                           0, 1, 0,
                           0, 0, 1;
  Eigen::VectorXd approach_op_pos(3);
  approach_op_pos << .3, .3, .7;
  Eigen::MatrixXd approach_op_rot_matrix(3, 3);
  approach_op_rot_matrix << 1, 0, 0, 
                            0, 1, 0, 
                            0, 0, 1;
  createPullAndPushAtPoseHA(mlr_factory, hybrid_automaton, goal_op_pos, goal_op_rot_matrix, retreat_op_pos, retreat_op_rot_matrix, approach_op_pos, approach_op_rot_matrix );
  //createSimpleHA(mlr_factory, hybrid_automaton, goal_op_pos, goal_op_rot_matrix);
  hybrid_automaton->setSystem(system);
  hybrid_automaton->initialize(.0);
  double t = 0.;
  for(uint i=0; i<10000; ++i) {
    t += .01;
    hybrid_automaton->step(t);
    //std::cout << hybrid_automaton->getCurrentControlMode()->getName() << std::endl;

    //std::cout << "JC active: " << bigger->isActive() << std::endl;
    //std::cout << "Sensor value: " << bigger->getSensor()->getCurrentValue() << std::endl;
    mlr::wait(.1);
  }

  hybrid_automaton->terminate();
}
