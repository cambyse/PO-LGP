#include "pr2System.h"
#include <hybrid_automaton/ControlMode.h>
#include <hybrid_automaton/ControlSwitch.h>
#include <hybrid_automaton/HybridAutomatonAbstractFactory.h>
#include <memory>

class MLRFactory;

typedef std::shared_ptr<MLRFactory> MLRFactoryPtr;

class MLRFactoryParams : public ha::HybridAutomatonAbstractParams {
public:
    std::string endeff;
};

class MLRFactory : public ha::HybridAutomatonAbstractFactory {
public:
   virtual ha::ControlSet::Ptr createControlSet(const ha::HybridAutomatonAbstractParams& params,
                                                          ha::Controller::Ptr ctrl);
   virtual ha::ControlSet::Ptr createControlSet(const ha::HybridAutomatonAbstractParams& params,
                                                          const std::vector<ha::Controller::Ptr>& ctrls);
   virtual ha::ControlSet::Ptr createTaskSpaceControlSet(const ha::HybridAutomatonAbstractParams& params,
                                                         ha::Controller::Ptr ctrl,
                                                         bool move_base );

   virtual ha::Controller::Ptr createGraspController(const ha::HybridAutomatonAbstractParams& params,
                                                     std::string name);
   virtual ha::Controller::Ptr createJointSpaceController(const ha::HybridAutomatonAbstractParams& params,
                                                          std::string name,
                                                          const Eigen::MatrixXd &goal_js,
                                                          double completion_time,
                                                          bool goal_relative);
   virtual ha::Controller::Ptr createSubjointSpaceController(const ha::HybridAutomatonAbstractParams& params,
                                                             std::string name,
                                                             const Eigen::MatrixXd& goal_js,
                                                             const Eigen::MatrixXd& index_vec,
                                                             bool is_relative);
   virtual ha::Controller::Ptr createBBSubjointSpaceController(const ha::HybridAutomatonAbstractParams& params,
                                                               std::string name,
                                                               bool use_tf,
                                                               const std::string& topic_name,
                                                               const std::string& tf_parent,
                                                               const Eigen::MatrixXd& index_vec,
                                                               bool is_relative);
   virtual ha::Controller::Ptr createBBSubjointSpaceControllerBase(const ha::HybridAutomatonAbstractParams& params,
                                                                   const std::string name,
                                                                   bool use_tf,
                                                                   const std::string& topic_name,
                                                                   const std::string& tf_parent,
                                                                   bool is_relative);
   virtual ha::Controller::Ptr createOperationalSpaceController(const ha::HybridAutomatonAbstractParams& params,
                                                                std::string name,
                                                                const Eigen::MatrixXd &goal_op_translation,
                                                                const Eigen::MatrixXd &goal_op_rot_matrix,
                                                                double completion_time,
                                                                bool is_relative);
   virtual ha::Controller::Ptr createOperationalSpaceController(const ha::HybridAutomatonAbstractParams& params,
                                                                std::string name,
                                                                const Eigen::MatrixXd &goal_op_translation,
                                                                const Eigen::MatrixXd &goal_op_rot_matrix,
                                                                bool is_relative);



    virtual ha::Controller::Ptr createBBOperationalSpaceController(const ha::HybridAutomatonAbstractParams& params,
                                                                   std::string name,
                                                                   bool trajectory,
                                                                   bool use_tf,
                                                                   const std::string frame,
                                                                   const std::string parent_frame,
                                                                   bool is_relative) { NIY; };

    virtual void CreateGCCM(const ha::HybridAutomatonAbstractParams& p, const ha::ControlMode::Ptr& cm_ptr, const std::string& name);

    MLRFactory(pr2System::Ptr system) : system(system) {};
    MLRFactory(const MLRFactory& other) : system(other.system) {};

protected:
   virtual HybridAutomatonAbstractFactory* _doClone() const {
        return new MLRFactory(*this);

    }

private:
    pr2System::Ptr system;
};

//  private:
//    ha::HybridAutomatonFactory::Ptr _ha_factory;
//    pr2System::Ptr system;
//  public:
//    MLRFactory(ha::HybridAutomatonFactory::Ptr ha_factory, pr2System::Ptr system) : _ha_factory(ha_factory), system(system) {};

//    typedef std::shared_ptr<MLRFactory> Ptr;

//    void CreateGCCM(const ha::ControlMode::Ptr& cm_ptr, const std::string& name);

//    void CreateGraspCMAndCS(const ha::ControlMode::Ptr& cm_ptr,
//                            const ha::ControlSwitch::Ptr& cs_ptr,
//                            const std::string& name);

//    void CreateGoToCMConvergenceCSAndMaxForceCS(const ha::ControlMode::Ptr& cm_ptr,
//                                                const ha::ControlSwitch::Ptr& convergence_cs_ptr,
//                                                const ha::ControlSwitch::Ptr& max_force_cs_ptr,
//                                                const std::string& name,
//                                                const Eigen::MatrixXd &goal_op_pos,
//                                                const Eigen::MatrixXd &goal_op_ori,
//                                                const Eigen::MatrixXd &ft_idx,
//                                                const Eigen::MatrixXd &max_ft,
//                                                const ha::JumpCondition::JumpCriterion ft_criterion,
//                                                bool use_base,
//                                                bool negate_ft_condition=false,
//                                                double ft_epsilon=0.0,
//                                                double max_vel_os_linear = -1,
//                                                double max_vel_os_angular = -1,
//                                                double pos_epsilon_os_linear = -1,
//                                                double pos_epsilon_os_angular = -1,
//                                                bool is_relative = false,
//                                                const Eigen::MatrixXd &kp_os_linear = Eigen::MatrixXd(),
//                                                const Eigen::MatrixXd &kp_os_angular = Eigen::MatrixXd(),
//                                                const Eigen::MatrixXd &kv_os_linear = Eigen::MatrixXd(),
//                                                const Eigen::MatrixXd &kv_os_angular = Eigen::MatrixXd());

//    void CreateGoToCMAndConvergenceCS(const ha::ControlMode::Ptr& cm_ptr,
//                                      const ha::ControlSwitch::Ptr& cs_ptr,
//                                      const std::string& name,
//                                      const Eigen::MatrixXd &goal_op_pos,
//                                      const Eigen::MatrixXd &goal_op_ori,
//                                      double max_vel_os_linear,
//                                      double max_vel_os_angular,
//                                      double pos_epsilon_os_linear,
//                                      double pos_epsilon_os_angular,
//                                      bool is_relative);

//    ha::Controller::Ptr createOperationalSpaceController(std::string name,
//                                                         const Eigen::MatrixXd &goal_op_translation,
//                                                         const Eigen::MatrixXd &goal_op_rot_matrix,
//                                                         bool is_relative);
//};
