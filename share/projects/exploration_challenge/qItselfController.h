#include "pr2System.h"
#include <hybrid_automaton/Controller.h>
#include <Eigen/Dense>

class pr2System;


class qItselfController : public ha::Controller {
  public:
    typedef boost::shared_ptr<qItselfController> Ptr;
    typedef boost::shared_ptr<const qItselfController> ConstPtr;
    qItselfController();
    /**
     * @brief Activate the controller for execution. Is called automatically from the ControlSet
     */
    virtual void initialize();

    /**
     * @brief Deactivate the controller for execution. Is called automatically from the ControlSet
     */
    virtual void terminate();

    /**
     * @brief Step function - gets called in each control cycle
     */
    virtual ::Eigen::MatrixXd step(const double& t);

    /**
     * @brief Transform a goal in relative coordinates into absolute coordinates.
     *
     * Controllers should provide an interface to add a "delta" value  \a goalRel to the currents system state.
     * i.e. in joint space, \a goalRel is a configuration that must be added to the current robots position.
     * in task space, goalRel is a frame that must be multiplied.
     *
     * If your Controller does not overload this method, you can not use \a relative goals.
     *
     * @returns the transformed goal , i.e. \a current + \a goalRel
     */
    virtual ::Eigen::MatrixXd relativeGoalToAbsolute(const Eigen::MatrixXd& goalRel) const;

    virtual void setGoal(const Eigen::MatrixXd& new_goal);

    virtual void setEndeff(const std::string& endeff);

//    virtual void setIndices(const Eigen::MatrixXd& index_vec) {
//       _index_vec = index_vec;
//    }

  private:
    std::string _fact;

    std::string _create_fact() const;

    //Eigen::MatrixXd _index_vec;
    bool _running;

    std::string _endeff;
};
