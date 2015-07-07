#pragma once

#include <hybrid_automaton/ControlSet.h>
#include <hybrid_automaton/HybridAutomaton.h>

/**
 * @brief Interface for a ControlSet
 *
 * A ControlSet combines the output of several Controllers.
 * Some advanced control methods need to combine several subtasks into a control signal.
 *
 * An example for this would be closing a hand and maintaining arm position at the same time.
 * In this case a Controller for the hand and a Controller for the arm would be added to the ControlSet.
 *
 * Another example could be different forces at different operational points of the robot which
 * then are combined in the ControlSet using nullspace projections.
 */
class pr2ControlSet : public ha::ControlSet {

	public:
		typedef boost::shared_ptr<pr2ControlSet> Ptr;
		typedef boost::shared_ptr<const pr2ControlSet> ConstPtr;

		pr2ControlSet(); 
		pr2ControlSet(const pr2ControlSet& cs);
    ~pr2ControlSet();

        /**
        * @brief Activate the ControlSet for execution. Is called automatically from the ControlMode
        */
		virtual void initialize();

        /**
        * @brief Deactivate the ControlSet after execution. Is called automatically from the ControlMode
        */
		virtual void terminate();

        /**
        * @brief Compute the control signal from all contained Controllers. Is called in the control loop.
        */
		virtual ::Eigen::MatrixXd step(const double& t);
};
