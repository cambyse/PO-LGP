#pragma once

#include <Ors/ors.h>
#include <pr2/roscom.h>
#include "activity.h"


// ============================================================================
/**
 * A SensorActivity listens to sensor data and adds symbols to the relational machine.
 *
 * Braindump: what kind of sensor stuff
 * - thresholds
 *   - total threshold: currentValue > totalThreshold
 *   - rel threshold: currentValue - initialValue > relativeThreshold
 * - FT:
 *   - side
 *   - only F?
 *   - only T?
 *   - xyz?
 *   - sum?
 * - joint_effort
 *   - joint
 * - anything gripper specific?
 */
struct SensorActivity : Activity {

  // the ControlActivityManager is a quick and dirty way to access ACCESS variables
  struct ControlActivityManager *cam;
  // TODO don't use the ControlActivitiManager to access the variable anymore
  // ACCESS(CtrlMsg, ctrl_obs);
  // ACCESS(MT::String, effects);

  bool _isTriggered;
  double _threshold;

  // Implement base class methods: modify KB
  virtual void configure(Node *fact); ///< calls stepSensor and registers
  virtual void step(double dt); ///< calls stepSensor and sets facts

  // actually do control related stuff
  virtual void configureSensor(Graph& specs);
  virtual void stepSensor();

  virtual bool isTriggered();
};
