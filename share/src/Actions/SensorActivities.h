#pragma once

#include <Ors/ors.h>
#include <pr2/roscom.h>
#include "activity.h"


// ============================================================================
/**
 * A SensorActivityABC (an ABC) listens to sensor data and adds symbols to the relational machine.
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
  ACCESSname(CtrlMsg, ctrl_obs);
  ACCESSname(mlr::String, effects);

  bool _isTriggered;
  double _threshold;

  // Implement base class methods: modify KB
  virtual void configure(); ///< calls stepSensor and registers
  virtual void step(double dt); ///< calls stepSensor and sets facts

  // actually do control related stuff
  virtual void configureSensor(Graph& specs);
  virtual void stepSensor();

  virtual bool isTriggered();
};
