#include "SensorActivities.h"
#include <Control/TaskControllerModule.h>
#include <RosCom/roscom.h>

// ============================================================================

void SensorActivity::configure() {
  configureSensor(params);
  _isTriggered = false;
}

void SensorActivity::activitySpinnerStep(double dt) {
  activityTime += dt;

  stepSensor();

  mlr::String doneString = "(done ";
  if (isTriggered()) {
    cout << "I was isTriggered(): " << endl;

    if (!_isTriggered) {
      effects.set()() << STRING("(triggered SensorActivity), ");
    }
    _isTriggered = true;
  }
}

void SensorActivity::configureSensor(Graph& specs) {
  Node *it;
  if((it=specs["threshold"])) {
    _threshold = it->get<double>();
  }
  else {
    _threshold = 5.;
  }
  // cout << _threshold << endl;
}

void SensorActivity::stepSensor() {
}

bool SensorActivity::isTriggered() {
  CtrlMsg obs = ctrl_obs.get();
  // cout << "Left:  " << obs.fL << endl;
  // cout << "Right: " << obs.fR << endl;
  return (sum(obs.fL)) > _threshold;
}
