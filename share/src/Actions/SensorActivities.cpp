#include "SensorActivities.h"
#include "TaskControllerModule.h"
#include "pr2/roscom.h"

// TODO I'm abusing TaskControllerModule to get the ctrl_obs variable.
//      Find a better way.
extern TaskControllerModule *taskControllerModule();

// ============================================================================
void SensorActivity::configure(Node *fact) {
  Activity::configure(fact);

  Graph* specs = getSpecsFromFact(fact);
  configureSensor(*specs);
  _isTriggered = false;
}

void SensorActivity::step(double dt) {
  activityTime += dt;

  stepSensor();

  MT::String doneString = "(done ";
  if (isTriggered()) {
    cout << "I was isTriggered(): " << endl;

    if (!_isTriggered) {
      // TODO fix modify KB
      auto* controlActivityManager = taskControllerModule();
      controlActivityManager->effects.set()() << STRING("(triggered SensorActivity), ");
    }
    _isTriggered = true;
  }
}

void SensorActivity::configureSensor(Graph& specs) {
  Node *it;
  if((it=specs["threshold"])) {
    _threshold = it->V<double>();
  }
  else {
    _threshold = 5.;
  }
  // cout << _threshold << endl;
}

void SensorActivity::stepSensor() {
}

bool SensorActivity::isTriggered() {
  auto* controlActivityManager = taskControllerModule();
  CtrlMsg obs = controlActivityManager->ctrl_obs.get();
  // cout << "Left:  " << obs.fL << endl;
  // cout << "Right: " << obs.fR << endl;
  return (sum(obs.fL)) > _threshold;
}
