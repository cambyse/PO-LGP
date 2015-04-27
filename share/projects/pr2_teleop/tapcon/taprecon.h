#pragma once

#include <Motion/feedbackControl.h>
#include <System/engine.h>
// #include <pr2/actionMachine.h>
// #include <pr2/actions.h>
#include <Mocap/mocapdata.h>

// ============================================================================
struct TapRecon: Module {
  ACCESS(arr, gamepadState);
  ACCESS(floatA, poses);
	ACCESS(floatA, reconfeatures);

  bool calibration_phase_taprecon; ///< indicates if we're in the calibration phase
	floatA recordedKey;
	int tick=0;

  MocapID mid;

  TapRecon();

  void open() {}
  void close() {}
  void step();

};
