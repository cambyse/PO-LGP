#pragma once

#include "activity.h"


//===========================================================================
/// Just call step on each Activity, that's it.
struct ActivitySpinnerModule : Module{
  ACCESS(ActivityL, A)

  ActivitySpinnerModule() : Module("ActivitySpinnerModule") {}

  /// @name module implementations
  void open(){}
  void step(){
    A.readAccess();
    for(Activity *act:A()) act->step(0.01);
    A.deAccess();
  }
  void close(){}
};
