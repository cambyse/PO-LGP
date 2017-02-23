#include "act_ComRos.h"

#include <RosCom/roscom.h>
#include <RosCom/spinner.h>

Thread* newRosComSpinner(){ return new RosCom_Spinner(); }
