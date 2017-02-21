#include "act_ComRos.h"

#include <RosCom/roscom.h>
#include <RosCom/spinner.h>

Act_ComRos::Act_ComRos(Roopi* r)
  : Act(r){
  rosCheckInit("Roopi");
  rosSpinner = new RosCom_Spinner();
  rosSpinner->threadLoop();
}

Act_ComRos::~Act_ComRos(){
  rosSpinner->threadClose();
  delete rosSpinner;
}
