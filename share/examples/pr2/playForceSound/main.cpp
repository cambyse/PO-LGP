#include <RosCom/roscom.h>
#include <Actions/PlayForceSound.h>
#include <Hardware/gamepad/gamepad.h>


// =================================================================================================
int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);

  rosCheckInit("playForceSound");

  ACCESSname(arr, Fl);
  ACCESSname(arr, Fr);
  RosCom_Spinner spinner;
  GamepadInterface gp;
  SubscriberConvNoHeader<geometry_msgs::WrenchStamped, arr, &conv_wrench2arr> subL("/ft_sensor/l_ft_compensated", Fl);
  SubscriberConvNoHeader<geometry_msgs::WrenchStamped, arr, &conv_wrench2arr> subR("/ft_sensor/r_ft_compensated", Fr);
  PlayForceSoundActivity fs;

  threadOpenModules(true);

//  for(uint i=0;i<1000;i++){
//    cout <<Fl.get()() <<endl;
//    mlr::wait(.03);
//  }
  //mlr::wait(30.);
  moduleShutdown().waitForValueGreaterThan(0);

  threadCloseModules();
  cout <<"bye bye" <<endl;
  return 0;
}
