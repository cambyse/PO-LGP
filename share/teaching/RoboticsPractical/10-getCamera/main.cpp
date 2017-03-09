#include <RosCom/roscom.h>
#include <Perception/perception.h>
#include <RosCom/spinner.h>


void getCam(){
  rosCheckInit();
  Access_typed<byteA> rightArmCam(NULL, "rightArmCam");

  RosInit rosInit;
  ImageViewer viewer("rightArmCam");
  SubscriberConvNoHeader<sensor_msgs::Image, byteA, &conv_image2byteA> sub("/cameras/right_hand_camera/image", rightArmCam);
  RosCom_Spinner spinner;

  threadOpenModules(true);

  moduleShutdown().waitForStatusGreaterThan(0);

  threadCloseModules();

  threadReportCycleTimes();
  cout <<"bye bye" <<endl;
}

int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);

  getCam();

  return 0;
}
