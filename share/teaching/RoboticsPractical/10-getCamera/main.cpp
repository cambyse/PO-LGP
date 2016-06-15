#include <RosCom/roscom.h>
#include <Perception/perception.h>
#include <RosCom/spinner.h>


void getCam(){
  rosCheckInit();
  Access_typed<byteA> rightArmCam(NULL, "rightArmCam");

  RosCom_Spinner spinner;
  ImageViewer viewer("kinect_rgb");

  threadOpenModules(true);

  SubscriberConvNoHeader<sensor_msgs::Image, byteA, &conv_image2byteA> sub("/camera/left_hand_camera/image", rightArmCam);

  for(uint i=0;i<100;i++){
    rightArmCam.waitForNextRevision();
    cout <<'.' <<flush;
//    mlr::wait(.1);
  }

  threadCloseModules();
  modulesReportCycleTimes();
  cout <<"bye bye" <<endl;
}

int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);

  getCam();

  return 0;
}
