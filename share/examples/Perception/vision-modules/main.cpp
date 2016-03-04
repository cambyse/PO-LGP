#include <Gui/graphview.h>
#include <Perception/perception.h>


void TEST(ModuleVision) {
  cout <<registry() <<endl;

  OpencvCamera cam;
  CvtGray cvtGray;
//  MotionFilter mofi;
//  DifferenceFilter difi("rgb", "ground", "diffImage");
  CannyFilter canniFi("gray", "canny");

//  ImageViewer iv1("rgb");
//  ImageViewer iv2("gray");
//  ImageViewer iv3("motion");
//  ImageViewer iv4("diffImage");
  ImageViewer iv5("canny");

  cout <<registry() <<endl;

  threadOpenModules(true);
  for(uint i=0;i<30;i++){
    mlr::wait(1.);
    modulesReportCycleTimes();
  }
  threadCloseModules();

  cout <<"bye bye" <<endl;
}

int main(int argc,char **argv) {
  mlr::initCmdLine(argc,argv);

  testModuleVision();

  return 0;
}
