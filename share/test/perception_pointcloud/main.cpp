#include <perception/perception.h>
#include <hardware/kinect.h>
#include <motion/motion.h>

int main(int argc, char **argv) {
  MT::initCmdLine(argc,argv);

  srand(time(NULL));


  // Variables
  VariableL V = newPointcloudVariables();
  GeometricState geo;

  // Processes
  KinectInterface kinect("KinectInterface");
  ProcessL P = newPointcloudProcesses(1);

  kinect.threadOpen();

  new OrsView(geo.fields(0), NULL); //example for creating views directly from code 

  MT::wait();

}
