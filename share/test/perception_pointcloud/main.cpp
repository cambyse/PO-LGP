#include <perception/perception.h>
#include <hardware/kinect.h>
#include <motion/motion.h>
#include <biros/biros.h>
#include <views/control.h>
#include <views/biros_views.h>

#include <omp.h>

int main(int argc, char **argv) {
  MT::initCmdLine(argc,argv);

  omp_set_num_threads(10);

  srand(time(NULL));


  // Variables
  VariableL V = newPointcloudVariables();
  GeometricState geo;

  // Processes
  KinectInterface kinect("KinectInterface");
  ProcessL P = newPointcloudProcesses();

  kinect.threadOpen();

  new OrsView(geo.fields(0), NULL); //example for creating views directly from code 
  b::openInsideOut();

  MT::wait();

}
