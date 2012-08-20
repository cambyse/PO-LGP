#include <unistd.h>
#include <MT/array.h>
#include <MT/util.h>
#include <hardware/hardware.h>
#include <hardware/kinect.h>
#include <motion/motion.h>
#include <motion/FeedbackControlTasks.h>
#include <biros/logging.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/passthrough.h>

SET_LOG(main, DEBUG);
typedef pcl::PointXYZRGBA PointT;

bool savePoint = false, selectionFinished = false;
bool newFrame;
int finger = 0;
// All homogenous!!
MT::Array<double> Y, X, A, kinectPt, orsPt;

void pointPickingOccured (const pcl::visualization::PointPickingEvent &event, void* viewer_void)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = 
    *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
  float x1,x2,x3;
  event.getPoint(x1,x2,x3);
  std::cout << "left mouse button pressed => (" << x1 << ", " << x2 << ", " << x3 << ") buffered" << std::endl;
  kinectPt = ARR(x1,x2,x3,1);
}

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = 
    *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
  if (event.getKeySym () == "m" && event.keyDown ())
  {
    std::cout << "m pressed => saving point" << std::endl;
    savePoint = true;
  }

  if (event.getKeySym () == "x" && event.keyDown ())
  {
    std::cout << "x pressed => will now calibrate" << std::endl;
    selectionFinished = true;
  }
  
  if (event.getKeySym () == "z" && event.keyDown ())
  {
    std::cout << "z pressed => new Kinect frame" << std::endl;
    newFrame = true;
  }
}

void calcTransformationMatrix() {
  arr Xt, At;
  transpose(Xt, X);
  At = inverse(Xt * X) * Xt * Y;
  transpose(A, At);
  std::stringstream error;
  error << "Calibration error: " << norm(A*Xt-Y);
  INFO(main, error.str().c_str() );
}

int main(int argn, char **argv) {

  MT::initCmdLine(argn,argv);
  //ThreadInfoWin win;
  //win.threadLoopWithBeat(.1);

  //X.resize(0,4);
  //Y.resize(0,4);

  // variables
  GeometricState geometricState;
  MotionPrimitive controllerTask;
  MotionFuture motionFuture;
  HardwareReference hardwareReference;
  SkinPressure skinPressure;
  JoystickState joystickState;

  PointCloudVar kinectData3d("KinectData3D");
  Image kinectDataRGB("KinectDataRGB");

  // processes
  Controller controller;
  Joystick joystick;
  SchunkArm schunkArm;
  SchunkHand schunkHand;

  KinectInterface kinect("Kinect");

  PoseViewer<HardwareReference> view(hardwareReference);

  ProcessL hardware=LIST<Process>(schunkArm, schunkHand, joystick);

  ProcessL P=LIST<Process>(controller, view); //, , schunkSkin, 

  
  
  cout <<"** setting controller to joystick mode" <<endl;
  Joystick_FeedbackControlTask joyTask;
  controllerTask.setFeedbackTask(joyTask, true, false, NULL);
  //view.threadLoopWithBeat(.01);
  loopWithBeat(hardware, .01); // hardware must be started before the controller
  loopWithBeat(P,.01);

	kinect.threadOpen();
	kinect.threadLoop();

  boost::shared_ptr<pcl::visualization::PCLVisualizer> 
    viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

  viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&viewer);
  viewer->registerPointPickingCallback (pointPickingOccured, (void*)&viewer);

  viewer->setBackgroundColor (255, 255, 255);
  viewer->addCoordinateSystem (.1);

  viewer->initCameraParameters ();

  while (true) {
    pcl::PointCloud<PointT>::Ptr cloud = 
      kinectData3d.get_point_cloud(NULL)->makeShared(); // TODO: perform copy in getter method!

    // this is just for convenience, if the arm is not in the cloud, set filter max to a higher value
    pcl::PassThrough<PointT> passthrough;
    passthrough.setInputCloud(cloud);
    passthrough.setFilterFieldName("z");
    passthrough.setFilterLimits(0,3.1);
    passthrough.filter(*cloud);

    viewer->addPointCloud(cloud);

    while(!viewer->wasStopped() && !newFrame && !selectionFinished){

      viewer->updatePointCloud(cloud, "cloud");
      viewer->spinOnce();
      //geometricState.writeAccess(NULL);
      //geometricState.ors.calcBodyFramesFromJoints();
      //geometricState.deAccess(NULL);

      if (savePoint) {
        geometricState.writeAccess(NULL);
        //geometricState.ors.calcBodyFramesFromJoints();
        //std::cout << geometricState.ors.bodies(1)->inLinks(0)->Q << std::endl;
        //MT::String shape("tipPoint");
        //shape << finger + 1;
        //std::cout << shape  << std::endl;
        double* orsPos = geometricState.ors.getShapeByName("marker")->X.pos.p;
        //ors::Vector orsPos = geometricState.ors.getBodyByName("m9")->X.pos;
        //std::cout << orsPos << std::endl;
        //ors::Shape *s = geometricState.ors.getShapeByName("marker");
        //geometricState.ors.kinematics(orsPt,s->body->index,&s->rel.pos);
        geometricState.deAccess(NULL);
        //std::cout << *orsPos << *(orsPos+1) << *(orsPos+2);
        orsPt = ARR( *orsPos, *(orsPos+1), *(orsPos+2), 1);
        std::cout << "Corresponding ors point is " << orsPt << std::endl;
        Y.append(orsPt);
        Y.resize(Y.N/4, 4);
        X.append(kinectPt);
        X.resize(X.N/4, 4);
        //std::cout << "Y dim = " << Y.nd << "\t X dim = " << X.nd << std::endl;
        //std::cout << X << std::endl;
        //std::cout << Y << std::endl;
        savePoint = false;
        finger = (finger+1)%3;
      }
    }

    //std::cout << "Finger tips are at " << std::endl;
    //for (int i = 0; i < 3; i++) {
      //geometricState.writeAccess(NULL);
      //geometricState.ors.calcBodyFramesFromJoints();
      //MT::String shape("tipPoint");
      //shape << i + 1;
      //std::cout << shape << std::endl;
      //double* orsPos = geometricState.ors.getShapeByName(shape)->X.pos.p;
      //geometricState.deAccess(NULL);
      //orsPt = ARR( *orsPos, *(orsPos+1), *(orsPos+2), 1);
      //std::cout << ors << std::endl;
    //}
    newFrame = false;
    viewer->removePointCloud();

    if (selectionFinished) {
      calcTransformationMatrix();
      std::cout << A << std::endl;
      break;
    }
  }
  
  viewer.reset();
  close(P);
  return 0;
}
