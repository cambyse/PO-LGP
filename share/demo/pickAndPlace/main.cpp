#include <motion/motion.h>
#include <perception/perception.h>
#include <perception/pointcloud.h>
#include <hardware/kinect.h>
#include <hardware/hardware.h>

#ifdef PCL
#include <pcl/visualization/pcl_visualizer.h>
#endif

#include <devTools/logging.h>

#include <biros/biros_views.h>
#include <biros/control.h>
#include <MT/ors.h>

SET_LOG(main, DEBUG);

/* What doesn't work yet:
 
 - collisions with the grasped object UNTIL the 4/5 time using a special proxy variable
 - feedback tasks (like open hand) have not termination criterion - fixed time is not ok!
*/

#include "behaviors.h"

int main(int argn,char** argv){
  MT::initCmdLine(argn, argv);
  //ThreadInfoWin win;
  //win.threadLoopWithBeat(.1);
  
  ProcessL P;

  //-- motion
  // variables
  GeometricState geometricState;
  MotionFuture motions;
  HardwareReference hardwareReference;
  SkinPressure skinPressure;
  JoystickState joystickState;
  
  // processes
  Process* ctrl = newMotionController(&hardwareReference, NULL, &motions);
  newActionProgressor(motions);
 
  // Perception
  //Variables                                  
  PointCloudVar kinectData3d("KinectData3D");   
  PointCloudSet objectClusters("ObjectClusters");
  ObjectSet objects("Objects"); 
  ObjectBeliefSet filteredObjects("filteredObjects");
  Workspace<FittingJob, FittingResult>
    fittingWorkspace("FittingWorkspace");    
  // Processes
  ProcessL Perception = newPointcloudProcesses(1);
  Perception(0)->threadOpen();
 
  while(objectClusters.get_point_clouds(NULL).N == 0) MT::wait(0.1);



//-- hardware
  // variables
  //(none)
  // processes
  SchunkArm schunkArm;
  SchunkHand schunkHand;
  SchunkSkin schunkSkin;
  ProcessL hardware = LIST<Process>(schunkArm, schunkHand, schunkSkin);

  // viewers
  //PoseView pose_view(hardwareReference.fields(0), NULL);
  //OrsView ors_view(geometricState.fields(0), NULL);

  // -- camera perception
  //Image camL("CameraL"), camR("CameraR");
  //Image hsvL("HsvL"), hsvR("HsvR");
  //FloatImage hsvEviL("hsvEviL"), hsvEviR("hsvEviR");
  //PerceptionOutput percOut;
  // processes
  //
  
  //Camera cam;
  //CvtHsv cvtHsv1(camL, hsvL);
  //CvtHsv cvtHsv2(camR, hsvR);
  //HsvFilter hsvFilterL(hsvL, hsvEviL);
  //HsvFilter hsvFilterR(hsvR, hsvEviR);
  //ShapeFitter shapeFitter(hsvEviL, hsvEviR, percOut);

  /////////////////////////////////////////////////////////////////////////////
  // inside-out

  //b::dump();
  b::openInsideOut();
  //

  loopWithBeat(hardware, .01);
  ctrl->threadLoopWithBeat(.01);

#if 0
  pcl::visualization::PCLVisualizer viewer ("3D Viewer");
  viewer.setBackgroundColor (0, 0, 0);
  viewer.addCoordinateSystem (.1);
  viewer.initCameraParameters ();
  PointCloudL plist = objectClusters.get_point_clouds(NULL);

  for(int j = 0; j<20; j++) {
    std::stringstream s;
    s << "cl_" << j;
    pcl::PointCloud<PointT>::Ptr tmp(new pcl::PointCloud<PointT>());
    viewer.addPointCloud(tmp,
        pcl::visualization::PointCloudColorHandlerRandom<PointT>(tmp), s.str());
  }

  for(uint l=0; l<100; ++l) {
    MT::wait(1.);
    //viewer.updatePointCloud(kinectData3d.get_point_cloud(NULL), "cluster");
    plist = objectClusters.get_point_clouds(NULL);
    for(int i = 0; i < plist.N; ++i) {
      std::stringstream n;
      //DEBUG_VAR(main, plist(i)->size());
      n << "cl_" << i;
      viewer.updatePointCloud(plist(i), pcl::visualization::PointCloudColorHandlerRandom<PointT>(plist(i)), n.str());
    }
    viewer.removeAllShapes();
    filteredObjects.readAccess(NULL);
    for(int i = 0; i < filteredObjects.objects.N; ++i) {
      std::stringstream name;
      name << "shape_" << i;
      viewer.addCylinder(*(filteredObjects.objects(i)->pcl_object), name.str());
    }
    filteredObjects.deAccess(NULL);
    viewer.spinOnce();
  }
#else
  //pick-and-place loop
  for(uint k=0;k<2;k++){
    pickOrPlaceObject(Action::grasp, "thing2", NULL);
    pickOrPlaceObject(Action::place, "thing2", "thing1");


    pickOrPlaceObject(Action::grasp, "thing1", NULL);
    pickOrPlaceObject(Action::place, "thing1", "thing2");

    //pickOrPlaceObject(Action::grasp, "box2", NULL);
    //pickOrPlaceObject(Action::place, "box2", "cyl2");
    
    //pickOrPlaceObject(Action::grasp, "box1", NULL);
    //pickOrPlaceObject(Action::place, "box1", "table");

    //pickOrPlaceObject(Action::grasp, "box2", NULL);
    //pickOrPlaceObject(Action::place, "box2", "cyl1");
    
    //pickOrPlaceObject(Action::grasp, "box1", NULL);
    //pickOrPlaceObject(Action::place, "box1", "cyl2");

    //pickOrPlaceObject(Action::grasp, "box2", NULL);
    //pickOrPlaceObject(Action::place, "box2", "table");
  }
#endif

  MT::wait(1300);
  //cam.threadClose();
  ctrl->threadClose();
  close(hardware);
  close(P);

  Perception(0)->threadClose();

  //birosInfo().dump();
  cout <<"bye bye" <<endl;
  return 0;
}
