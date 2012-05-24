#include <perception/pointcloud.h>
#include <JK/utils/util.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <queue>

int main(int argc, char **argv) {

  // Variables
  PointCloudVar kinectData3d("KinectData3D");
  PointCloudSet objectClusters("ObjectClusters");
  ObjectSet objects("Objects");

  // Processes
  KinectInterface kinect("Kinect");
  ObjectClusterer clusterer;
  ObjectFitterIntegrator integrator;

  ObjectFitterWorkerFactory factory;
  ObjectFitter fitter(&factory, &integrator, 10); //no process but a master!

  kinect.threadLoop();
  clusterer.threadLoop();

  MT::wait(2.);


  // TODO: put into Viewer<PointCloudVar>
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addCoordinateSystem (.1);
  viewer->initCameraParameters ();
  //viewer->addPointCloud(kinectData3d.get_point_cloud_copy(NULL), "cloud");
  while(objectClusters.get_point_clouds(NULL).N == 0) MT::wait(0.1);
  
  viewer->addPointCloud(kinectData3d.get_point_cloud(NULL), "cluster");

  PointCloudL plist = objectClusters.get_point_clouds(NULL);

  for(int i = 0; i < 1000; ++i) {
    //if(i % 50 == 0 ) {
      objectClusters.readAccess(NULL);
      plist = objectClusters.point_clouds;
      std::queue<FittingJob> jobs;
      for(int i = 0; i < plist.N; ++i) {
        jobs.push(plist(i));
      }
      fitter.pause();
      fitter.restart(jobs, NULL);
      objectClusters.deAccess(NULL);
    //}
    MT::wait(.1);
    for(int i = 0; i < objects.get_objects(NULL).N; ++i) {
      std::stringstream name;
      name << "cyl_" << i;
      viewer->removeShape(name.str());  
      viewer->addCylinder(*objects.get_objects(NULL)(i), name.str());
    }
    viewer->updatePointCloud(kinectData3d.get_point_cloud(NULL), "cluster");
    viewer->spinOnce();
  }

  kinect.threadStop();

}
