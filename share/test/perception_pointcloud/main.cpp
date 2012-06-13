#include <perception/pointcloud.h>
#include <JK/utils/util.h>
#include <JK/particle_filter/particle_filter.h>
#include <MT/gauss.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <queue>

Gaussian g;
double gaussian_weight(const arr& particle, const arr &measurement) {
   g.c = measurement;
   g.C = eye(measurement.N) *.00001;
   return g.evaluate(particle);
}

void gaussian_control(arr &after, const arr &before) {
  after = before + 0.0005 * randn(before.d0, 1);  
}

int main(int argc, char **argv) {
  MT::initCmdLine(argc,argv);

  srand(time(NULL));

  // Variables
  PointCloudVar kinectData3d("KinectData3D");
  PointCloudSet objectClusters("ObjectClusters");
  ObjectSet objects("Objects");
  ObjectSet filteredObjects("filteredObjects");

  // Processes
  KinectInterface kinect("Kinect");
  ObjectClusterer clusterer;
  ObjectFitterIntegrator integrator;

  ObjectFitterWorkerFactory factory;
  ObjectFitter fitter(&factory, &integrator, 10); //no process but a master!

  ObjectFilter filter("Object Filter");

  kinect.threadLoop();
  clusterer.threadLoop();

  // TODO: put into Viewer<PointCloudVar>
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addCoordinateSystem (.1);
  viewer->initCameraParameters ();
  //viewer->addPointCloud(kinectData3d.get_point_cloud_copy(NULL), "cloud");
  while(objectClusters.get_point_clouds(NULL).N == 0) MT::wait(0.1);
  
  viewer->addPointCloud(kinectData3d.get_point_cloud(NULL), "cluster");

  PointCloudL plist = objectClusters.get_point_clouds(NULL);

  for(int j = 0; j<20; j++) {
    std::stringstream s;
    s << "cl_" << j;
    pcl::PointCloud<PointT>::Ptr tmp(new pcl::PointCloud<PointT>());
    viewer->addPointCloud(tmp,
        pcl::visualization::PointCloudColorHandlerRandom<PointT>(tmp), s.str());
  }

  filter.threadLoop();
  for(int t = 0; t < 1000; ++t) {
    if(t % 5 == 0 ) {
      objectClusters.readAccess(NULL);
      plist = objectClusters.point_clouds;
      std::queue<FittingJob> jobs;
      for(int i = 0; i < plist.N; ++i) {
        std::stringstream n;
        n << "cl_" << i;
        jobs.push(plist(i));
        viewer->updatePointCloud(plist(i), pcl::visualization::PointCloudColorHandlerRandom<PointT>(plist(i)), n.str());
      }
      std::cout << plist.N << std::endl;
      fitter.pause();
      viewer->removeAllShapes();
      filteredObjects.readAccess(NULL);
      for(int i = 0; i < filteredObjects.objects.N; ++i) {
        std::stringstream name;
        name << "shape_" << i;
        viewer->addCylinder(*filteredObjects.objects(i), name.str());
      }
      filteredObjects.deAccess(NULL);
      fitter.restart(jobs, NULL);
      objectClusters.deAccess(NULL);
    }
    MT::wait(.1);
    viewer->updatePointCloud(kinectData3d.get_point_cloud(NULL), "cluster");
    viewer->spinOnce();
  }

  kinect.threadStop();

}
