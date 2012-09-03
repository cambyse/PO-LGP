#include <unistd.h>
#include <MT/array.h>
#include <MT/util.h>
#include <MT/calibration.h>
#include <MT/vision.h>
#include <hardware/hardware.h>
#include <hardware/kinect.h>
#include <motion/motion.h>
#include <devTools/logging.h>
#include <biros/biros.h>
#include <biros/control.h>
#include <perception/pointcloud.h>
#include <motion/FeedbackControlTasks.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

SET_LOG(main, DEBUG);

typedef pcl::PointXYZRGBA PointT;

bool savePoint = false, selectionFinished = false;
bool newFrame;
// Y := world coordinates
// X := kinect coordinates
// A := transformation matrix
MT::Array<double> Y, X, A, kinectPt, orsPt;

int extractMarker(arr& point, pcl::PointCloud<PointT>::Ptr &marked, pcl::PointCloud<PointT>::Ptr cloud) {
  floatA markerColor = birosInfo().getParameter<floatA>("markerColor");


  float threshold = birosInfo().getParameter<float>("colorThreshold");
  pcl::PointIndices::Ptr indices(new pcl::PointIndices());
  for (int i = 0; i < cloud->size(); ++i) {
    floatA rgb;
    rgb.append((*cloud)[i].r / 256.);
    rgb.append((*cloud)[i].g / 256.);
    rgb.append((*cloud)[i].b / 256.);
    rgb.reshape(1,3);
    floatA hsv;
    rgb2hsv(hsv, rgb);

    floatA tol; tol.append(.1); tol.append(1); tol.append(1);

    hsv.reshape(3);

    //DEBUG_VAR(main, hsv_diff(hsv, marker, tol));

    if(hsv_diff(hsv, markerColor, tol) < threshold ) {
      (*cloud)[i].r = 255;
      (*cloud)[i].g = 0;
      (*cloud)[i].b = 0;

      indices->indices.push_back(i);
    }
  }
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(indices);
  extract.setNegative(false);
  extract.filter(*marked);


  if (marked->size() == 0) return 0;

  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
  tree->setInputCloud (marked);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance(0.01);
  ec.setMinClusterSize(100);
  ec.setMaxClusterSize(1500);
  ec.setSearchMethod(tree);
  ec.setInputCloud(marked);
  ec.extract(cluster_indices);

  int max = 0;
  pcl::PointIndices id;

  // append cluster to PointCloud list 
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it) {
    if (it->indices.size() > max) {
      max = it->indices.size();
      id = *it;
    }
  }
  pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
  for (std::vector<int>::const_iterator pit = id.indices.begin (); pit != id.indices.end (); pit++)
    cloud_cluster->points.push_back (marked->points[*pit]); 
  cloud_cluster->width = cloud_cluster->points.size ();
  cloud_cluster->height = 1;
  cloud_cluster->is_dense = true;
  marked = cloud_cluster;


  arr mean = ARR(0.,0.,0.);
  for (int i = 0; i < marked->size(); ++i) {
    mean += ARR((*marked)[i].x, (*marked)[i].y, (*marked)[i].z);
  }
  
  point = mean * (1./marked->size());

  //DEBUG_VAR(main, point);

  return marked->size();
}



void calcTransformationMatrix() {
  A =inverse(~X*Y * inverse(~Y*Y));
}

void computeMode();
void extractMode();


int main(int argn, char **argv) {

  MT::initCmdLine(argn,argv);

  if(birosInfo().getParameter<int>("mode", NULL) == 1) {
    extractMode();  
  }
  else {
    computeMode();  
  }
}

void extractMode() {
  // variables
  GeometricState geometricState;
  MotionPrimitive controllerTask;
  HardwareReference hardwareReference;
  SkinPressure skinPressure;
  JoystickState joystickState;

  PointCloudVar kinectData3d("KinectData3D");
  //Image kinectDataRGB("KinectDataRGB");

  //// processes
  Process* controller = newMotionController(&hardwareReference, &controllerTask, NULL);

  Joystick joystick;
  SchunkArm schunkArm;
  SchunkHand schunkHand;
  SchunkSkin schunkSkin;

  KinectInterface kinect("Kinect");
  //setup kinect callback; don't need to loop
  kinect.threadOpen();


  PointCloudSet objectClusters("ObjectClusters");
  ObjectSet objects("Objects");
  ObjectBeliefSet filteredObjects("filteredObjects");
  Workspace<FittingJob, FittingResult> fittingWorkspace("FittingWorkspace");
  
  ObjectFitter fitter;
  ObjectFitterWorker worker;
  ObjectFilter filter("Object Filter");

  ProcessL hardware = LIST<Process>(schunkArm, schunkHand, schunkSkin, joystick);
  ProcessL P; P.append(controller); //, view); //, , schunkSkin, 

  new PoseView(hardwareReference.fields(0), NULL);

  cout <<"** setting controller to joystick mode" <<endl;
  Joystick_FeedbackControlTask joyTask;
  controllerTask.setFeedbackTask(joyTask, true, false, NULL);

  loopWithBeat(hardware, .01); // hardware must be started before the controller
  loopWithBeat(P,.01);
  //b::openInsideOut();
  boost::shared_ptr<pcl::visualization::PCLVisualizer> 
    viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

  //viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&viewer);

  viewer->setBackgroundColor (255, 255, 255);
  viewer->addCoordinateSystem (.1);

  viewer->initCameraParameters ();

  pcl::PointCloud<PointT>::Ptr cloud = 
    kinectData3d.get_point_cloud(NULL)->makeShared(); // TODO: perform copy in getter method!

  //// this is just for convenience, if the arm is not in the cloud, set filter max to a higher value
  pcl::PassThrough<PointT> passthrough;
  passthrough.setInputCloud(cloud);
  passthrough.setFilterFieldName("z");
  passthrough.setFilterLimits(0,1.1);
  passthrough.filter(*cloud);

  //viewer->addPointCloud(cloud, "marked");
  viewer->addPointCloud(cloud, "cloud");
  viewer->addSphere(PointT(), 0.02, "sphere");
  arr last_saved_ors_point = ARR(0., 0., 0.);
  arr last_saved_kinect_point = ARR(0., 0., 0.);
  arr old_point = ARR(0., 0., 0.);
  while(!viewer->wasStopped() && !newFrame && !selectionFinished){
    cloud = 
      kinectData3d.get_point_cloud(NULL)->makeShared(); // TODO: perform copy in getter method!

    //// this is just for convenience, if the arm is not in the cloud, set filter max to a higher value
    pcl::PassThrough<PointT> passthrough;
    passthrough.setInputCloud(cloud);
    passthrough.setFilterFieldName("z");
    passthrough.setFilterLimits(0,1.1);
    passthrough.filter(*cloud);
    

    arr kinect_point;
    pcl::PointCloud<PointT>::Ptr marked(new pcl::PointCloud<PointT>());
    int num = extractMarker(kinect_point, marked, cloud);

    objectClusters.writeAccess(NULL);
    objectClusters.point_clouds.clear();
    if(marked && marked->size() > 0) {
      objectClusters.point_clouds.append(marked);
    }
    objectClusters.deAccess(NULL);
    
    MT::wait(0.5);

    geometricState.writeAccess(NULL);
    geometricState.ors.calcBodyFramesFromJoints();
    arr ors_point = ARR(geometricState.ors.getShapeByName("marker")->X.pos.p[0],
        geometricState.ors.getShapeByName("marker")->X.pos.p[1],
        geometricState.ors.getShapeByName("marker")->X.pos.p[2]);
    geometricState.deAccess(NULL);

    if (norm(old_point - ors_point) < 10e-3 &&
        norm(last_saved_ors_point - ors_point) > 10e-5) {
      filteredObjects.readAccess(NULL);
      if(filteredObjects.objects.N > 0 && 
          norm(last_saved_kinect_point - ARRAY(filteredObjects.objects(0)->position)) > 10e-5) {
        Y.append(ors_point);
        Y.resize(Y.N/3, 3);
        X.append(filteredObjects.objects(0)->position(0));
        X.append(filteredObjects.objects(0)->position(1));
        X.append(filteredObjects.objects(0)->position(2));
        X.resize(X.N/3, 3);
        cout << ors_point << "---" << filteredObjects.objects(0)->position <<endl;
        last_saved_ors_point = ors_point;
        last_saved_kinect_point = ARRAY(filteredObjects.objects(0)->position);

        arr array_pos;
        array_pos = ARRAY(filteredObjects.objects(0)->position);
        arr kinect_transformation = birosInfo().getParameter<arr>("kinect_trans_mat", NULL);
        array_pos.append(1);
        array_pos = kinect_transformation*array_pos;

        viewer->removeAllShapes();
        viewer->addSphere(*filteredObjects.objects(0)->pcl_object, "sphere");

        geometricState.writeAccess(NULL);
        ors::Shape *ball = geometricState.ors.getShapeByName("target1");
        ball->X.pos.set(array_pos.p);
        ball->rel.setDifference(ball->body->X, ball->X);
        geometricState.deAccess(NULL);
      }
      filteredObjects.deAccess(NULL);
    }
    old_point = ors_point;
    
    viewer->updatePointCloud(cloud, "cloud");
    //viewer->updatePointCloud(marked, pcl::visualization::PointCloudColorHandlerRandom<PointT>(marked), "marked");
    viewer->spinOnce();
  }

  newFrame = false;
  viewer->removePointCloud();

  //save points
  std::ofstream kinect_points("kt.points");
  std::ofstream world_points("wd.points");
  kinect_points << X << std::endl;
  world_points << Y << std::endl;
  //
}
void computeMode() {
  MT::load(X, "kt.points");
  MT::load(Y, "wd.points");

  int xd0 = X.d0;
  int yd0 = Y.d0;

  X = ~X;
  Y = ~Y;
  X.append(ones(X.d1,1));
  Y.append(ones(Y.d1,1));
  X.reshape(4, xd0);
  Y.reshape(4, yd0);
  X = ~X;
  Y = ~Y;

  calcTransformationMatrix();
  arr K, R, t;
  decomposeCameraProjectionMatrix(K, R, t, A, true);
  
  //viewer.reset();
  //close(P);
}
