#include <unistd.h>
#include <MT/array.h>
#include <MT/util.h>
#include <MT/calibration.h>
//#include <hardware/hardware.h>
//#include <hardware/kinect.h>
//#include <motion/motion.h>
#include <biros/logging.h>
//#include <motion/FeedbackControlTasks.h>
//#include <pcl/point_types.h>
//#include <pcl/PointIndices.h>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/kdtree/kdtree.h>
//#include <pcl/filters/passthrough.h>
//#include <pcl/filters/extract_indices.h>
//#include <pcl/search/kdtree.h>
//#include <pcl/segmentation/extract_clusters.h>

//SET_LOG(main, DEBUG);

//typedef pcl::PointXYZRGBA PointT;

//bool savePoint = false, selectionFinished = false;
//bool newFrame;
//int finger = 0;
// Y := world coordinates
// X := kinect coordinates
// A := transformation matrix
MT::Array<double> Y, X, A, kinectPt, orsPt;

//int extractMarker(arr& point, pcl::PointCloud<PointT>::Ptr &marked, const pcl::PointCloud<PointT>::Ptr cloud) {
  //arr markerColor = birosInfo.getParameter<arr>("markerColor");
  //int threshold = birosInfo.getParameter<int>("colorThreshold");
  //pcl::PointIndices::Ptr indices(new pcl::PointIndices());
  //for (int i = 0; i < cloud->size(); ++i) {
    //arr pointColor = ARR((*cloud)[i].r, (*cloud)[i].g, (*cloud)[i].b);
    //if(norm(pointColor - markerColor) < threshold) {
      //indices->indices.push_back(i);
    //}
  //}
  //pcl::ExtractIndices<PointT> extract;
  //extract.setInputCloud(cloud);
  //extract.setIndices(indices);
  //extract.setNegative(false);
  //extract.filter(*marked);

  //if (marked->size() == 0) return 0;

  //pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
  //tree->setInputCloud (marked);

  //std::vector<pcl::PointIndices> cluster_indices;
  //pcl::EuclideanClusterExtraction<PointT> ec;
  //ec.setClusterTolerance(0.01);
  //ec.setMinClusterSize(100);
  //ec.setMaxClusterSize(150);
  //ec.setSearchMethod(tree);
  //ec.setInputCloud(marked);
  //ec.extract(cluster_indices);

  //int max = 0;
  //pcl::PointIndices id;

  //// append cluster to PointCloud list 
  //for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it) {
    //if (it->indices.size() > max) {
      //max = it->indices.size();
      //id = *it;
    //}
  //}
  //pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
  //for (std::vector<int>::const_iterator pit = id.indices.begin (); pit != id.indices.end (); pit++)
    //cloud_cluster->points.push_back (marked->points[*pit]); 
  //cloud_cluster->width = cloud_cluster->points.size ();
  //cloud_cluster->height = 1;
  //cloud_cluster->is_dense = true;
  //marked = cloud_cluster;

  //arr mean = ARR(0.,0.,0.);
  //for (int i = 0; i < marked->size(); ++i) {
    //mean += ARR((*marked)[i].x, (*marked)[i].y, (*marked)[i].z);
  //}
  
  //point = mean * (1./marked->size());
  //DEBUG_VAR(main, point);
  //return marked->size();
//}

//void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void)
//{
  //boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = 
    //*static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);

  //if (event.getKeySym () == "q" && event.keyDown ())
  //{
    //std::cout << "q pressed => will now calibrate" << std::endl;
    //selectionFinished = true;
  //}
  
//}


void calcTransformationMatrix() {
  A = inverse(~X*Y * inverse(~Y*Y));
}

int main(int argn, char **argv) {

  //MT::initCmdLine(argn,argv);

  //// variables
  //GeometricState geometricState;
  //MotionPrimitive controllerTask;
  //MotionFuture motionFuture;
  //HardwareReference hardwareReference;
  //SkinPressure skinPressure;
  //JoystickState joystickState;

  //PointCloudVar kinectData3d("KinectData3D");
  //Image kinectDataRGB("KinectDataRGB");

  //// processes
  //Controller controller;
  //Joystick joystick;
  //SchunkArm schunkArm;
  //SchunkHand schunkHand;

  //KinectInterface kinect("Kinect");

  //PoseViewer<HardwareReference> view(hardwareReference);

  //ProcessL hardware=LIST<Process>(schunkArm, schunkHand, joystick);

  //ProcessL P=LIST<Process>(controller, view); //, , schunkSkin, 

  //cout <<"** setting controller to joystick mode" <<endl;
  //Joystick_FeedbackControlTask joyTask;
  //controllerTask.setFeedbackTask(joyTask, true, false, NULL);
  //loopWithBeat(hardware, .01); // hardware must be started before the controller
  //loopWithBeat(P,.01);

	//kinect.threadOpen();
	//kinect.threadLoop();

  //boost::shared_ptr<pcl::visualization::PCLVisualizer> 
    //viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

  //viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&viewer);

  //viewer->setBackgroundColor (255, 255, 255);
  //viewer->addCoordinateSystem (.1);

  //viewer->initCameraParameters ();

  //pcl::PointCloud<PointT>::Ptr cloud = 
    //kinectData3d.get_point_cloud(NULL)->makeShared(); // TODO: perform copy in getter method!

  //// this is just for convenience, if the arm is not in the cloud, set filter max to a higher value
  //pcl::PassThrough<PointT> passthrough;
  //passthrough.setInputCloud(cloud);
  //passthrough.setFilterFieldName("z");
  //passthrough.setFilterLimits(0,1.1);
  //passthrough.filter(*cloud);

  //viewer->addPointCloud(cloud, "cloud");
  //viewer->addPointCloud(cloud, "marked");
  //arr old_ors = ARR(0., 0., 0.);
  //while(!viewer->wasStopped() && !newFrame && !selectionFinished){
    //cloud = 
      //kinectData3d.get_point_cloud(NULL)->makeShared(); // TODO: perform copy in getter method!

    //// this is just for convenience, if the arm is not in the cloud, set filter max to a higher value
    //pcl::PassThrough<PointT> passthrough;
    //passthrough.setInputCloud(cloud);
    //passthrough.setFilterFieldName("z");
    //passthrough.setFilterLimits(0,1.1);
    //passthrough.filter(*cloud);

    //arr kinect_point;
    //pcl::PointCloud<PointT>::Ptr marked(new pcl::PointCloud<PointT>());
    //int num = extractMarker(kinect_point, marked, cloud);

    //geometricState.writeAccess(NULL);
    //geometricState.ors.calcBodyFramesFromJoints();
    //arr ors_point = ARR(geometricState.ors.getShapeByName("marker")->X.pos.p[0],
        //geometricState.ors.getShapeByName("marker")->X.pos.p[1],
        //geometricState.ors.getShapeByName("marker")->X.pos.p[2]);

    //geometricState.deAccess(NULL);
    //if (num > 40 && norm(old_ors - ors_point) > 10e-5) {
      //Y.append(ors_point);
      //Y.resize(Y.N/3, 3);
      //X.append(kinect_point);
      //X.resize(X.N/3, 3);
      //old_ors = ors_point;
    //}


    //viewer->updatePointCloud(marked, pcl::visualization::PointCloudColorHandlerRandom<PointT>(marked), "marked");
    //viewer->updatePointCloud(cloud, "cloud");
    //viewer->spinOnce();

    //if (savePoint) {
    //geometricState.writeAccess(NULL);
    //geometricState.ors.calcBodyFramesFromJoints();
    ////std::cout << geometricState.ors.bodies(1)->inLinks(0)->Q << std::endl;
    ////MT::String shape("tipPoint");
    ////shape << finger + 1;
    ////std::cout << shape  << std::endl;
    //double* orsPos = geometricState.ors.getShapeByName("marker")->X.pos.p;
    ////ors::Vector orsPos = geometricState.ors.getBodyByName("m9")->X.pos;
    ////std::cout << orsPos << std::endl;
    ////ors::Shape *s = geometricState.ors.getShapeByName("marker");
    ////geometricState.ors.kinematics(orsPt,s->body->index,&s->rel.pos);
    //geometricState.deAccess(NULL);
    ////std::cout << *orsPos << *(orsPos+1) << *(orsPos+2);
    //orsPt = ARR( *orsPos, *(orsPos+1), *(orsPos+2));
    //std::cout << "Corresponding ors point is " << orsPt << std::endl;
    //Y.append(orsPt);
    //Y.resize(Y.N/3, 3);
    //X.append(kinectPt);
    //X.resize(X.N/3, 3);
    ////std::cout << "Y dim = " << Y.nd << "\t X dim = " << X.nd << std::endl;
    ////std::cout << X << std::endl;
    ////std::cout << Y << std::endl;
    //savePoint = false;
    //finger = (finger+1)%3;
    //}
  //}

  //newFrame = false;
  //viewer->removePointCloud();

  //save points
  //std::ifstream kinect_points("kt.points");
  //std::ifstream world_points("wd.points");
  //kinect_points << X << std::endl;
  //world_points << Y << std::endl;
  //
  MT::load(X, "kt.points");
  MT::load(Y, "wd.points");

  X = ~X;
  Y = ~Y;
  X.append(ones(X.d1, 1));
  Y.append(ones(Y.d1, 1));
  X = ~X;
  Y = ~Y;


  //calcTransformationMatrix();
  arr K, R, t;
  //decomposeCameraCalibMatrix(K, R, t, A);
  std::cout << A << std::endl;
  
  //viewer.reset();
  //close(P);
  return 0;
}
