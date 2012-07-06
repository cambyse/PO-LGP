#include "pointcloud.h"
#include "perception.h"

#include <numeric>
#include <limits>

#include <biros/biros.h>
#include <biros/logging.h>
#include <MT/array.h>

#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <vtkSmartPointer.h>
#include <vtkDataSet.h>
#include <vtkLineSource.h>
#include <vtkTubeFilter.h>

ObjectClusterer::ObjectClusterer() : Process("ObjectClusterer") {
  birosInfo.getVariable(data_3d, "KinectData3D", this, true);
  birosInfo.getVariable(point_clouds, "ObjectClusters", this, true);
}

void findMinMaxOfCylinder(double &min, double &max, arr &start, const pcl::PointCloud<PointT>::Ptr &cloud, const arr &direction) {
  arr dir = direction/norm(direction);
  min = std::numeric_limits<double>::max();
  max = -std::numeric_limits<double>::max();
  for(int i=0; i<cloud->size(); ++i) {
    arr point = ARR((*cloud)[i].x, (*cloud)[i].y, (*cloud)[i].z);
    double p = scalarProduct(dir, point);
    if(p < min) {
      min = p;
      copy(start, point);
    }
    if(p > max) max = p;
  }
}

struct sObjectFitterWorker {
  sObjectFitterWorker(ObjectFitterWorker *p) : p(p) {}
  ObjectFitterWorker *p;

  void createNewJob(const pcl::PointCloud<PointT>::Ptr &cloud, const pcl::PointIndices::Ptr &inliers){ 
    FittingJob outliers(new pcl::PointCloud<PointT>());
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*outliers);
    p->jobs->writeAccess(p);
    p->jobs->data.push(outliers);
    p->jobs->deAccess(p);
  }
  
  double confidenceForCylinder(pcl::PointIndices::Ptr &inliers, FittingResult& object, const pcl::PointCloud<PointT>::Ptr &cloud, const pcl::PointCloud<pcl::Normal>::Ptr &normals) {
    // Create the segmentation object for cylinder segmentation and set all the parameters
    // TODO: make parameters
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; 
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_CYLINDER);
    seg.setMethodType (pcl::SAC_RANSAC);
    double ndw = birosInfo.getParameter<double>("CylNormalDistanceWeight", p, 0.07);
    seg.setNormalDistanceWeight (ndw);
    seg.setMaxIterations (100);
    double dt = birosInfo.getParameter<double>("CylDistanceThreshold", p, 0.01);
    seg.setDistanceThreshold (dt);
    seg.setRadiusLimits (0.01, 0.1);
    seg.setInputCloud (cloud);
    seg.setInputNormals (normals);

    pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);
    seg.segment (*inliers_cylinder, *coefficients_cylinder);

    int minCloudSize = birosInfo.getParameter<int>("minCloudSize", p, 500);
    if (inliers_cylinder->indices.size() < minCloudSize) {
      object.reset();   
      return 0;
    }
    else {
      object = coefficients_cylinder;
      inliers = inliers_cylinder;
      return 1./(1 + cloud->size() - inliers->indices.size());
    }
  }
  
  double confidenceForSphere(pcl::PointIndices::Ptr &inliers, FittingResult& object, const pcl::PointCloud<PointT>::Ptr &cloud, const pcl::PointCloud<pcl::Normal>::Ptr &normals) {
    // Create the segmentation object for sphere segmentation and set all the parameters
    // TODO: make parameters
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; 
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_SPHERE);
    seg.setMethodType (pcl::SAC_RANSAC);
    double ndw = birosInfo.getParameter<double>("SphereNormalDistanceWeight", p, 10);
    seg.setNormalDistanceWeight (ndw);
    seg.setMaxIterations (100);
    double dt = birosInfo.getParameter<double>("SphereDistanceThreshold", p, .0005);
    seg.setDistanceThreshold (dt);
    seg.setRadiusLimits (0.01, 0.1);
    seg.setInputCloud (cloud);
    seg.setInputNormals (normals);

    pcl::PointIndices::Ptr inliers_sphere(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients_sphere(new pcl::ModelCoefficients);
    seg.segment (*inliers_sphere, *coefficients_sphere);
    int minCloudSize = birosInfo.getParameter<int>("minCloudSize", p, 500);
    if (inliers_sphere->indices.size() < minCloudSize) {
      object.reset();   
      return 0;
    }
    else {
      object = coefficients_sphere;
      inliers = inliers_sphere;
     
      // if rest points are enough create new job
      //if (cloud->size() - inliers_sphere->indices.size() > 500) {
        //createNewJob(cloud, inliers_sphere);
      //}
      return 1./(1 + cloud->size() - inliers->indices.size());
    }
  }
};

void ObjectClusterer::open() {}

void ObjectClusterer::close() {}

void ObjectClusterer::step() {
  //get a copy of the kinect data
  pcl::PointCloud<PointT>::Ptr cloud(data_3d->get_point_cloud_copy(this));
  if(cloud->points.size() == 0) return;

  // filter all points too far away
  // TODO: filter also points too far left/right/up/down
  pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>());
  pcl::PassThrough<PointT> passthrough;
  passthrough.setInputCloud(cloud);
  passthrough.setFilterFieldName("z");
  passthrough.setFilterLimits(0,1.5);
  passthrough.filter(*cloud_filtered);
  passthrough.setInputCloud(cloud_filtered);
  passthrough.setFilterFieldName("x");
  passthrough.setFilterLimits(-0.4,0.25);
  passthrough.filter(*cloud_filtered);
 
  // filter away the table. This is done by fitting a plane to all data and
  // remove all inliers. This assumes that there is one big plane, which
  // contains all other objects.
  pcl::IndicesPtr inliers(new std::vector<int>);
  pcl::SampleConsensusModelPlane<PointT>::Ptr planemodel 
    (new pcl::SampleConsensusModelPlane<PointT> (cloud_filtered));
  pcl::RandomSampleConsensus<PointT> ransac(planemodel);
  ransac.setDistanceThreshold(0.01);
  ransac.computeModel();
  ransac.getInliers(*inliers);

  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloud_filtered);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*cloud_filtered);

  if (cloud_filtered->points.size() == 0) return;

  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
  tree->setInputCloud (cloud_filtered);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance(0.01);
  int minCloudSize = birosInfo.getParameter<int>("minCloudSize", this, 500);
  ec.setMinClusterSize(minCloudSize);
  ec.setMaxClusterSize(25000);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud_filtered);
  ec.extract(cluster_indices);

  PointCloudL _point_clouds;
  // append cluster to PointCloud list 
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it) {
    pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
      cloud_cluster->points.push_back (cloud_filtered->points[*pit]); 
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    _point_clouds.append(cloud_cluster);
  }

  point_clouds->set_point_clouds(_point_clouds, this);
}

void ObjectFitterIntegrator::restart() {
  objects->writeAccess(this);
  objects->objects.clear();
  objects->deAccess(this);
}

void ObjectFitterIntegrator::integrateResult(const FittingResult &result) {
  // add to ObjectSet  
  if (result.get() == 0) return;
  objects->writeAccess(this);
  objects->objects.append(result);
  objects->deAccess(this);
}

ObjectFitterWorker::ObjectFitterWorker() : Worker<FittingJob, FittingResult>("ObjectFitter (Worker)"), s(new sObjectFitterWorker(this)) {}

void ObjectFitterWorker::doWork(FittingResult &object, const FittingJob &cloud) {
  // Build kd-tree from cloud
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
  tree->setInputCloud (cloud);
  
  // Estimate point normals
  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  ne.setSearchMethod (tree);
  ne.setInputCloud (cloud);
  ne.setKSearch (50);
  
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  ne.compute (*cloud_normals);

  FittingResult cyl_object;
  pcl::PointIndices::Ptr cyl_inliers;
  double cylinder_confidence = s->confidenceForCylinder(cyl_inliers, cyl_object, cloud, cloud_normals);
  FittingResult sphere_object;
  pcl::PointIndices::Ptr sphere_inliers;
  double sphere_confidence = s->confidenceForSphere(sphere_inliers, sphere_object, cloud, cloud_normals);

  double threshold = 0.;

  pcl::PointIndices::Ptr inliers;
  if (sphere_confidence > threshold && sphere_confidence > cylinder_confidence) {
     object = sphere_object; 
     inliers = sphere_inliers;
  }
  else if (cylinder_confidence > threshold) {
    object = cyl_object;  
    inliers = cyl_inliers;
    double min, max;
    arr direction = ARR(object->values[3], object->values[4], object->values[5]);
    pcl::PointCloud<PointT>::Ptr cylinder(new pcl::PointCloud<PointT>());
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cylinder);
    arr start;
    findMinMaxOfCylinder(min, max, start, cylinder, direction);
    arr s = ARR(object->values[0], object->values[1], object->values[2]);
    direction = direction/norm(direction);
    arr st = s+scalarProduct(direction, (start-s))*direction;
    direction = (max - min) * direction;
    object->values[0] = st(0);
    object->values[1] = st(1);
    object->values[2] = st(2);
    object->values[3] = direction(0);
    object->values[4] = direction(1);
    object->values[5] = direction(2);
  }
  else {
    object.reset();  
    return;
  }
  //if rest points are enough create new job
 
  int minCloudSize = birosInfo.getParameter<int>("minCloudSize", this, 500);
  if (cloud->size() - inliers->indices.size() > minCloudSize) {
    s->createNewJob(cloud, inliers);
  }
}
struct sObjectFilter {
  bool filterShape(arr& pos, intA& nums, const arr& measurement, const int i, const double epsilon ) {
    if (norm(measurement - pos[i]) < epsilon)  {
      pos[i] = pos[i] * ((nums(i)-1.)/nums(i)) + measurement * (1./nums(i));
      nums(i)++;
      return true;
    }
    return false;
  }

  void filterCylinders(arr& pos, const FittingResultL& objects, Process *p) {
    intA nums;
    pos.clear();
    nums.resize(0);
    pos.resize(0,7);
    for (int i = 0; i< objects.N; ++i) {
      if (objects(i)->values.size() != 7) continue;
      bool found = false;
      arr measurement;
      measurement.resize(1,7);
      std::copy(objects(i)->values.begin(), objects(i)->values.end(), measurement.p);
      arr measurement_;
      measurement_.append(measurement.sub(0,0,0,2)+measurement.sub(0,0,3,5));
      measurement_.append(-measurement.sub(0,0,3,5));
      measurement_.append(measurement(0,6));
      measurement_.resize(1,7);
      double epsilon = birosInfo.getParameter<double>("objectDistance", p);
      for (int j = 0; j<pos.d0; ++j) {
        if(filterShape(pos, nums, measurement, j, epsilon)) { found = true; break;}
        else if (filterShape(pos, nums, measurement_, j, epsilon)) {found = true; break; }
      }
      if(!found) {
        pos.append(measurement);
        nums.append(1);
      }
    }

  }
  void filterSpheres(arr& pos, const FittingResultL& objects, Process* p) {
    intA nums;
    pos.clear();
    nums.resize(0);
    pos.resize(0,4);
    for (int i = 0; i< objects.N; ++i) {
      if (objects(i)->values.size() != 4) continue;
      bool found = false;
      arr measurement;
      measurement.resize(1,4);
      std::copy(objects(i)->values.begin(), objects(i)->values.end(), measurement.p);
      double epsilon = birosInfo.getParameter<double>("objectDistance", p);
      for (int j = 0; j<pos.d0; ++j) {
        if(filterShape(pos, nums, measurement, j, epsilon)) { found = true; break; }
      }
      if(!found) {
        pos.append(measurement);
        nums.append(1);
      }
    }

  }
};

ObjectFilter::ObjectFilter(const char* name) : Process(name) {
  s = new sObjectFilter;  
}

void ObjectFilter::open() {
  birosInfo.getVariable(in_objects, "Objects", this, true);
  birosInfo.getVariable(out_objects, "filteredObjects", this, true);
}

void ObjectFilter::step() {
  arr cyl_pos, sph_pos;
  cyl_pos.resize(0,7);
  sph_pos.resize(0,7);
  in_objects->readAccess(this);
  s->filterCylinders( cyl_pos,in_objects->objects, this);
  s->filterSpheres( sph_pos,in_objects->objects, this);
  in_objects->deAccess(this);
  out_objects->writeAccess(this);
  out_objects->objects.clear();
  for (int i = 0; i<cyl_pos.d0; i++) {
    ObjectBelief *cyl = new ObjectBelief();
    cyl->position = ARR(cyl_pos(i,0), cyl_pos(i,1), cyl_pos(i,2));
    cyl->rotation.setDiff(ARR(0,0,1), ARR(cyl_pos(i,3), cyl_pos(i,4), cyl_pos(i,5)));
    cyl->shapeParams(RADIUS) = cyl_pos(i,6);//.025;
    cyl->shapeParams(HEIGHT) = norm(ARR(cyl_pos(i,3), cyl_pos(i,4), cyl_pos(i,5)));
    cyl->shapeType = ors::cylinderST;
    out_objects->objects.append(cyl);
  }
  for (int i = 0; i<sph_pos.d0; i++) {
    ObjectBelief *sph = new ObjectBelief;
    sph->position = ARR(sph_pos(i,0), sph_pos(i,1), sph_pos(i,2));
    sph->shapeParams(RADIUS) = sph_pos(i,3);
    sph->shapeType = ors::sphereST;
    out_objects->objects.append(sph);
  }
  out_objects->deAccess(this);
}

void ObjectTransformator::open() {
  birosInfo.getVariable(kinect_objects, "filteredObjects", this, true);
  geo.init("GeometricState", this);
}

void createCylinder(ors::Body& cyl, const arr &pos, const arr &direction, const double radius) {
  ors::Transformation t;
  arr dir = direction.sub(0,2)*(1./direction(3));

  for (uint i = 0; i < 3; ++i) t.pos(i) = pos(i) + 0.5*dir(i);
  
  t.rot.setDiff(ARR(0,0,1), dir);
  
  arr size = ARR(0.1, 1, norm(dir), radius);
 
  ors::Shape* s = new ors::Shape();
  for (uint i = 0; i < 4; ++i) s->size[i] = size(i);
  for (uint i = 0; i < 3; ++i) s->color[i] = .3;
  s->type = ors::cylinderST;
  s->body = &cyl;
  s->name = "pointcloud_shape";
  
  cyl.shapes.append(s);
  cyl.X = t; 
}

void createSphere(ors::Body& cyl, const arr &pos, const double radius) {
  ors::Transformation t;

  for (uint i = 0; i < 3; ++i) t.pos(i) = pos(i);
  
  arr size = ARR(0.1, 1, 1, radius);
 
  ors::Shape* s = new ors::Shape();
  for (uint i = 0; i < 4; ++i) s->size[i] = size(i);
  for (uint i = 0; i < 3; ++i) s->color[i] = .3;
  s->type = ors::sphereST;
  s->body = &cyl;
  s->name = "pointcloud_shape";
  
  cyl.shapes.append(s);
  cyl.X = t; 
    
}

void ObjectTransformator::step() {
  arr transformation = birosInfo.getParameter<arr>("kinect_trans_mat", this);
  geo.pull();
  // remove all shape_num objects
  for (int i=geo().ors.bodies.N-1;i>=0;i--) {
    if (strncmp(geo().ors.bodies(i)->name, "pointcloud_object", 16) == 0) {
      ors::Body *b = geo().ors.bodies(i);
      geo().ors.bodies.remove(i);  
      delete b;
    }
  }
  for (int i=geo().ors.shapes.N-1;i>=0;i--) {
    if (strncmp(geo().ors.shapes(i)->name, "pointcloud_shapes", 16) == 0) {
      ors::Shape *s = geo().ors.shapes(i);
      geo().ors.shapes.remove(i);  
      delete s;
    }
  }
  kinect_objects->readAccess(this);
  for(int i=0;i<kinect_objects->objects.N;i++){
    //add new body  
    MT::String name;
    name << "pointcloud_object" << i;
    ors::Body *b = new ors::Body();
    if(kinect_objects->objects(i)->values.size() == 7) {
      arr pos;
      arr direction;
      pos.resize(4);
      direction.resize(4);
      
      pos(0) = kinect_objects->objects(i)->values[0];
      pos(1) = kinect_objects->objects(i)->values[1];
      pos(2) = kinect_objects->objects(i)->values[2];
      pos(3) = 1;

      direction(0) = kinect_objects->objects(i)->values[3];
      direction(1) = kinect_objects->objects(i)->values[4];
      direction(2) = kinect_objects->objects(i)->values[5];
      direction(3) = 1;

      direction = transformation * direction;
      pos = transformation * pos;

      arr rad = ARR(kinect_objects->objects(i)->values[6], 0, 0, 1);
      rad = (transformation * rad); 
      rad = rad * (1./rad(3));
      double radius = norm(rad.sub(0,2));

      createCylinder(*b, pos, direction, radius);  
      b->name = name.p;
      geo().ors.bodies.append(b);
      int ibody = geo().ors.bodies.N - 1;
      int i; ors::Shape *s;
      for_list(i, s, b->shapes) {
        s->ibody = ibody;
        s->index = geo().ors.shapes.N;
        geo().ors.shapes.append(s);
      }
    }
    else if(kinect_objects->objects(i)->values.size() == 4) {
      arr pos;
      pos.resize(4);

      pos(0) = kinect_objects->objects(i)->values[0];
      pos(1) = kinect_objects->objects(i)->values[1];
      pos(2) = kinect_objects->objects(i)->values[2];
      pos(3) = 1;

      arr rad = ARR(kinect_objects->objects(i)->values[3], 0, 0, 1);
      rad = (transformation * rad); 
      rad = rad * (1./rad(3));
      double radius = norm(rad.sub(0,2));

      createSphere(*b, pos, radius);  
      b->name = name.p;
      geo().ors.bodies.append(b);
      int ibody = geo().ors.bodies.N - 1;
      int i; ors::Shape *s;
      for_list(i, s, b->shapes) {
        s->ibody = ibody;
        s->index = geo().ors.shapes.N;
        geo().ors.shapes.append(s);
      }
    }
  }
  kinect_objects->deAccess(this);
  geo().ors.calcBodyFramesFromJoints();
  geo.push();
}
