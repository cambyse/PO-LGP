#include "system.h"
#include "methods.h"
#include <pcl/visualization/pcl_visualizer.h>

#include "plane.h"
#include "dataStructures.h"

void TEST(KinectModules) {

//  DisplayPrimitives primitives;
//  OpenGL gl;
//  gl.camera = kinectCam;
//  gl.add(glStandardScene, NULL);
//  gl.add(glDrawPrimitives, &primitives);
//  primitives.P.append(new Plane(1,1,1,2.));
//  ors::Shape *s = new ors::Shape(primitives.G, NoBody)->type=ors::boxST;
//  gl.update();

  PCL_ModuleSystem S;

  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>(640,480));
  pcl::visualization::PCLVisualizer viewer("3D Viewer");
  viewer.addPointCloud<PointT>(cloud, "cloud");

//  CloudView *cv = new CloudView(S.pcl_cloud.get());
//  primitives.P.append(cv);

  engine().open(S);

  for(;;){
    if(engine().shutdown.getValue()>0) break;
    S.kinect_points.var->waitForNextRevision();

    cloud = S.pcl_cloud.get();

    if(cloud){
      viewer.updatePointCloud(cloud, "cloud");
      viewer.spinOnce();
    }

//    cv->cloud = S.pcl_cloud.get();
//    gl.update();

  }

  engine().close(S);
  cout <<"bye bye" <<endl;
}


int main(int argc,char **argv){
  testKinectModules();

  return 0;
};

