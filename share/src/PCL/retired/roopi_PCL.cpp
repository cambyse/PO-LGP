#include "roopi_PCL.h"

#include <Perception/kinect2pointCloud.h>
#include <Gui/viewer.h>
#include <PCL/pclViewer.h>
#include <PCL/pipeline.h>

ThreadL newPclPipeline(bool view){
  ThreadL threads;
  threads.append(new Kinect2PointCloud());
  threads.append(new Conv_arr_pcl("pclRawInput", "kinect_points", "kinect_rgb"));
  threads.append(new PclPipeline("pclRawInput"));
  if(view) threads.append(new PointCloudViewer());
//  if(view) threads.append(new PclViewer("pclRawInput"));
  return threads;
}

