#include "act_Perception.h"

#include <Perception/filter.h>
#include <Perception/percViewer.h>
#include <Perception/kinect2pointCloud.h>
#include <Gui/viewer.h>
#include <PCL/pclViewer.h>
#include <PCL/pipeline.h>

Act_PerceptionFilter::Act_PerceptionFilter(Roopi* r, bool view)
  : Act(r){
  filter = ptr<Act_Filter>(new Act_Filter(r, new Filter));
  if(view) viewerInput = ptr<Act_PercViewer>(new Act_PercViewer(r, new PercViewer("percepts_input")));
  if(view) viewerFiltered = ptr<Act_PercViewer>(new Act_PercViewer(r, new PercViewer("percepts_filtered")));
}


Act_PclPipeline::Act_PclPipeline(Roopi* r, bool view)
  : Act(r){
  kin2cloud = ptr<Kinect2PointCloud>(new Kinect2PointCloud);
  cloud2pcl = ptr<Conv_arr_pcl>(new Conv_arr_pcl("pclRawInput", "kinect_points", "kinect_rgb"));
  pclPipeline = ptr<PclPipeline>(new PclPipeline("pclRawInput"));
//  if(view) viewer = ptr<PointCloudViewer>(new PointCloudViewer());
  if(view) viewer = ptr<PclViewer>(new PclViewer("pclRawInput"));
}
