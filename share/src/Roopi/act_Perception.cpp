#include "act_Perception.h"

#include <Perception/filter.h>
#include <Perception/percViewer.h>
#include <Perception/kinect2pointCloud.h>
#include <Gui/viewer.h>
#include <PCL/pclViewer.h>
#include <PCL/pipeline.h>

Act_PerceptionFilter::Act_PerceptionFilter(Roopi* r, bool view)
  : Act(r){
  filter = make_shared<Act_Filter>(r, new Filter);
  if(view) viewerInput = make_shared<Act_PercViewer>(r, new PercViewer("percepts_input"));
  if(view) viewerFiltered = make_shared<Act_PercViewer>(r, new PercViewer("percepts_filtered"));
}


Act_PclPipeline::Act_PclPipeline(Roopi* r, bool view)
  : Act(r){
  kin2cloud = make_shared<Kinect2PointCloud>();
  cloud2pcl = make_shared<Conv_arr_pcl>("pclRawInput", "kinect_points", "kinect_rgb");
  pclPipeline = make_shared<PclPipeline>("pclRawInput");
//  if(view) viewer = make_shared<PointCloudViewer>();
  if(view) viewer = make_shared<PclViewer>("pclRawInput");
}
