#pragma once

#include "act.h"
#include "act_Thread.h"

typedef Act_Th<struct Filter> Act_Filter;
typedef Act_Th<struct PercViewer> Act_PercViewer;

struct Act_PerceptionFilter : Act{
  Act_PerceptionFilter(Roopi *r, bool view=true);
  shared_ptr<Act_Filter> filter;
  shared_ptr<Act_PercViewer> viewerInput;
  shared_ptr<Act_PercViewer> viewerFiltered;
};

struct Act_PclPipeline : Act{
  Act_PclPipeline(Roopi *r, bool view=true);
  shared_ptr<struct Kinect2PointCloud> kin2cloud;
  shared_ptr<struct Conv_arr_pcl> cloud2pcl;
  shared_ptr<struct PclPipeline> pclPipeline;
//  shared_ptr<struct PointCloudViewer> viewer;
  shared_ptr<struct PclViewer> viewer;
};
