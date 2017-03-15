#pragma once

#include "act.h"
#include "act_Thread.h"

typedef Act_Th<struct Filter> Act_Filter;
typedef Act_Th<struct PercViewer> Act_PercViewer;

struct Act_PerceptionFilter : Act{
  Act_PerceptionFilter(Roopi *r, bool view=true);
  ptr<Act_Filter> filter;
  ptr<Act_PercViewer> viewer;
};

struct Act_PclPipeline : Act{
  Act_PclPipeline(Roopi *r, bool view=true);
  ptr<struct Kinect2PointCloud> kin2cloud;
  ptr<struct Conv_arr_pcl> cloud2pcl;
  ptr<struct PclPipeline> pclPipeline;
//  ptr<struct PointCloudViewer> viewer;
  ptr<struct PclViewer> viewer;
};
