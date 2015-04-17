#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class SimpleOpenNIViewer {
private:
	struct sViewer *s;
public:
	SimpleOpenNIViewer();
	~SimpleOpenNIViewer();
	void run();
};
