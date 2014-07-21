#ifndef TESTMETHOD_H_
#define TESTMETHOD_H_

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

class TestMethod {
    //----typedefs/classes----//

    //----members----//

    //----methods----//
public:
    TestMethod() = default;
    virtual ~TestMethod() = default;
    static void process(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr & output);
};

#endif /* TESTMETHOD_H_ */
