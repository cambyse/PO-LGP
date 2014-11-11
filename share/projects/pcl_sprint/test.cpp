#include <pcl/point_representation.h>

#include <iostream>

using std::cout;
using std::endl;

int main(int argn, char ** args) {

    pcl::PointXYZRGB point(1,10,100);
    point.x = 2;
    point.y = 20;
    point.z = 200;

    auto rgb = point.getRGBVector3i();
    cout << "(" << point.x << "," << point.y << "," << point.z << "/" << rgb.x() << "," << rgb.y() << "," << rgb.z() << ")" << endl;

    // pcl::PointRepresentation<pcl::PointXYZ> rep;
    // cout << rep.getNumberOfDimensions() << endl;

    return 0;
}
