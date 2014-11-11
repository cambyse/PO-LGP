#include <pcl/common/common_headers.h>

#include <vector>
#include <tuple>

namespace arithmetic {

    std::tuple<double,double,double,double,double,double> PointXYZRGB_to_tuple(const pcl::PointXYZRGB & point);

    pcl::PointXYZRGB tuple_to_PointXYZRGB(const std::tuple<double,double,double,double,double,double> & t);

    std::tuple<double,double,double,double,double,double> add_tuples(
        const std::tuple<double,double,double,double,double,double> & t1,
        const std::tuple<double,double,double,double,double,double> & t2);

    std::tuple<double,double,double,double,double,double> subtract_tuples(const std::tuple<double,double,double,double,double,double> & t1,
                                                                          const std::tuple<double,double,double,double,double,double> & t2);

    std::tuple<double,double,double,double,double,double> multiply_tuple(const std::tuple<double,double,double,double,double,double> & t, const double & d);

} // end namespace arithmetic
