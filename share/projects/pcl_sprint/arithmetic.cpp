#include "arithmetic.h"

namespace arithmetic {

    std::tuple<double,double,double,double,double,double> PointXYZRGB_to_tuple(const pcl::PointXYZRGB & point) {
        std::tuple<double,double,double,double,double,double> ret;
        std::get<0>(ret) = point.x;
        std::get<1>(ret) = point.y;
        std::get<2>(ret) = point.z;
        auto rgb = point.getRGBVector3i();
        std::get<3>(ret) = (double)rgb.x();
        std::get<4>(ret) = (double)rgb.y();
        std::get<5>(ret) = (double)rgb.z();
        return ret;
    }

    pcl::PointXYZRGB tuple_to_PointXYZRGB(const std::tuple<double,double,double,double,double,double> & t) {
        pcl::PointXYZRGB point(std::get<3>(t),std::get<4>(t),std::get<5>(t));
        point.x = std::get<0>(t);
        point.y = std::get<1>(t);
        point.z = std::get<2>(t);
        return point;
    }

    std::tuple<double,double,double,double,double,double> add_tuples(
        const std::tuple<double,double,double,double,double,double> & t1,
        const std::tuple<double,double,double,double,double,double> & t2) {
        return std::make_tuple(std::get<0>(t1)+std::get<0>(t2),
                               std::get<1>(t1)+std::get<1>(t2),
                               std::get<2>(t1)+std::get<2>(t2),
                               std::get<3>(t1)+std::get<3>(t2),
                               std::get<4>(t1)+std::get<4>(t2),
                               std::get<5>(t1)+std::get<5>(t2));
    }

    std::tuple<double,double,double,double,double,double> subtract_tuples(const std::tuple<double,double,double,double,double,double> & t1,
                                                                          const std::tuple<double,double,double,double,double,double> & t2) {
        return std::make_tuple(std::get<0>(t1)-std::get<0>(t2),
                               std::get<1>(t1)-std::get<1>(t2),
                               std::get<2>(t1)-std::get<2>(t2),
                               std::get<3>(t1)-std::get<3>(t2),
                               std::get<4>(t1)-std::get<4>(t2),
                               std::get<5>(t1)-std::get<5>(t2));
    }

    std::tuple<double,double,double,double,double,double> multiply_tuple(const std::tuple<double,double,double,double,double,double> & t, const double & d) {
        return std::make_tuple(std::get<0>(t)*d,std::get<1>(t)*d,std::get<2>(t)*d,std::get<3>(t)*d,std::get<4>(t)*d,std::get<5>(t)*d);
    }

} // end namespace arithmetic
