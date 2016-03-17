#include <Core/util.h>
#include <pr2/roscom.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf/transform_listener.h>
#include <object_recognition_msgs/TableArray.h>

#include <set>
#include <list>

using std::cout;
using std::endl;
using std::set;
using std::list;

namespace std {
ostream & operator<<(ostream & out, const tf::Vector3 & vec) {
    out << "(" << vec.x() << "," << vec.y() << "," << vec.z() << ")";
    return out;
}
}

arr conv_points2arr(const std::vector<geometry_msgs::Point>& pts){
        uint n=pts.size();
        arr P(n,3);
        for(uint i=0;i<n;i++){
            P(i,0) = pts[i].x;
            P(i,1) = pts[i].y;
            P(i,2) = pts[i].z;
        }
        return P;
}

std::vector<geometry_msgs::Point> conv_arr2points(const arr& pts){
    uint n=pts.d0;
    std::vector<geometry_msgs::Point> P(n);
    for(uint i=0;i<n;i++){
        P[i].x = pts(i, 0);
        P[i].y = pts(i, 1);
        P[i].z = pts(i, 2);
    }
    return P;
}

double distance(const arr& b, const arr& m, const arr& p, double& mi) {
    arr mn = m/length(m);
    arr delta = p - b;
    mi = scalarProduct(delta, mn);
    arr d = delta - mi*mn;
    return length(d);
}

void publish_coordinate_frame(ros::Publisher & publisher,
                              const tf::Vector3 & zero_vec,
                              const tf::Vector3 & x_vec,
                              const tf::Vector3 & y_vec,
                              const tf::Vector3 & z_vec) {
    visualization_msgs::MarkerArray marker_array;
    for(auto vec : {zero_vec, x_vec, y_vec, z_vec}) {
        visualization_msgs::Marker marker;
        marker.header.stamp = ros::Time();
        marker.id = rand();
        marker.lifetime = ros::Duration(0.5);
        marker.header.frame_id = "base_link";
        if(vec==zero_vec) {
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.pose.position.x = zero_vec.x();
            marker.pose.position.y = zero_vec.y();
            marker.pose.position.z = zero_vec.z();
            marker.color.a = 1;
            marker.color.r = 1;
            marker.color.g = 1;
            marker.color.b = 1;
            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
        } else {
            marker.type = visualization_msgs::Marker::ARROW;
            marker.points.resize(2);
            marker.points[0].x = zero_vec.x();
            marker.points[0].y = zero_vec.y();
            marker.points[0].z = zero_vec.z();
            marker.points[1].x = vec.x();
            marker.points[1].y = vec.y();
            marker.points[1].z = vec.z();
            marker.color.a = 1;
            marker.color.r = vec==x_vec;
            marker.color.g = vec==y_vec;
            marker.color.b = vec==z_vec;
            marker.scale.x = 0.05;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
        }
//        cout << "vec: (" << vec.x() << "," << vec.y() << "," << vec.z() << ")" << endl;
        marker_array.markers.push_back(marker);
    }
    publisher.publish(marker_array);
}

struct Table {
    arr pos;
    arr convex_hull;
    tf::Transform transform;
    Table(const arr& pos,
          const arr& convex_hull,
          tf::Transform transform):
        pos(pos),
        convex_hull(convex_hull),
        transform(transform)
    {}
};

struct ClusterFilter{
    //----types/classes----/

    //----members----//
    ros::NodeHandle* nh;
    ros::Subscriber sub_clusters;
    ros::Subscriber sub_table;
    ros::Publisher pub;
    tf::TransformListener listener;
    std::vector<std::shared_ptr<Table> > tables;
    bool show_base_coordinate_frame = true;
    bool show_camera_coordinate_frame = true;
    bool show_table_coordinate_frames = true;
    bool show_means_as_spheres = true;
    bool show_tables_convex_hull_with_dots = true;
    double min_table_height = 0.3;

    //----methods----//
    ClusterFilter(){
        nh = new ros::NodeHandle;
        sub_clusters = nh->subscribe( "/tabletop/clusters", 1, &ClusterFilter::cluster_callback, this);
        sub_table = nh->subscribe( "/table_array", 1, &ClusterFilter::table_callback, this);
        pub = nh->advertise<visualization_msgs::MarkerArray>("/tabletop/filtered_clusters", 1);
    }
    ~ClusterFilter(){
        nh->shutdown();
        delete nh;
    }
    void table_callback(const object_recognition_msgs::TableArray& msg) {
        tables.clear();
        for(auto& table: msg.tables) {
            tf::Transform base_to_table_transform;
            {
                // get base-->head
                tf::StampedTransform base_to_head_transform;
                try{
                    listener.lookupTransform("/base_link", msg.header.frame_id, ros::Time(0), base_to_head_transform);
                }
                catch (tf::TransformException &ex) {
                    ROS_ERROR("%s",ex.what());
                    ros::Duration(1.0).sleep();
                    continue;
                }
                // get head-->table
                tf::Transform head_to_table_transform;
                head_to_table_transform.setIdentity();
                head_to_table_transform.setOrigin(tf::Vector3(table.pose.position.x,
                                                              table.pose.position.y,
                                                              table.pose.position.z));
                head_to_table_transform.setRotation(tf::Quaternion(table.pose.orientation.x,
                                                                   table.pose.orientation.y,
                                                                   table.pose.orientation.z,
                                                                   table.pose.orientation.w));
                // get base-->table by concatenating
                base_to_table_transform = base_to_head_transform * head_to_table_transform;
            }
            // basis vectors (they transform as points actually!!) of interest
            tf::Vector3 zero_vec(0,0,0);
            tf::Vector3 z_vec(0,0,1);
            // transform basis vectors
            zero_vec = base_to_table_transform * zero_vec;
            z_vec = base_to_table_transform * z_vec;
            // the actual z-VECTOR (as opposed to point) is the difference
            tf::Vector3 true_z_vec = z_vec - zero_vec;
            if(fabs(true_z_vec.dot(tf::Vector3(0,0,1))) >= .9 and zero_vec.z() > min_table_height) {
                arr zero_arr = { zero_vec.x(), zero_vec.y(), zero_vec.z() };
                arr convex_hull(table.convex_hull.size(), 3);
                for(uint i=0; i<table.convex_hull.size(); ++i) {
                    auto& point = table.convex_hull[i];
                    tf::Vector3 p = base_to_table_transform * tf::Vector3(point.x, point.y, point.z);
                    convex_hull(i, 0) = p.x();
                    convex_hull(i, 1) = p.y();
                    convex_hull(i, 2) = p.z();
                }
                std::shared_ptr<Table> new_table = std::shared_ptr<Table>(new Table(zero_arr, convex_hull, base_to_table_transform));
                tables.push_back(new_table);
            }
        }

        // publish some coordinate systems
        if(show_base_coordinate_frame) {
            // base system
            publish_coordinate_frame(pub, tf::Vector3(0,0,0), tf::Vector3(1,0,0), tf::Vector3(0,1,0), tf::Vector3(0,0,1));
        }
        if(show_camera_coordinate_frame) {
            // camera system
            tf::Vector3 zero_vec(0,0,0);
            tf::Vector3 x_vec(1,0,0);
            tf::Vector3 y_vec(0,1,0);
            tf::Vector3 z_vec(0,0,1);
            tf::StampedTransform base_to_head_transform;
            try{
                listener.lookupTransform("/base_link", msg.header.frame_id, ros::Time(0), base_to_head_transform);
            }
            catch (tf::TransformException &ex) {
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
            }
            for(auto * vec : {&zero_vec, &x_vec, &y_vec, &z_vec}) *vec = base_to_head_transform * (*vec);
            // show coordinate frame
            publish_coordinate_frame(pub, zero_vec, x_vec, y_vec, z_vec);
        }
        if(show_table_coordinate_frames) {
            // base tables
            for(auto table : tables) {
                tf::Vector3 zero_vec(0,0,0);
                tf::Vector3 x_vec(1,0,0);
                tf::Vector3 y_vec(0,1,0);
                tf::Vector3 z_vec(0,0,1);
                for(auto * vec : {&zero_vec, &x_vec, &y_vec, &z_vec}) *vec = table->transform * (*vec);
                // show coordinate frame
                publish_coordinate_frame(pub, zero_vec, x_vec, y_vec, z_vec);
            }
        }
    }

    void cluster_callback(const visualization_msgs::MarkerArray& msg) {
        visualization_msgs::MarkerArray filtered_marker;
        cout << "#tables " << tables.size() << endl;
        for(auto &marker: msg.markers) {
            bool marker_on_some_table = false;
            // get mean in base coordinates
            tf::Vector3 mean_in_base;
            {
                // get transform
                tf::StampedTransform head_to_base_transform;
                try{
                    listener.lookupTransform("base_link", marker.header.frame_id, ros::Time(0), head_to_base_transform);
                }
                catch (tf::TransformException &ex) {
                    ROS_ERROR("%s",ex.what());
                    ros::Duration(1.0).sleep();
                    return;
                }
                // get mean
                arr pts = conv_points2arr(marker.points);
                arr mean = sum(pts,0)/(double)pts.d0;
                // transform mean
                mean_in_base = head_to_base_transform * tf::Vector3(mean(0), mean(1), mean(2));
            }

            // check if mean in on some table
            for (auto table : tables) {
                // visualize convex hull
                if(show_tables_convex_hull_with_dots) {
                    for(uint i = 0; i<table->convex_hull.d0; ++i) {
                        visualization_msgs::Marker sphere;
                        sphere.header.stamp = ros::Time();
                        sphere.id = rand();
                        sphere.lifetime = ros::Duration(0.5);
                        sphere.header.frame_id = "base_link";
                        sphere.type = visualization_msgs::Marker::SPHERE;
                        sphere.pose.position.x = table->convex_hull(i, 0);
                        sphere.pose.position.y = table->convex_hull(i, 1);
                        sphere.pose.position.z = table->convex_hull(i, 2);
                        sphere.scale.x = .02;
                        sphere.scale.y = .02;
                        sphere.scale.z = .02;
                        sphere.color.a = 1;
                        sphere.color.r = 1;
                        sphere.color.g = 0;
                        sphere.color.b = 0;
                        filtered_marker.markers.push_back(sphere);
                    }
                }
                // check if mean is within table convex hull
                {
//                    arr mean_in_base_arr = {mean_in_base.x(), mean_in_base.y()};
//                    arr convex_hull_2D(table->convex_hull.d0, 2);
//                    for(uint i=0; i<table->convex_hull.d0; ++i) {
//                        convex_hull_2D(i, 0) = table->convex_hull(i, 0);
//                        convex_hull_2D(i, 1) = table->convex_hull(i, 1);
//                    }
//                    double dist = distanceToConvexHull(convex_hull_2D, mean_in_base_arr, nullptr, nullptr, true);
//                    marker_on_some_table = marker_on_some_table or dist < 0;
                }
                // check if mean in within table bounding box
                {
                    double minX = DBL_MAX;
                    double minY = DBL_MAX;
                    double maxX = -DBL_MAX;
                    double maxY = -DBL_MAX;
                    for(uint p_idx=0; p_idx<table->convex_hull.d0; ++p_idx) {
                        arr point = table->convex_hull[p_idx];
                        minX = std::min(minX, point(0));
                        maxX = std::max(maxX, point(0));
                        minY = std::min(minY, point(1));
                        maxY = std::max(maxY, point(1));
                    }
                    // show marker (break to not show multiple times)
                    marker_on_some_table = marker_on_some_table or (
                                mean_in_base.x() > minX and
                                mean_in_base.x() < maxX and
                                mean_in_base.y() > minY and
                                mean_in_base.y() < maxY
                                );
                }
                if(marker_on_some_table) break;
            }
            if(marker_on_some_table) {
                // publish the cluster
                filtered_marker.markers.push_back(marker);
                // visualize mean as sphere
                if(show_means_as_spheres) {
                    visualization_msgs::Marker sphere;
                    sphere.header.stamp = ros::Time();
                    sphere.id = rand();
                    sphere.lifetime = ros::Duration(0.5);
                    sphere.header.frame_id = marker.header.frame_id;
                    sphere.header.frame_id = "base_link";
                    sphere.type = visualization_msgs::Marker::SPHERE;
                    sphere.pose.position.x = mean_in_base.x();
                    sphere.pose.position.y = mean_in_base.y();
                    sphere.pose.position.z = mean_in_base.z();
                    sphere.scale.x = .1;
                    sphere.scale.y = .1;
                    sphere.scale.z = .1;
                    sphere.color.a = 1;
                    sphere.color.r = 0;
                    sphere.color.g = 1;
                    sphere.color.b = 0;
                    filtered_marker.markers.push_back(sphere);
                }
            }
        }
        cout << "Size: " << filtered_marker.markers.size() << endl;
        pub.publish(filtered_marker);
    }

};


void doit(){
    ros::init(mlr::argc, mlr::argv, "cluster_fitter", ros::init_options::NoSigintHandler);

    ClusterFilter fitter;

    cout << "started" << endl;
    ros::spin();

}


int main(int argc, char** argv){
    mlr::initCmdLine(argc, argv);
    doit();
    return 0;
}
