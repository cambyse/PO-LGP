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

struct Table {
    arr pos;
    arr convexHull;
    Table(const arr& pos, const arr& convexHull) : pos(pos), convexHull(convexHull) {}
};

struct ClusterFilter{
    //----types/classes----/

    //----members----//
    ros::NodeHandle* nh;
    ros::Subscriber sub_clusters;
    ros::Subscriber sub_table;
    ros::Publisher pub;

    tf::TransformListener listener;

    bool table_found = false;


    std::vector<std::shared_ptr<Table> > tables;

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
            tf::StampedTransform transform;
            try{
                ors::Transformation X;
                listener.lookupTransform(msg.header.frame_id, "/base_link", ros::Time(0), transform);
            }
            catch (tf::TransformException &ex) {
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
                return;
            }
            tf::Transform transform_table;
            transform_table.setOrigin(tf::Vector3(table.pose.position.x,
                                                  table.pose.position.y,
                                                  table.pose.position.z));
            transform_table.setRotation(tf::Quaternion(table.pose.orientation.x,
                                                       table.pose.orientation.y,
                                                       table.pose.orientation.z,
                                                       table.pose.orientation.w));
            transform_table = transform_table.inverse();
            tf::Vector3 zVec(0, 0, 1);
            tf::Vector3 zVecBase = transform * transform_table * zVec;
            tf::Vector3 pos = transform * transform_table * tf::Vector3(0, 0, 0);

            double aligned = fabs(zVec.dot(zVecBase));

            if(aligned >= .8) {
                arr apos = { pos.x(), pos.y(), pos.z() };
                arr convexHull(table.convex_hull.size(), 2);
                for(uint i=0; i<table.convex_hull.size(); ++i) {
                    auto& point = table.convex_hull[i];
                    tf::Vector3 p = transform * transform_table * tf::Vector3(point.x, point.y, point.z);
                    convexHull(i, 0) = p.x();
                    convexHull(i, 1) = p.y();
                    //convexHull(i, 2) = p.z();  We only want the projection to XY
                }

                std::shared_ptr<Table> table = std::shared_ptr<Table>(new Table(apos, convexHull));
                tables.push_back(table);
            }
        }
        table_found = true;
    }

    void cluster_callback(const visualization_msgs::MarkerArray& msg) {
        if(!table_found) return;
        visualization_msgs::MarkerArray filtered_marker;
        for(auto &marker: msg.markers) {
            tf::StampedTransform transform;
            try{
                ors::Transformation X;
                listener.lookupTransform(marker.header.frame_id, "/base_link", ros::Time(0), transform);
            }
            catch (tf::TransformException &ex) {
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
                return;
            }
            arr pts = conv_points2arr(marker.points);
            arr mean = sum(pts,0)/(double)pts.d0;

            tf::Vector3 mb = transform * tf::Vector3(mean(0), mean(1), mean(2));
            arr mean_in_base = {mb.x(), mb.y()};

            for (auto table: tables) {
                double dist = distanceToConvexHull(table->convexHull, mean_in_base, nullptr, nullptr, true);
                if(dist < 0) {
                    filtered_marker.markers.push_back(marker);
                    break;
                }
            }
        }
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
