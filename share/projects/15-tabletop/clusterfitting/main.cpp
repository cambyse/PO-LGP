#include <Core/util.h>
#include <pr2/roscom.h>
#include <visualization_msgs/MarkerArray.h>
#include <Core/array-vector.h>
#include "plane.h"
#include "object_detector.h"


void conv_2_PCL(pcl::PointCloud<PointT>::Ptr Cloud, const std::vector<geometry_msgs::Point>& pts){
    uint n=pts.size();
    for(uint i=0;i<n;i++){
        pcl::PointXYZRGB point(0,255,0);
        point.x = pts[i].x;
        point.y = pts[i].y;
        point.z = pts[i].z;
        Cloud->points.push_back(point);
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


struct Fitting{
    ros::NodeHandle* nh;
    ros::Subscriber sub;
    ros::Publisher pub;
    double threshold;

    MT::Array<std::tuple<int, arr, arr> > trackedClusters;

    Fitting():threshold(.1){
        nh = new ros::NodeHandle;
        sub = nh->subscribe( "/tabletop/clusters", 1, &Fitting::callback, this);
        //pub = nh->advertise<visualization_msgs::MarkerArray>("/tabletop/tracked_clusters", 1);
        //sub = nh->subscribe( "/tabletop/tracked_clusters", 1, &Fitting::callback, this);
        pub = nh->advertise<visualization_msgs::MarkerArray>("/tabletop/fitted_clusters", 1);
    }
    ~Fitting(){
        nh->shutdown();
        delete nh;
    }

    void callback(const visualization_msgs::MarkerArray::ConstPtr& msg) {
        //uint n=msg->markers.size();

        for(auto& marker:msg->markers){
            pcl::PointCloud<PointT>::Ptr onePCL (new pcl::PointCloud<PointT>);
            conv_2_PCL(onePCL,marker.points);

            bool found=false;

            // detect cylinder
            pcl::PointCloud<pcl::Normal>::Ptr normal_extracted (new pcl::PointCloud<pcl::Normal>);
            normalEstimator(onePCL,normal_extracted,50);
            pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices);
            found = cylinderDetector(onePCL,normal_extracted,coefficients_cylinder,inliers_cylinder,0.01,0.05);

            if(!found){

                cout<<"found one cylinder "<<endl;
            }
        }

        cout <<"#clusters = " <<trackedClusters.N <<endl;

        visualization_msgs::MarkerArray marker_array;
        for(auto& cluster:trackedClusters) {
            visualization_msgs::Marker marker;
            marker.ns = "my_namespace";
            marker.id = rand(); // TODO
            marker.type = visualization_msgs::Marker::POINTS;
            marker.points = conv_arr2points(std::get<2>(cluster));

            marker.header.frame_id = "base_link";
            marker.header.stamp = ros::Time();
            marker.action = visualization_msgs::Marker::ADD;


            marker.color.a = 1.0; // Don't forget to set the alpha!
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;

            marker_array.markers.push_back(marker);
        }
        pub.publish(marker_array);

    }
};


void doit(){
    rosCheckInit();

    Fitting fitting;

//    for(;;){
//        ros::spinOnce();
//    }

    ros::spin();

}


int main(int argc, char** argv){
  MT::initCmdLine(argc, argv);
  cout<<"fitting cluster "<<endl;
  doit();
  return 0;
}
