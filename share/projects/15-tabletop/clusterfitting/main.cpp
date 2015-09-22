#include <Core/util.h>
#include <pr2/roscom.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PointStamped.h>
#include <Core/array-vector.h>
#include <Perception/plane.h>
#include <Perception/object_detector.h>
#include <tf/transform_listener.h>


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

        pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);
        bool found=false;

        visualization_msgs::MarkerArray marker_array;

        for(auto& marker:msg->markers){
            pcl::PointCloud<PointT>::Ptr onePCL (new pcl::PointCloud<PointT>);
            conv_2_PCL(onePCL,marker.points);



            // detect cylinder
            pcl::PointCloud<pcl::Normal>::Ptr normal_extracted (new pcl::PointCloud<pcl::Normal>);
            normalEstimator(onePCL,normal_extracted,50);

            pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices);
            found = cylinderDetector(onePCL,normal_extracted,coefficients_cylinder,inliers_cylinder,0.01,0.05);

            if(found)
            {
                visualization_msgs::Marker marker;
                marker.ns = "my_namespace";
                marker.id = rand(); // TODO

                marker.header.frame_id = "base_link";
                marker.header.stamp = ros::Time();
                marker.action = visualization_msgs::Marker::ADD;
                marker.lifetime = ros::Duration(2.);




                marker.type = visualization_msgs::Marker::CYLINDER;

                // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
                marker.action = visualization_msgs::Marker::ADD;

                // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header

                geometry_msgs::PointStamped pt;
                 geometry_msgs::PointStamped pt_transformed;
                  //pt.header  =  coefficients_cylinder->header;
                  pt.header.frame_id  = marker.header.frame_id;
                  pt.header.stamp     = marker.header.stamp;
                  pt.point.x = coefficients_cylinder->values[0];
                  pt.point.y = coefficients_cylinder->values[1];
                  pt.point.z = coefficients_cylinder->values[2];

                  tf::TransformListener listener;
                  const std::string target_frame = "base_link";
                  const std::string original_frame = "camera_rgb_optical_frame";
                  const ros::Time time = ros::Time::now();
                  listener.waitForTransform(target_frame, original_frame, time, ros::Duration(1.0));
                  listener.transformPoint(target_frame, time, pt, original_frame, pt_transformed);


                marker.pose.position.x =  pt_transformed.point.x;
                marker.pose.position.y =  pt_transformed.point.y;
                marker.pose.position.z =  pt_transformed.point.z;
                double ax = coefficients_cylinder->values[3];
                double ay = coefficients_cylinder->values[4];
                double az = coefficients_cylinder->values[5];

                ors::Quaternion quat;
                quat.setDiff(ors::Vector(0,0,1), ors::Vector(ax, ay, az));


                marker.pose.orientation.x = quat.x;
                marker.pose.orientation.y = quat.y;
                marker.pose.orientation.z = quat.z;
                marker.pose.orientation.w = quat.w;

                // Set the scale of the marker -- 1x1x1 here means 1m on a side
                marker.scale.x = coefficients_cylinder->values[6];
                marker.scale.y = coefficients_cylinder->values[6];
                marker.scale.z = coefficients_cylinder->values[6];

                // Set the color -- be sure to set alpha to something non-zero!
                marker.color.r = 0.0f;
                marker.color.g = 1.0f;
                marker.color.b = 0.0f;
                marker.color.a = 1.0;

                marker_array.markers.push_back(marker);
            }

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
