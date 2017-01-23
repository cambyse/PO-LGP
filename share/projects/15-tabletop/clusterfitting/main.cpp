#include <Core/util.h>
#include <RosCom/roscom.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PointStamped.h>
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

    mlr::Array<std::tuple<int, arr, arr> > trackedClusters;

    Fitting():threshold(.1){
        nh = new ros::NodeHandle;
        sub = nh->subscribe( "/tabletop/clusters", 1, &Fitting::callback, this);
        //sub = nh->subscribe( "/tabletop/tracked_clusters", 1, &Fitting::callback, this);
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

            bool foundCylinder = cylinderDetector(onePCL,normal_extracted,coefficients_cylinder,inliers_cylinder,0.01,0.05);


            //Returning the parametric model
            if(foundCylinder)
            {

                cout<<"found cylinder "<<endl;


                int markerid = marker.id;

                visualization_msgs::Marker marker_add, marker_del;
/*/
                marker_del.id = markerid; // TODO

                marker_del.ns = "my_namespace";
                marker_del.header.frame_id = "head_mount_kinect_rgb_optical_frame";
                marker_del.header.stamp = ros::Time();
                marker_del.action = visualization_msgs::Marker::DELETE;
                //marker.lifetime = ros::Duration(2.);
                marker_del.type = visualization_msgs::Marker::CYLINDER;


/*/
                marker_add.ns = "my_namespace";
                marker_add.id = markerid;
                marker_add.header.frame_id = "head_mount_kinect_rgb_optical_frame";
                marker_add.header.stamp = ros::Time();
                marker_add.action = visualization_msgs::Marker::ADD;
                marker_add.lifetime = ros::Duration(15.);
                marker_add.type = visualization_msgs::Marker::CYLINDER;


                marker_add.pose.position.x =  coefficients_cylinder->values[0];
                marker_add.pose.position.y =  coefficients_cylinder->values[1];
                marker_add.pose.position.z =  coefficients_cylinder->values[2];

                double ax = coefficients_cylinder->values[3];
                double ay = coefficients_cylinder->values[4];
                double az = coefficients_cylinder->values[5];

                mlr::Quaternion quat;
                quat.setDiff(mlr::Vector(0,0,1), mlr::Vector(ax, ay, az));


                marker_add.pose.orientation.x = quat.x;
                marker_add.pose.orientation.y = quat.y;
                marker_add.pose.orientation.z = quat.z;
                marker_add.pose.orientation.w = quat.w;

                // Set the scale of the marker -- 1x1x1 here means 1m on a side
                marker_add.scale.x = 2*coefficients_cylinder->values[6];
                marker_add.scale.y = 2*coefficients_cylinder->values[6];
                marker_add.scale.z = 2*coefficients_cylinder->values[6];

                // Set the color -- be sure to set alpha to something non-zero!
                marker_add.color.r = 0.0f;
                marker_add.color.g = 1.0f;
                marker_add.color.b = 0.0f;
                marker_add.color.a = 1.0;

                marker_array.markers.push_back(marker_add);
            }


            mlr::Quaternion orientation;
            arr center(3);
            arr length(3);
            std::vector<pcl::ModelCoefficients::Ptr> outCoefficients;

            bool foundCube = IsABox(onePCL,normal_extracted,outCoefficients,orientation,center,length);

            //Returning the parametric model
            if(foundCube)
            {

                cout<<"found a box "<<endl;
                int markerid = rand();

                visualization_msgs::Marker marker_add, marker_del;
/*/
                marker_del.id = markerid; // TODO

                marker_del.ns = "my_namespace";
                marker_del.header.frame_id = "head_mount_kinect_rgb_optical_frame";
                marker_del.header.stamp = ros::Time();
                marker_del.action = visualization_msgs::Marker::DELETE;
                //marker.lifetime = ros::Duration(2.);
                marker_del.type = visualization_msgs::Marker::CUBE;
/*/


                marker_add.ns = "my_namespace";
                marker_add.id = markerid;
                marker_add.header.frame_id = "head_mount_kinect_rgb_optical_frame";
                marker_add.header.stamp = ros::Time();
                marker_add.action = visualization_msgs::Marker::ADD;
                marker_add.lifetime = ros::Duration(10.);
                marker_add.type = visualization_msgs::Marker::CUBE;


                marker_add.pose.position.x =  center(0);
                marker_add.pose.position.y =  center(1);
                marker_add.pose.position.z =  center(2);


                marker_add.pose.orientation.x = orientation.x;
                marker_add.pose.orientation.y = orientation.y;
                marker_add.pose.orientation.z = orientation.z;
                marker_add.pose.orientation.w = orientation.w;

               // Set the scale of the marker -- 1x1x1 here means 1m on a side
               marker_add.scale.x = length(0);
               marker_add.scale.y = length(1);
               marker_add.scale.z = length(2);


                // Set the color -- be sure to set alpha to something non-zero!
                marker_add.color.r = 1.0f;
                marker_add.color.g = 0.0f;
                marker_add.color.b = 0.0f;
                marker_add.color.a = 1.0;

                marker_array.markers.push_back(marker_add);

                //plane
                for(int ii=0;ii<3;ii++){
                    visualization_msgs::Marker marker_add, marker_del;
    /*/
                    marker_del.id = markerid; // TODO

                    marker_del.ns = "my_namespace";
                    marker_del.header.frame_id = "head_mount_kinect_rgb_optical_frame";
                    marker_del.header.stamp = ros::Time();
                    marker_del.action = visualization_msgs::Marker::DELETE;
                    //marker.lifetime = ros::Duration(2.);
                    marker_del.type = visualization_msgs::Marker::CUBE;
    /*/


                    marker_add.ns = "my_namespace";
                    marker_add.id = rand();
                    marker_add.header.frame_id = "head_mount_kinect_rgb_optical_frame";
                    marker_add.header.stamp = ros::Time();
                    marker_add.action = visualization_msgs::Marker::ADD;
                    marker_add.lifetime = ros::Duration(10.);
                    marker_add.type = visualization_msgs::Marker::ARROW;


                    marker_add.pose.position.x =  center(0);
                    marker_add.pose.position.y =  center(1);
                    marker_add.pose.position.z =  center(2);

                    double ax = outCoefficients[ii]->values[0];
                    double ay = outCoefficients[ii]->values[1];
                    double az = outCoefficients[ii]->values[2];

                    mlr::Quaternion quat;
                    quat.setDiff(mlr::Vector(1,0,0), mlr::Vector(ax, ay, az));


                    marker_add.pose.orientation.x = quat.x;
                    marker_add.pose.orientation.y = quat.y;
                    marker_add.pose.orientation.z = quat.z;
                    marker_add.pose.orientation.w = quat.w;

                   // Set the scale of the marker -- 1x1x1 here means 1m on a side
                   marker_add.scale.x = 0.5;
                   marker_add.scale.y = 0.05;
                   marker_add.scale.z = 0.05;


                    // Set the color -- be sure to set alpha to something non-zero!
                    marker_add.color.r = 0.0f;
                    marker_add.color.g = 0.0f;
                    marker_add.color.b = 1.0f;
                    marker_add.color.a = 1.0;

                    marker_array.markers.push_back(marker_add);
                }




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
  mlr::initCmdLine(argc, argv);
  cout<<"fitting cluster "<<endl;
  doit();
  return 0;
}
