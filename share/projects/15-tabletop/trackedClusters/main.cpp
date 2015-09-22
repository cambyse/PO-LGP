#include <Core/util.h>
#include <pr2/roscom.h>
#include <visualization_msgs/MarkerArray.h>
#include <Core/array-vector.h>

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


struct Tracker{
    ros::NodeHandle* nh;
    ros::Subscriber sub;
    ros::Publisher pub;
    double threshold;

    MT::Array<std::tuple<int, arr, arr> > trackedClusters;

    Tracker():threshold(.1){
        nh = new ros::NodeHandle;
        sub = nh->subscribe( "/tabletop/clusters", 1, &Tracker::callback, this);
        pub = nh->advertise<visualization_msgs::MarkerArray>("/tabletop/tracked_clusters", 1);
    }
    ~Tracker(){
        nh->shutdown();
        delete nh;
    }

    void callback(const visualization_msgs::MarkerArray::ConstPtr& msg) {
        uint n=msg->markers.size();
//        cout <<"n=" <<n <<endl;

        for(auto& marker:msg->markers){
            arr pts = conv_points2arr(marker.points);
//            cout <<pts.dim() <<endl;
            arr mean = sum(pts,0)/(double)pts.d0;
//            cout <<mean <<endl;
            bool found=false;
            for(auto& tc:trackedClusters){
                if(length(mean - std::get<1>(tc)) < threshold){
                    found = true;
                    std::get<2>(tc) = pts;
                }
            }
            if(!found){
                trackedClusters.append(std::make_tuple(13, mean, pts));
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

    Tracker tracker;

//    for(;;){
//        ros::spinOnce();
//    }

    ros::spin();

}


int main(int argc, char** argv){
  MT::initCmdLine(argc, argv);
  doit();
  return 0;
}
