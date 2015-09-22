#include <Core/util.h>
#include <pr2/roscom.h>
#include <visualization_msgs/MarkerArray.h>
#include <Core/array-vector.h>

#include <set>
#include <list>

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


struct Tracker{
    //----types/classes----/
    struct Cluster {
        Cluster() {}
        Cluster(int id,
                arr mean,
                arr points,
                double relevance,
                bool active,
                std::string frame_id):
            id(id),
            mean(mean),
            points(points),
            relevance(relevance),
            active(active),
            frame_id(frame_id)
        {}
        int id;
        arr mean;
        arr points;
        double relevance;
        bool active;
        std::string frame_id;
    };

    //----members----//
    ros::NodeHandle* nh;
    ros::Subscriber sub;
    ros::Publisher pub;
    double threshold;
    list<Cluster> tracked_clusters;
    set<int> id_set;
    double relevance_decay_factor = 0.9;
    double relevance_threshold = 0.01;

    //----methods----//
    Tracker():threshold(.1){
        nh = new ros::NodeHandle;
        sub = nh->subscribe( "/tabletop/clusters", 1, &Tracker::callback, this);
        pub = nh->advertise<visualization_msgs::MarkerArray>("/tabletop/tracked_clusters", 1);
    }
    ~Tracker(){
        nh->shutdown();
        delete nh;
    }

    void callback(const visualization_msgs::MarkerArray& msg) {
        // go through currently provided clusters
        visualization_msgs::MarkerArray marker_array;
        for(auto & marker : msg.markers){
            arr pts = conv_points2arr(marker.points);
            arr mean = sum(pts,0)/(double)pts.d0;
            // find matching cluster
            bool found = false;
            int id;
            for(auto & cluster : tracked_clusters){
                if(length(mean - cluster.mean) < threshold){
                    found = true;
                    cluster.mean = mean;
                    cluster.points = pts;
                    cluster.active = true;
                    cluster.relevance = 1;
                    cluster.frame_id = marker.header.frame_id;
                    id = cluster.id;
                    break;
                }
            }
            // new cluster if not found
            if(!found){
                id = 1;
                while(id_set.find(id)!=id_set.end()) ++id;
                id_set.insert(id);
                tracked_clusters.push_back(Cluster(id, mean, pts, 1, true, marker.header.frame_id));
            }

        }

        // show clusters (old and new)
        auto cluster_it = tracked_clusters.begin();
        while(cluster_it != tracked_clusters.end()) {
            // update cluster relevance
            cluster_it->relevance *= relevance_decay_factor;
            // remove clusters with low relevance
            if(cluster_it->relevance<relevance_threshold) {
                auto next_it = cluster_it;
                ++next_it;
                id_set.erase(cluster_it->id);
                tracked_clusters.erase(cluster_it);
                cluster_it = next_it;
                break;
            }
            // make all cluster inactive
            cluster_it->active = false;

            visualization_msgs::Marker new_marker;
            new_marker.type = visualization_msgs::Marker::POINTS;
            new_marker.points = conv_arr2points(cluster_it->points);
            new_marker.scale.x = .001;
            new_marker.scale.y = .001;
            new_marker.id = cluster_it->id;
            new_marker.lifetime = ros::Duration(0.5);
            new_marker.header.stamp = ros::Time::now();
            new_marker.header.frame_id = cluster_it->frame_id;

            new_marker.color.a = 1.0; // Don't forget to set the alpha!
            new_marker.color.r = (double)((new_marker.id*10000)%97)/97;
            new_marker.color.g = (double)((new_marker.id*10000)%91)/91;
            new_marker.color.b = (double)((new_marker.id*10000)%89)/89;
            marker_array.markers.push_back(new_marker);

            ++cluster_it;
        }

        cout <<"#clusters = " <<tracked_clusters.size() <<endl;
        cout <<"#id = " << id_set.size() << " (max: " << *(id_set.end()) << ")" <<endl;

        pub.publish(marker_array);

    }
};


void doit(){
    ros::init(MT::argc, MT::argv, "cluster_tracker", ros::init_options::NoSigintHandler);

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
