#include "environment.h"

environment::environment(bool useRos){
    if(useRos){
    track = myBaxter.task(GRAPH("map=pos ref1=endeffR ref2=base_footprint target=[0.7 -0.2 1.05] PD=[0.3, .8, 1., 1.]"));
    track_left = myBaxter.task(GRAPH("map=pos ref1=endeffL ref2=base_footprint target=[0.5 0.9 1.5] PD=[0.3, .8, 1., 1.]"));
    align1 = myBaxter.task(GRAPH("map=vecAlign ref1=endeffR ref2=base_footprint vec1=[1 0 0] vec2=[1 0 0] target=[0] PD=[0.3, .8, 1., 1.]"));
    align2 = myBaxter.task(GRAPH("map=vecAlign ref1=endeffR ref2=base_footprint vec1=[1 0 0] vec2=[0 1 0] target=[0] PD=[0.3, .8, 1., 1.]"));
    alignE = myBaxter.task(GRAPH("map=vecAlign ref1=elbowR ref2=base_footprint vec1=[0 0 1] vec2=[0 0 1] target=[0] PD=[0.3, .8, 1., 1.]"));


    z_arm = 1.05;
    state_space = ARR(0.35, 0.7, -0.7, 0.1); //(x_min, x_max, y_min, y_max)
    }

}
environment::~environment(){
    cout << "homing" << endl;
    auto home = myBaxter.task(GRAPH("map=qItself PD=[.5, 1., .2, 10.]"));
    myBaxter.modifyTarget(home, myBaxter.q0());
    myBaxter.stop({track, align1, align2});
    myBaxter.waitConv({home});
}

void environment::start(arr marker_pos, arr arm_pos, bool useRos){
    if(useRos){
        track_marker_position_baxter();
        arm_position = ARR(track->y(0), track->y(1));
    }else{
        marker_position = marker_pos;
        arm_position = arm_pos;
    }
}

arr environment::get_reward(arr state, double threshold){
    double reward_x;
    double reward_y;
    reward_x = -abs(state(0)-marker_position(0));
    reward_y = -abs(state(1)-marker_position(1));
    if(sqrDistance(state, marker_position) <= threshold){
        reward_x += 10;
        reward_y += 10;
    }

    return ARR(reward_x, reward_y);
}

arr environment::get_next_state(arr state, arr action){
    arr next_state = state + action;
    return next_state;
}

arr environment::get_next_state_baxter(arr state, arr action){
    arr target = ARR((state+action)(0), (state+action)(1), z_arm);
    // baxter position control to target
    myBaxter.modifyTarget(track, target);
    myBaxter.waitConv({track, align1, align2});
    arr next_state = ARR(track->y(0), track->y(1));
    return next_state;
}

void environment::track_marker_position_baxter(){
    ACCESSname(ar::AlvarMarkers, ar_pose_markers);
        double marker_pos_x = 0.0;
        double marker_pos_y = 0.0;
        ar_track_alvar_msgs::AlvarMarkers msg = ar_pose_markers.get();
        if (msg.markers.size() > 0)
        {
            for(int i=0; i<10; i++){
                ar_track_alvar_msgs::AlvarMarker marker = msg.markers[0];
                marker_pos_x += marker.pose.pose.position.x;
                marker_pos_y += marker.pose.pose.position.y;
            }
            marker_pos_x = marker_pos_x/10.;
            marker_pos_y = marker_pos_y/10.;

        }
        if(marker_pos_x < state_space(0)){
            marker_pos_x = state_space(0);
        } else if (marker_pos_x > state_space(1)){
            marker_pos_x = state_space(1);
        }
        if(marker_pos_y < state_space(2)){
            marker_pos_y = state_space(2);
        } else if (marker_pos_y > state_space(3)){
            marker_pos_y = state_space(3);
        }
        marker_position = ARR(marker_pos_x, marker_pos_y);
}



bool environment::in_table(arr state){
    if(abs(state(0)) > 5 || abs(state(1)) > 5){
        return false;
    }
    return true;
}

// only for simulation
void environment::set_random_positions() {
    marker_position(0) = mlr::rnd.uni() * 20 - 10;
    marker_position(1) = mlr::rnd.uni() * 20 - 10;
    arm_position(0) = mlr::rnd.uni() * 20 - 10;
    arm_position(1) = mlr::rnd.uni() * 20 - 10;
}


