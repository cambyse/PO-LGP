#include <RosCom/roscom.h>
#include <Control/TaskControllerModule.h>
#include <Hardware/gamepad/gamepad.h>
#include <Ors/orsviewer.h>
#include <RosCom/baxter.h>

#include <RosCom/subscribeTabletop.h>
#include <RosCom/subscribeAlvarMarkers.h>
#include <RosCom/perceptionCollection.h>
#include <Algo/perceptionFilter.h>

// =================================================================================================
int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);

  rosCheckInit("minimalPositionControl");

  {

    Access_typed<sensor_msgs::JointState> jointState(NULL, "jointState");

    TaskControllerModule tcm("baxter");
//    OrsViewer view;
    OrsPoseViewer ctrlView({"ctrl_q_real", "ctrl_q_ref"}, tcm.realWorld);
    SendPositionCommandsToBaxter spctb;
    if(mlr::getParameter<bool>("useRos")){
      new Subscriber<sensor_msgs::JointState> ("/robot/joint_states", jointState);
    }

    SubscribeTabletop tabletop_subscriber;

    Collector data_collector;

    Filter myFilter;

    ACCESSname(visualization_msgs::MarkerArray, objects)
    Subscriber<visualization_msgs::MarkerArray> sub_objects("/tabletop/tracked_clusters", objects);

    RosCom_Spinner spinner; //the spinner MUST come last: otherwise, during closing of all, it is closed before others that need messages

    threadOpenModules(true);

    Access_typed<arr> ctrl_q_ref(NULL, "ctrl_q_ref");
    ctrl_q_ref.waitForRevisionGreaterThan(10); //wait a few steps (e.g. to ensure sync with real bot)

    //-- create three tasks
    CtrlTask position("endeffL", //name
                  new DefaultTaskMap(posTMT, tcm.modelWorld.get()(), "endeffL", NoVector, "base_footprint"), //map
                  1., .8, 1., 1.); //time-scale, damping-ratio, maxVel, maxAcc
    position.map.phi(position.y, NoArr, tcm.modelWorld.get()()); //get the current value
    position.y_ref = position.y + ARR(0.2, 0.5, 0.3); //set a target

    CtrlTask position1("endeffR", //name
                  new DefaultTaskMap(posTMT, tcm.modelWorld.get()(), "endeffR", NoVector, "base_footprint"), //map
                  1., .8, 1., 1.); //time-scale, damping-ratio, maxVel, maxAcc
    position1.map.phi(position1.y, NoArr, tcm.modelWorld.get()()); //get the current value
    position1.y_ref = position1.y + ARR(0.2, -0.5, 0.3); //set a target

    //-- tell the controller to take care of them
    tcm.ctrlTasks.set() = { &position, &position1}; //, &align2 };


    mlr::wait(5.);

    tcm.ctrlTasks.set()->clear();

    /*
     * Time to find and poke the interesting object.
     */

    visualization_msgs::MarkerArray clusters = objects.get();

    // While there are no objects found, keep looking.
    while (clusters.markers.size() == 0 )
    {
      clusters = objects.get();
      ROS_INFO("No objects found.");
      mlr::wait(1);
    }

    double min_dist = 99999;
    int min_id = -1;

    for (int i = 0; i < clusters.markers.size(); i++)
    {
      visualization_msgs::Marker marker = clusters.markers[i];
      double dist = length(ARR(marker.points[0].x, marker.points[0].y, marker.points[0].z));
      if (dist < min_dist)
      {
        min_dist = dist;
        min_id = i;
      }
    }

    if (min_id == -1)
      exit(0);


    // Get point of interest
    visualization_msgs::Marker first_cluster = clusters.markers[min_id];


    // Convert that point into a position relative to the base_footprint.
    tf::Vector3 pointToPoke(first_cluster.points[0].x, first_cluster.points[0].y, first_cluster.points[0].z);

    tf::TransformListener listener;
    tf::StampedTransform baseTransform;
    try{
      listener.waitForTransform("/reference/base", first_cluster.header.frame_id, ros::Time(0), ros::Duration(1.0));
      listener.lookupTransform("/reference/base", first_cluster.header.frame_id, ros::Time(0), baseTransform);
    }
    catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        exit(0);
    }

    std::cout << "Point to poke, relative to the camera: " << pointToPoke.getX();
    std::cout << ' ' << pointToPoke.getY() << ' ' << pointToPoke.getZ() << std::endl;

    // Convert into base frame, for clarity
    pointToPoke = baseTransform * pointToPoke;

    std::cout << "Point to poke, relative to base: " << pointToPoke.getX();
    std::cout << ' ' << pointToPoke.getY() << ' ' << pointToPoke.getZ() << std::endl;

    /*
     * Now we know where we want to poke. Let's poke!
     *  Using FOL, poke the object.
     *  This is currently equivalent to setting the position of the end effector and the point as equivalent.
     */

    CtrlTask position2("endeffR", //name
                  new DefaultTaskMap(posTMT, tcm.modelWorld.get()(), "endeffR", NoVector, "base_footprint"), //map
                  1., .8, 1., 1.); //time-scale, damping-ratio, maxVel, maxAcc
    position2.map.phi(position2.y, NoArr, tcm.modelWorld.get()()); //get the current value
    position2.y_ref = ARR(pointToPoke.getX(), pointToPoke.getY(), pointToPoke.getZ()+1); //set a target

    //-- tell the controller to take care of them
    tcm.ctrlTasks.set() = { &position2 };

    mlr::wait(10);
    //-- create a homing with
    CtrlTask homing("homing",
                  new TaskMap_qItself(),
                  .5, 1., .2, 10.);
    homing.y_ref = tcm.q0;

    tcm.ctrlTasks.set() = { &homing };

    mlr::wait(10);
//    moduleShutdown().waitForValueGreaterThan(0);

    threadCloseModules();
  }

  cout <<"bye bye" <<endl;
  return 0;
}
