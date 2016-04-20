#include <RosCom/roscom.h>
#include <RosCom/spinner.h>
#include <Control/TaskControllerModule.h>
#include <Hardware/gamepad/gamepad.h>
#include <Ors/orsviewer.h>
#include <RosCom/baxter.h>

#include <Actions/PlayFunnySound.h>

#include <RosCom/subscribeTabletop.h>
#include <RosCom/subscribeAlvarMarkers.h>
#include <RosCom/perceptionCollection.h>
#include <RosCom/perceptionFilter.h>
#include <RosCom/filterObject.h>

// =================================================================================================
int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);

  rosCheckInit("minimalPositionControl");

  {

    Access_typed<sensor_msgs::JointState> jointState(NULL, "jointState");

    TaskControllerModule tcm("baxter");
//    OrsViewer view;
    OrsPoseViewer ctrlView({"ctrl_q_real", "ctrl_q_ref"}, tcm.realWorld);

    if(mlr::getParameter<bool>("useRos")){
      new SendPositionCommandsToBaxter();
      new Subscriber<sensor_msgs::JointState> ("/robot/joint_states", jointState);
    }

    SubscribeTabletop tabletop_subscriber;

    Collector data_collector;

    Filter myFilter;

    ACCESSname(FilterObjects, object_database)

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

    while (1)
    {
      FilterObjects filter_objects = object_database.get();
      FilterObjects clusters;
      for (FilterObject* fo : filter_objects)
      {
          if (fo->type == FilterObject::FilterObjectType::cluster)
          {
            clusters.append(fo);
          }
      }
      if (clusters.N == 0)
      {
        std::cout << "No clusters found" << std::endl;
        mlr::wait(1.);
        continue;
      }

      double min_dist = 99999;
      int min_id = -1;

      for (uint i = 0; i < clusters.N; i++)
      {
        double dist = length(dynamic_cast<Cluster*>(clusters(i))->mean);
        if (dist < min_dist)
        {
          min_dist = dist;
          min_id = i;
        }
      }

      if (min_id == -1)
        exit(0);


//      // Get point of interest
//      Cluster* first_cluster = dynamic_cast<Cluster*>(clusters(min_id));

//      // Convert that point into a position relative to the base_footprint.
//      tf::Vector3 pointToPoke(first_cluster->mean(0), first_cluster->mean(1), first_cluster->mean(2));

//      tf::TransformListener listener;
//      tf::StampedTransform baseTransform;
//      try{
//        listener.waitForTransform("/reference/base", first_cluster->frame_id, ros::Time(0), ros::Duration(1.0));
//        listener.lookupTransform("/reference/base", first_cluster->frame_id, ros::Time(0), baseTransform);
//      }
//      catch (tf::TransformException &ex) {
//          ROS_ERROR("%s",ex.what());
//          ros::Duration(1.0).sleep();
//          exit(0);
//      }

//      std::cout << "Point to poke, relative to the camera: " << pointToPoke.getX();
//      std::cout << ' ' << pointToPoke.getY() << ' ' << pointToPoke.getZ() << std::endl;

//      // Convert into base frame, for clarity
//      pointToPoke = baseTransform * pointToPoke;

//      std::cout << "Point to poke, relative to base: " << pointToPoke.getX();
//      std::cout << ' ' << pointToPoke.getY() << ' ' << pointToPoke.getZ() << std::endl;

      Cluster* first_cluster = dynamic_cast<Cluster*>(clusters(min_id));

      ors::Vector pointToPoke = first_cluster->transform * first_cluster->mean;

      /*
       * Now we know where we want to poke. Let's poke!
       *  Using FOL, poke the object.
       *  This is currently equivalent to setting the position of the end effector and the point as equivalent.
       */

      CtrlTask position2("endeffR", //name
                    new DefaultTaskMap(posTMT, tcm.modelWorld.get()(), "endeffR", NoVector, "base_footprint"), //map
                    1., .8, 1., 1.); //time-scale, damping-ratio, maxVel, maxAcc
      position2.map.phi(position2.y, NoArr, tcm.modelWorld.get()()); //get the current value
      position2.y_ref = ARR(pointToPoke.x, pointToPoke.y, pointToPoke.z+1); //set a target

      //-- tell the controller to take care of them
      tcm.ctrlTasks.set() = { &position2 };

      mlr::wait(5);

      PlayFunnySoundActivity* pfs = new PlayFunnySoundActivity;
      pfs->open();
      mlr::wait(2);
      delete pfs;

      tcm.ctrlTasks.set() = { &position1 };

      mlr::wait(10);
    }
//    moduleShutdown().waitForValueGreaterThan(0);

    threadCloseModules();
  }

  cout <<"bye bye" <<endl;
  return 0;
}
