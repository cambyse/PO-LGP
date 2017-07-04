#include <act_RosPublish.h>

#include <Kin/kin.h>
#ifdef MLR_ROS
#include <RosCom/roscom.h>
#endif

Act_RosPublish::Act_RosPublish(Roopi* r, VariableBase& var, double beatIntervalSec) : Act(r), var(var){
    LOG(0) <<"VARIABLE: " <<var.name <<" type:" <<NAME(var.type) <<endl;
    if(var.type==typeid(mlr::KinematicWorld)){
      VariableData<mlr::KinematicWorld> *v = dynamic_cast<VariableData<mlr::KinematicWorld>*>(&var);
      CHECK(v, "");
#ifdef MLR_ROS
      publisher = new PublisherConv<visualization_msgs::MarkerArray, mlr::KinematicWorld, conv_Kin2Markers>
          (STRING("/roopi/"<<var.name), Access<mlr::KinematicWorld>(NULL, var), beatIntervalSec);
#else
      LOG(0) <<" COMPILED WITHOUT ROS: NOT publishing topic '" <<STRING("/roopi/"<<var.name) <<"'";
#endif
    }
}

