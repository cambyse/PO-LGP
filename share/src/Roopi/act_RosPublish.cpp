#include <act_RosPublish.h>

#include <RosCom/roscom.h>
#include <Kin/kin.h>

Act_RosPublish::Act_RosPublish(Roopi* r, VariableBase& var, double beatIntervalSec) : Act(r), var(var){
    LOG(0) <<"VARIABLE: " <<var.name <<" type:" <<NAME(var.type) <<endl;
    if(var.type==typeid(mlr::KinematicWorld)){
      VariableData<mlr::KinematicWorld> *v = dynamic_cast<VariableData<mlr::KinematicWorld>*>(&var);
      CHECK(v, "");
      publisher = new PublisherConv<visualization_msgs::MarkerArray, mlr::KinematicWorld, conv_Kin2Markers>
          (STRING("/roopi/"<<var.name), Access<mlr::KinematicWorld>(NULL, var), beatIntervalSec);
    }
}
