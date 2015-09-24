#include <Core/util.h>
#include <pr2/rosutil.h>
#include <pr2/roscom.h>
#include <pr2/rosmacro.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <Core/array-vector.h>
#include <System/engine.h>
#include <Perception/perception.h>
#include <Perception/depth_packing.h>
#include <Perception/kinect2pointCloud.h>
#include <Gui/mesh.h>
#include <Actions/swig.h>
#include <tf/transform_listener.h>
#include <Motion/taskMaps.h>
#include <Actions/taskCtrlActivities.h>
#include <Motion/feedbackControl.h>


struct GazeTask : TaskCtrlActivity {
  virtual void configure2(const char *name, Graph& specs, ors::KinematicWorld& world);
  virtual void step2(double dt){}
  virtual bool isConv();
};
void GazeTask::configure2(const char *name, Graph& specs, ors::KinematicWorld& world) {
  ors::Vector v = world.getShapeByName("testObject")->X.pos;
  map = new DefaultTaskMap(gazeAtTMT,world,"endeffHead",ors::Vector(0.,0.,-1.),NULL,v);
  task = new CtrlTask(name, *map,specs);
  stopTolerance=1e-2; //TODO: overwrite from specs
}
bool GazeTask::isConv(){
  return ((task->y_ref.nd==1 && task->y.N==task->y_ref.N
           && maxDiff(task->y, task->y_ref)<stopTolerance
           && maxDiff(task->v, task->v_ref)<stopTolerance));
}

struct GraspTask : TaskCtrlActivity {
  virtual void configure2(const char *name, Graph& specs, ors::KinematicWorld& world);
  virtual void step2(double dt){}
  virtual bool isConv();
};
void GraspTask::configure2(const char *name, Graph& specs, ors::KinematicWorld& world) {
  map = new DefaultTaskMap(posTMT,world,"endeffR",NoVector,"testObject");
  task = new CtrlTask(name, *map,specs);
  stopTolerance=1e-2; //TODO: overwrite from specs
}
bool GraspTask::isConv(){
  return ((task->y_ref.nd==1 && task->y.N==task->y_ref.N
           && maxDiff(task->y, task->y_ref)<stopTolerance
           && maxDiff(task->v, task->v_ref)<stopTolerance));
}

int main(int argc, char** argv){
  registerActivity<GazeTask>("GazeTask");
  registerActivity<GraspTask>("GraspTask");

  ActionSwigInterface S;

  /// look at object
  S.setFact("(GazeTask){PD=[.5, .9, .5, 10.]}");
  S.waitForCondition("(conv GazeTask)");
  S.setFact("(GazeTask)!, (conv GazeTask)!");

  /// open gripper
  S.setFact("(FollowReferenceActivity){ type=qItself,ref1=r_gripper_joint,target=[0.08], PD=[1.5, .9, .5, 10.]}");
  S.waitForCondition("(conv FollowReferenceActivity)");
  S.setFact(" (FollowReferenceActivity)!, (conv FollowReferenceActivity)!");

  /// bring gripper in pre grasp position
  S.setFact("(GraspTask){PD=[1., .9, .5, 10.]}");
  S.setFact("(FollowReferenceActivity endeffR){ type=vecAlign,vec1=[0,1,0], ref2=base_footprint,vec2=[0,0,1] target=[0], PD=[.5, .9, .5, 10.]}");
  S.waitForCondition("(conv GraspTask)");
  S.waitForCondition("(conv FollowReferenceActivity endeffR)");
  S.setFact("(GraspTask)!, (conv GraspTask)!, (FollowReferenceActivity endeffR)! ,(conv FollowReferenceActivity endeffR)!");

  /// close gripper
  S.setFact("(FollowReferenceActivity){ type=qItself,ref1=r_gripper_joint,target=[0.02], PD=[.5, .9, .5, 10.]}");
  S.waitForCondition("(conv FollowReferenceActivity)");
  S.setFact(" (FollowReferenceActivity)!, (conv FollowReferenceActivity)!");

  return 0;
}
