#include <Core/util.h>
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
  virtual void configure2(const char *name, Graph& specs, ors::KinematicWorld& world){
    map = new DefaultTaskMap(gazeAtTMT, world, "endeffHead", ors::Vector(0.,0.,-1.), "testObject");
    task = new CtrlTask(name, *map, specs);
  }
};

struct GraspTask : TaskCtrlActivity {
  virtual void configure2(const char *name, Graph& specs, ors::KinematicWorld& world){
    map = new DefaultTaskMap(posTMT, world, "endeffR", NoVector, "testObject");
    task = new CtrlTask(name, *map, specs);
  }
};


int main(int argc, char** argv){
  registerActivity<GazeTask>("GazeTask");
  registerActivity<GraspTask>("GraspTask");

  ActionSwigInterface S;
  S.createNewSymbol("r_gripper_joint");
  S.createNewSymbol("gaze");

  /// look at object -- three ways to create the activity
#if 1
  S.setFact("(GazeTask){ PD=[.5, .9, .5, 10.] }");
  S.waitForCondition("(conv GazeTask)");
  S.setFact("(GazeTask)!, (conv GazeTask)!");
#elif 0
  S.setFact("(FollowReferenceActivity endeffHead testObject){ type=gazeAt, PD=[.5, .9, .5, 10.] }");
  S.waitForCondition("(conv FollowReferenceActivity endeffHead testObject)");
  S.setFact("(FollowReferenceActivity endeffHead testObject)!, (conv FollowReferenceActivity endeffHead testObject)!");
#else
  newActivity<FollowReferenceActivity>(S.getState(), {"FollowReferenceActivity", "endeffHead", "testObject"}, { NO(type, mlr::String("gazeAt")), NO(PD, ARR(.5, .9, .5, 10.))});
  S.waitForCondition("(conv FollowReferenceActivity endeffHead testObject)");
  S.setFact("(FollowReferenceActivity endeffHead testObject)!, (conv FollowReferenceActivity endeffHead testObject)!");
#endif

  /// open gripper
  S.setFact("(FollowReferenceActivity r_gripper_joint){ type=qItself, target=[0.08], PD=[1.5, .9, .5, 10.]}");
  S.waitForCondition("(conv FollowReferenceActivity r_gripper_joint)");
  S.setFact(" (FollowReferenceActivity r_gripper_joint)!, (conv FollowReferenceActivity r_gripper_joint)!");

  /// bring gripper in pre grasp position
  S.setFact("(GraspTask){ PD=[1., .9, .5, 10.] }");
  S.setFact("(FollowReferenceActivity endeffR){ type=vecAlign, vec1=[0,1,0], ref2=base_footprint, vec2=[0,0,1], target=[0], PD=[.5, .9, .5, 10.]}");
  S.waitForCondition("(conv FollowReferenceActivity endeffR) (conv GraspTask)");
  S.setFact("(GraspTask)!, (conv GraspTask)!, (FollowReferenceActivity endeffR)! ,(conv FollowReferenceActivity endeffR)!");

  /// close gripper
  S.setFact("(FollowReferenceActivity r_gripper_joint){ type=qItself, target=[0.02], PD=[.5, .9, .5, 10.]}");
  S.waitForCondition("(conv FollowReferenceActivity r_gripper_joint)");
  S.setFact(" (FollowReferenceActivity r_gripper_joint)!, (conv FollowReferenceActivity r_gripper_joint)!");

  return 0;
}
