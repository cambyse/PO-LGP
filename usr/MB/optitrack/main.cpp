#include <Core/util.h>
#include "../../share/teaching/RoboticsPractical/interface/myBaxter.h"
#include "RosCom/subscribeOptitrack.h"
#include <Control/taskController.h>

// =================================================================================================

int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);

  ACCESSname(geometry_msgs::TransformStamped, opti_drone)
  ACCESSname(geometry_msgs::TransformStamped, opti_body)

  {
    MyBaxter baxter;
    mlr::wait(2.);

    auto home = baxter.task(GRAPH(" map=qItself PD=[.5, 1., .2, 10.]"));
    baxter.modifyTarget(home, baxter.q0());
    baxter.waitConv({home});
    baxter.stop({home});

    cout << "Getting current pos. " << endl;
    CtrlTask* currentPositionTask = baxter.task(
                        "rel",
                        new DefaultTaskMap(posTMT, baxter.getKinematicWorld(), "endeffL", NoVector, "base_footprint"), //map
                        1., 1, 1., 1.);
    currentPositionTask->map.phi(currentPositionTask->y, NoArr, baxter.getKinematicWorld()); //get the current value

    arr pos = currentPositionTask->y;

    CtrlTask* currentElbow = baxter.task(
                        "rel2",
                        new DefaultTaskMap(posTMT, baxter.getKinematicWorld(), "elbowL", NoVector, "base_footprint"), //map
                        1., 1, 1., 1.);
    currentElbow->map.phi(currentElbow->y, NoArr, baxter.getKinematicWorld()); //get the current value

    arr pos2 = currentElbow->y;

    geometry_msgs::TransformStamped msg;
    geometry_msgs::Transform trans;

    while(1)
    {
      msg = opti_drone.get();
      trans = msg.transform;
      if (trans.translation.x != 0)
        break;
    }
    ors::Transformation origin = conv_transform2transformation(trans);

    while(1)
    {
      msg = opti_body.get();
      trans = msg.transform;
      if (trans.translation.x != 0)
        break;
    }
    ors::Transformation originE = conv_transform2transformation(trans);
    baxter.stop({currentElbow});

    while(1)
    {
      msg = opti_drone.get();
      trans = msg.transform;
      ors::Transformation new_tf = conv_transform2transformation(trans);
      new_tf.appendInvTransformation(origin);
      double scale = 5;
      arr new_target = pos + ARR(new_tf.pos.x / scale, new_tf.pos.y / scale, new_tf.pos.z / scale);
      cout << new_target << endl;
      baxter.modifyTarget(currentPositionTask, new_target);


      ors::Vector xVec = new_tf.rot.getX(); //rot.getMatrix() * ors::Vector(1, 0, 0);
      ors::Vector yVec = new_tf.rot.getY(); //rot.getMatrix() * ors::Vector(0, 1, 0);
      ors::Vector zVec = new_tf.rot.getZ(); //rot.getMatrix() * ors::Vector(0, 0, 1);


      CtrlTask* alignX = baxter.task(GRAPH(STRING("map=vecAlign ref1=endeffL ref2=base_footprint vec1=[1 0 0] vec2=["
                                             << xVec.x << ' ' << xVec.y << ' ' << xVec.z
                                             << "] target=[1] PD=[1., 1., .8, 1.] prec=[100]")));

      CtrlTask* alignY = baxter.task(GRAPH(STRING("map=vecAlign ref1=endeffL ref2=base_footprint vec1=[0 1 0] vec2=["
                                             << yVec.x << ' ' << yVec.y << ' ' << yVec.z
                                             << "] target=[1] PD=[1., 1., .8, 1.] prec=[100]")));

      CtrlTask* alignZ = baxter.task(GRAPH(STRING("map=vecAlign ref1=endeffL ref2=base_footprint vec1=[0 0 1] vec2=["
                                             << zVec.x << ' ' << zVec.y << ' ' << zVec.z
                                             << "] target=[1] PD=[1., 1., .8, 1.] prec=[100]")));

      opti_drone.waitForNextRevision();
      baxter.stop({alignX, alignY, alignZ});

//      opti_body.waitForNextRevision();
//      msg = opti_body.get();
//      trans = msg.transform;
//      new_tf = conv_transform2transformation(trans);
//      new_tf.appendInvTransformation(originE);
//      new_target = pos2 + ARR(new_tf.pos.x, new_tf.pos.y, new_tf.pos.z);
//      cout << new_target << endl;
      //baxter.modifyTarget(currentElbow, new_target);

    }
  }
  moduleShutdown().waitForValueGreaterThan(0);


  cout <<"bye bye" <<endl;
  return 0;
}

