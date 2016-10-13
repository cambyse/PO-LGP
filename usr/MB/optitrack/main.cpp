#include <Core/util.h>
#include "../../share/teaching/RoboticsPractical/interface/myBaxter.h"
#include "RosCom/subscribeOptitrack.h"
#include <Control/taskController.h>

// =================================================================================================


int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);

//  ACCESSname(geometry_msgs::TransformStamped, opti_drone)
//  ACCESSname(geometry_msgs::TransformStamped, opti_body)


    MyBaxter baxter;

    mlr::wait(2.);


//    arr my_q(22);
//    my_q = ARR(0, 0, 0, 0, 0, -0.0122718, 1.02777, -0.0571408, 0.0191748);
//    my_q.append(ARR(-1.01818, -0.0180243, -1.11405, -0.0483204, 1.95774, 0.0414175, 0.671884, 0.0368155, 1.0362));
//    my_q.append(ARR(-0.0145728, -0.495092, 0, 0.02083)); //apparently I can't initialize a vector with more than 9 elements


    auto home = baxter.task(GRAPH(" map=qItself PD=[.5, 1., .2, 5.]"));
    baxter.modifyTarget(home, baxter.q0());
    baxter.waitConv({home});
    baxter.stop({home});
    cout << "Homed." << endl;

    mlr::wait(5);

    ors::Body* body = baxter.getModelWorld().getBodyByName("optitrackbody_0");
//    ors::Body* body = baxter.getModelWorld().getBodyByName("alvar_10");
    while (!body)
    {
      mlr::wait(0.1);
      body = baxter.getModelWorld().getBodyByName("optitrackbody_0");
//      body = baxter.getModelWorld().getBodyByName("alvar_10");
    }
    ors::Transformation initial_optitrack_tf = body->X;

    CtrlTask* currentHandPositionTask = baxter.task(
                        "hand_task",
                        new TaskMap_Default(posTMT, baxter.getModelWorld(), "endeffR", NoVector, "base_footprint"), //map
                        1, 1, .2, 1.);
    currentHandPositionTask->map.phi(currentHandPositionTask->y, NoArr, baxter.getModelWorld()); //get the current value

//    arr initial_end_eff_pos = currentHandPositionTask->y;
//    cout << "Initial: " << initial_end_eff_pos << endl;

    ors::Transformation initial = baxter.getModelWorld().getShapeByName("endeffR")->X;

    ors::Vector xVec_endeff = initial.rot.getX();
    ors::Vector yVec_endeff = initial.rot.getY();
    ors::Vector zVec_endeff = initial.rot.getZ();

    CtrlTask* alignX_endeff = baxter.task(GRAPH(STRING("map=vecAlign ref1=endeffR ref2=base_footprint vec1=[1 0 0] vec2=["
                                           << xVec_endeff.x << ' ' << xVec_endeff.y << ' ' << xVec_endeff.z
                                           << "] target=[1] PD=[1., 1., .2, 5] prec=[100]")));

    CtrlTask* alignY_endeff = baxter.task(GRAPH(STRING("map=vecAlign ref1=endeffR ref2=base_footprint vec1=[0 1 0] vec2=["
                                           << yVec_endeff.x << ' ' << yVec_endeff.y << ' ' << yVec_endeff.z
                                                       << "] target=[1] PD=[1., 1., .2, 5] prec=[100]")));

    CtrlTask* alignZ_endeff = baxter.task(GRAPH(STRING("map=vecAlign ref1=endeffR ref2=base_footprint vec1=[0 0 1] vec2=["
                                           << zVec_endeff.x << ' ' << zVec_endeff.y << ' ' << zVec_endeff.z
                                                       << "] target=[1] PD=[1., 1., .2, 5] prec=[100]")));

    CtrlTask* limits = baxter.task("limits", new LimitsConstraint());
    limits->setGainsAsNatural(1, 1);



    while (1){
      body = baxter.getModelWorld().getBodyByName("optitrackbody_0");
//      body = baxter.getModelWorld().getBodyByName("alvar_10");

      if (!body)
        continue;

      // I want: new_tf = (body->X * initial_optitrack_tf^-1) * initial_endeff;
      baxter.stop({alignX_endeff, alignY_endeff, alignZ_endeff});

      ors::Transformation new_tf; new_tf.setDifference(initial_optitrack_tf, body->X);


//      (body->X.pos - initial_optitrack_tf.pos)
      //      ors::Vector new_pos = new_tf * initial.pos;
//      ors::Vector new_pos = new_tf.pos + initial.pos;
      ors::Vector new_pos = (body->X.pos - initial_optitrack_tf.pos) + initial.pos;


      ors::Quaternion new_rot = (body->X.rot * -initial_optitrack_tf.rot) * initial.rot;


      baxter.modifyTarget(currentHandPositionTask, new_pos.getArr());

//      baxter.stop({alignX_endeff, alignY_endeff, alignZ_endeff});

//      xVec_endeff = new_tf.rot.getX();
//      yVec_endeff = new_tf.rot.getY();
//      zVec_endeff = new_tf.rot.getZ();

      xVec_endeff = new_rot.getX();
      yVec_endeff = new_rot.getY();
      zVec_endeff = new_rot.getZ();

      cout << new_pos << endl;
//      cout << xVec_endeff << endl;
//      cout << yVec_endeff << endl;
//      cout << zVec_endeff << endl;

      alignX_endeff = baxter.task(GRAPH(STRING("map=vecAlign ref1=endeffR ref2=base_footprint vec1=[1 0 0] vec2=["
                                             << xVec_endeff.x << ' ' << xVec_endeff.y << ' ' << xVec_endeff.z
                                               << "] target=[1] PD=[1., 1., .2, 5] prec=[100]")));

      alignY_endeff = baxter.task(GRAPH(STRING("map=vecAlign ref1=endeffR ref2=base_footprint vec1=[0 1 0] vec2=["
                                             << yVec_endeff.x << ' ' << yVec_endeff.y << ' ' << yVec_endeff.z
                                               << "] target=[1] PD=[1., 1., .2, 5] prec=[100]")));

      alignZ_endeff = baxter.task(GRAPH(STRING("map=vecAlign ref1=endeffR ref2=base_footprint vec1=[0 0 1] vec2=["
                                             << zVec_endeff.x << ' ' << zVec_endeff.y << ' ' << zVec_endeff.z
                                               << "] target=[1] PD=[1., 1., .2, 5] prec=[100]")));

      mlr::wait(.05);
    }

//    // create shapes
//    cout<<"create shapes"<<endl;
//    char hand_body[20]="optitrackbody_0";
//    char forearm_body[20]="optitrackbody_1";
//    char elbow_body[20]="optitrackbody_2";
//    //char object[20]="optitrackmarker_2";

//    ors::Body* hand = baxter.getModelWorld().getBodyByName(hand_body);
//    ors::Body* forearm = baxter.getModelWorld().getBodyByName(forearm_body);
//    ors::Body* elbow = baxter.getModelWorld().getBodyByName(elbow_body);

//    ors::Transformation hand_tf = hand->X;
//    ors::Transformation forearm_tf = forearm->X;
//    ors::Transformation elbow_tf = elbow->X;

//    mlr::wait(2.);


//    ors::Transformation hand_body_origin = hand_tf;
//    ors::Transformation forearm_body_origin = forearm_tf;
//    ors::Transformation elbow_body_origin = elbow_tf;


//    ors::Shape* endeffr = baxter.getModelWorld().getShapeByName("endeffR");
//    ors::Transformation endeffR_origin= endeffr->X;

//    ors::Shape* wristr = baxter.getModelWorld().getShapeByName("wristR");
//    ors::Transformation forearmR_origin= wristr->X;

//    ors::Shape* elbowr = baxter.getModelWorld().getShapeByName("elbowR");
//    ors::Transformation elbowR_origin= elbowr->X;


//    mlr::wait(1.);

//        cout << "Getting current pos. " << endl;
//        CtrlTask* currentHandPositionTask = baxter.task(
//                            "rel",
//                            new DefaultTaskMap(posTMT, baxter.getModelWorld(), "endeffR", NoVector, "base_footprint"), //map
//                            1., 1, 1., 1.);
//        currentHandPositionTask->map.phi(currentHandPositionTask->y, NoArr, baxter.getModelWorld()); //get the current value

//        arr pos = currentHandPositionTask->y;
//        baxter.modifyTarget(currentHandPositionTask, pos);


//    while(1)
//    {

//      double scale = 1;

////  //---------------------------------------ENDEFF

//      hand = baxter.getModelWorld().getBodyByName(hand_body);
//      hand_tf = hand->X;

//      ors::Transformation new_hand_tf = hand_tf;
//      new_hand_tf.appendInvTransformation(hand_body_origin); // new_hand_tf = new_hand_tf * hand_body_origin^-1

//      //========= new_hand_tf should be back in base frame
//      arr new_hand_target = pos + ARR(new_hand_tf.pos.x / scale, new_hand_tf.pos.y / scale, new_hand_tf.pos.z / scale);
//      baxter.modifyTarget(currentHandPositionTask, new_hand_target);
//      //baxter.modifyTarget(currentHandPositionTask, hand_pos);

//      new_hand_tf.appendTransformation(endeffR_origin);

////      ors::Quaternion orig_rot = hand_origin.rot;
////      orig_rot.invert();
////      new_hand_tf = hand_tf;
////      new_hand_tf.addRelativeRotation(orig_rot);
////      new_hand_tf.addRelativeRotation(endeffR_origin.rot);

//      ors::Vector xVec_endeff = new_hand_tf.rot.getX(); //rot.getMatrix() * ors::Vector(1, 0, 0);
//      ors::Vector yVec_endeff = new_hand_tf.rot.getY(); //rot.getMatrix() * ors::Vector(0, 1, 0);
//      ors::Vector zVec_endeff = new_hand_tf.rot.getZ(); //rot.getMatrix() * ors::Vector(0, 0, 1);

//      CtrlTask* alignX_endeff = baxter.task(GRAPH(STRING("map=vecAlign ref1=endeffR ref2=base_footprint vec1=[1 0 0] vec2=["
//                                             << xVec_endeff.x << ' ' << xVec_endeff.y << ' ' << xVec_endeff.z
//                                             << "] target=[1] PD=[1., 1., .8, 1.] prec=[1000]")));

//      CtrlTask* alignY_endeff = baxter.task(GRAPH(STRING("map=vecAlign ref1=endeffR ref2=base_footprint vec1=[0 1 0] vec2=["
//                                             << yVec_endeff.x << ' ' << yVec_endeff.y << ' ' << yVec_endeff.z
//                                             << "] target=[1] PD=[1., 1., .8, 1.] prec=[1000]")));

//      CtrlTask* alignZ_endeff = baxter.task(GRAPH(STRING("map=vecAlign ref1=endeffR ref2=base_footprint vec1=[0 0 1] vec2=["
//                                             << zVec_endeff.x << ' ' << zVec_endeff.y << ' ' << zVec_endeff.z
//                                             << "] target=[1] PD=[1., 1., .8, 1.] prec=[1000]")));


//      mlr::wait(0.05);

//      //baxter.stop({alignX_endeff, alignY_endeff, alignZ_endeff});

////-------------------------------------------------FOREARM

//      forearm = baxter.getModelWorld().getBodyByName(forearm_body);
//      forearm_tf = forearm->X;

//      ors::Transformation new_forearm_tf = forearm_tf;
//      new_forearm_tf.appendInvTransformation(forearm_body_origin);
//      new_forearm_tf.appendTransformation(forearmR_origin);

//      ors::Vector xVec_forearm = new_forearm_tf.rot.getX(); //rot.getMatrix() * ors::Vector(1, 0, 0);
////      ors::Vector yVec_forearm = new_forearm_tf.rot.getY(); //rot.getMatrix() * ors::Vector(0, 1, 0);
////      ors::Vector zVec_forearm = new_forearm_tf.rot.getZ(); //rot.getMatrix() * ors::Vector(0, 0, 1);

//      CtrlTask* alignX_forearm = baxter.task(GRAPH(STRING("map=vecAlign ref1=wristR ref2=base_footprint vec1=[1 0 0] vec2=["
//                                           << xVec_forearm.x << ' ' << xVec_forearm.y << ' ' << xVec_forearm.z
//                                           << "] target=[1] PD=[1., 1., .8, 1.] prec=[1000]")));

////      CtrlTask* alignY_forearm = baxter.task(GRAPH(STRING("map=vecAlign ref1=wristR ref2=base_footprint vec1=[0 1 0] vec2=["
////                                           << yVec_forearm.x << ' ' << yVec_forearm.y << ' ' << yVec_forearm.z
////                                           << "] target=[1] PD=[1., 1., .8, 1.] prec=[300]")));

////      CtrlTask* alignZ_forearm = baxter.task(GRAPH(STRING("map=vecAlign ref1=wristR ref2=base_footprint vec1=[0 0 1] vec2=["
////                                           << zVec_forearm.x << ' ' << zVec_forearm.y << ' ' << zVec_forearm.z
////                                           << "] target=[1] PD=[1., 1., .8, 1.] prec=[300]")));

//      mlr::wait(0.05);

////      baxter.stop({alignX_endeff, alignY_endeff, alignZ_endeff});

////      baxter.stop({alignX_forearm, alignY_forearm, alignZ_forearm});
////      baxter.stop({alignX_forearm});


//      //---------------------------ELBOW

//      elbow = baxter.getModelWorld().getBodyByName(elbow_body);
//      elbow_tf = elbow->X;

//      ors::Transformation new_elbow_tf = elbow_tf;
//      new_elbow_tf.appendInvTransformation(elbow_body_origin);
//      new_elbow_tf.appendTransformation(elbowR_origin);

////      ors::Vector xVec_elbow = new_elbow_tf.rot.getX(); //rot.getMatrix() * ors::Vector(1, 0, 0);
////      ors::Vector yVec_elbow = new_elbow_tf.rot.getY(); //rot.getMatrix() * ors::Vector(0, 1, 0);
//      ors::Vector zVec_elbow = new_elbow_tf.rot.getZ(); //rot.getMatrix() * ors::Vector(0, 0, 1);



////      CtrlTask* alignX_elbow = baxter.task(GRAPH(STRING("map=vecAlign ref1=elbowR ref2=base_footprint vec1=[1 0 0] vec2=["
////                                             << xVec_elbow.x << ' ' << xVec_elbow.y << ' ' << xVec_elbow.z
////                                             << "] target=[1] PD=[1., 1., .8, 1.] prec=[1000]")));

////      CtrlTask* alignY_elbow = baxter.task(GRAPH(STRING("map=vecAlign ref1=elbowR ref2=base_footprint vec1=[0 1 0] vec2=["
////                                             << yVec_elbow.x << ' ' << yVec_elbow.y << ' ' << yVec_elbow.z
////                                             << "] target=[1] PD=[1., 1., .8, 1.] prec=[300]")));

//      CtrlTask* alignZ_elbow = baxter.task(GRAPH(STRING("map=vecAlign ref1=elbowR ref2=base_footprint vec1=[0 0 1] vec2=["
//                                             << zVec_elbow.x << ' ' << zVec_elbow.y << ' ' << zVec_elbow.z
//                                             << "] target=[1] PD=[1., 1., .8, 1.] prec=[1000]")));

//      mlr::wait(0.05);

//      baxter.stop({alignX_endeff, alignY_endeff, alignZ_endeff});

//      //baxter.stop({alignX_forearm, alignY_forearm, alignZ_forearm});
//      baxter.stop({alignX_forearm});

//      //baxter.stop({alignX_elbow, alignY_elbow, alignZ_elbow});
//      baxter.stop({alignZ_elbow});
//    }
//  }

   moduleShutdown().waitForValueGreaterThan(0);

  cout <<"bye bye" <<endl;
  return 0;
}


// q =   0 0 0 0 0 -0.0122718 1.02777 -0.0571408 0.0191748 -1.01818 -0.0180243 -1.11405 -0.0483204 1.95774 0.0414175 0.671884 0.0368155 1.0362 -0.0145728 -0.495092 0 0.02083



