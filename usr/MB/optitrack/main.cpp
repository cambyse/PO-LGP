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

    mlr::Body* body = baxter.getModelWorld().getBodyByName("optitrackbody_0");
//    mlr::Body* body = baxter.getModelWorld().getBodyByName("alvar_10");
    while (!body)
    {
      mlr::wait(0.1);
      body = baxter.getModelWorld().getBodyByName("optitrackbody_0");
//      body = baxter.getModelWorld().getBodyByName("alvar_10");
    }
    mlr::Transformation initial_optitrack_tf = body->X;

    CtrlTask* currentHandPositionTask = baxter.task(
                        "hand_task",
                        new TaskMap_Default(posTMT, baxter.getModelWorld(), "endeffR", NoVector, "base_footprint"), //map
                        1, 1, .2, 1.);
    currentHandPositionTask->map.phi(currentHandPositionTask->y, NoArr, baxter.getModelWorld()); //get the current value

//    arr initial_end_eff_pos = currentHandPositionTask->y;
//    cout << "Initial: " << initial_end_eff_pos << endl;

    mlr::Transformation initial = baxter.getModelWorld().getShapeByName("endeffR")->X;

    mlr::Vector xVec_endeff = initial.rot.getX();
    mlr::Vector yVec_endeff = initial.rot.getY();
    mlr::Vector zVec_endeff = initial.rot.getZ();

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

      mlr::Transformation new_tf; new_tf.setDifference(initial_optitrack_tf, body->X);


//      (body->X.pos - initial_optitrack_tf.pos)
      //      mlr::Vector new_pos = new_tf * initial.pos;
//      mlr::Vector new_pos = new_tf.pos + initial.pos;
      mlr::Vector new_pos = (body->X.pos - initial_optitrack_tf.pos) + initial.pos;


      mlr::Quaternion new_rot = (body->X.rot * -initial_optitrack_tf.rot) * initial.rot;


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

//    mlr::Body* hand = baxter.getModelWorld().getBodyByName(hand_body);
//    mlr::Body* forearm = baxter.getModelWorld().getBodyByName(forearm_body);
//    mlr::Body* elbow = baxter.getModelWorld().getBodyByName(elbow_body);

//    mlr::Transformation hand_tf = hand->X;
//    mlr::Transformation forearm_tf = forearm->X;
//    mlr::Transformation elbow_tf = elbow->X;

//    mlr::wait(2.);


//    mlr::Transformation hand_body_origin = hand_tf;
//    mlr::Transformation forearm_body_origin = forearm_tf;
//    mlr::Transformation elbow_body_origin = elbow_tf;


//    mlr::Shape* endeffr = baxter.getModelWorld().getShapeByName("endeffR");
//    mlr::Transformation endeffR_origin= endeffr->X;

//    mlr::Shape* wristr = baxter.getModelWorld().getShapeByName("wristR");
//    mlr::Transformation forearmR_origin= wristr->X;

//    mlr::Shape* elbowr = baxter.getModelWorld().getShapeByName("elbowR");
//    mlr::Transformation elbowR_origin= elbowr->X;


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

//      mlr::Transformation new_hand_tf = hand_tf;
//      new_hand_tf.appendInvTransformation(hand_body_origin); // new_hand_tf = new_hand_tf * hand_body_origin^-1

//      //========= new_hand_tf should be back in base frame
//      arr new_hand_target = pos + ARR(new_hand_tf.pos.x / scale, new_hand_tf.pos.y / scale, new_hand_tf.pos.z / scale);
//      baxter.modifyTarget(currentHandPositionTask, new_hand_target);
//      //baxter.modifyTarget(currentHandPositionTask, hand_pos);

//      new_hand_tf.appendTransformation(endeffR_origin);

////      mlr::Quaternion orig_rot = hand_origin.rot;
////      orig_rot.invert();
////      new_hand_tf = hand_tf;
////      new_hand_tf.addRelativeRotation(orig_rot);
////      new_hand_tf.addRelativeRotation(endeffR_origin.rot);

//      mlr::Vector xVec_endeff = new_hand_tf.rot.getX(); //rot.getMatrix() * mlr::Vector(1, 0, 0);
//      mlr::Vector yVec_endeff = new_hand_tf.rot.getY(); //rot.getMatrix() * mlr::Vector(0, 1, 0);
//      mlr::Vector zVec_endeff = new_hand_tf.rot.getZ(); //rot.getMatrix() * mlr::Vector(0, 0, 1);

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

//      mlr::Transformation new_forearm_tf = forearm_tf;
//      new_forearm_tf.appendInvTransformation(forearm_body_origin);
//      new_forearm_tf.appendTransformation(forearmR_origin);

//      mlr::Vector xVec_forearm = new_forearm_tf.rot.getX(); //rot.getMatrix() * mlr::Vector(1, 0, 0);
////      mlr::Vector yVec_forearm = new_forearm_tf.rot.getY(); //rot.getMatrix() * mlr::Vector(0, 1, 0);
////      mlr::Vector zVec_forearm = new_forearm_tf.rot.getZ(); //rot.getMatrix() * mlr::Vector(0, 0, 1);

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

//      mlr::Transformation new_elbow_tf = elbow_tf;
//      new_elbow_tf.appendInvTransformation(elbow_body_origin);
//      new_elbow_tf.appendTransformation(elbowR_origin);

////      mlr::Vector xVec_elbow = new_elbow_tf.rot.getX(); //rot.getMatrix() * mlr::Vector(1, 0, 0);
////      mlr::Vector yVec_elbow = new_elbow_tf.rot.getY(); //rot.getMatrix() * mlr::Vector(0, 1, 0);
//      mlr::Vector zVec_elbow = new_elbow_tf.rot.getZ(); //rot.getMatrix() * mlr::Vector(0, 0, 1);



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



