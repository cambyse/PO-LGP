#include <MyBaxter.h>

// =================================================================================================
int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);


  {
    //-------------------

    MyBaxter bax;


    //threadOpenModules(true);
    bax.ctrl_q_ref.waitForRevisionGreaterThan(10); //wait a few steps (e.g. to ensure sync with real bot)

    //------------------

    mlr::wait(2.);

    //-----------Pick up/move object
//    CtrlTask *alignXZ = bax.align("alignXZ", "endeffL", Vector_x, "shape1", Vector_z,-1);
//    CtrlTask *alignYY= bax.align("alignYY", "endeffL", Vector_y, "shape1", Vector_y, 1);
//    CtrlTask *alignZX= bax.align("alignZX", "endeffL", Vector_z, "shape1", Vector_x, 1);


//    CtrlTask *position =  bax.goToPosition("endeffL","shape1", ARR(0., 0., 0.4), 3);

//    bax.openGripper();

//    bax.changePosition(position, ARR(0., 0., 0.05), 3.0 );

//    bax.closeGripper();

//    bax.changePosition(position, ARR(0.0, 0.0, 0.3), 3.0 );

//    bax.changePosition(position, ARR(0.3, -0.3, 0.3), 3.0 );

//    bax.changePosition(position, ARR(0.3, -0.3, 0.1), 3.0 );

//    bax.openGripper();


//  Identify cluster

    CtrlTask *pos_eeL =  bax.goToPosition("endeffL","base_footprint", ARR(0.6, 0.6, 1.3), 3);
    CtrlTask *pos_eeR =  bax.goToPosition("endeffR","base_footprint", ARR(0.6, -0.6, 1.3), 3);
    mlr::wait(2.);

    bax.reportPerceptionObjects();
    mlr::Vector closest_vec = bax.arPose();

    if (closest_vec.length() == 0)
      closest_vec.set(0.5, 0.6, 1);

    arr closest = ARR(closest_vec.x, closest_vec.y, closest_vec.z);

    //baxter.modify(posR, GRAPH("PD=[3., 1., 1., 1.]"));
    bax.changePosition(pos_eeR, closest + ARR(0.02, 0.05, .3));


    bax.changePosition(pos_eeR, closest + ARR(0.02, 0.05, -.07));


    mlr::wait(2.);

    bax.changePosition(pos_eeR, closest + ARR(0.02, 0.05, -.08));



    bax.reportJointState();
    bax.disablePosControl();

    for (uint i = 0; i < 100; i++)
    {
      //bax.publishTorque(ARR(0.,0.,0.,0.,0.,0.,0.));

      mlr::wait(0.01);
    }


    bax.enablePosControl();



//    moduleShutdown().waitForStatusGreaterThan(0);

    bax.homing();



//    moduleShutdown().waitForStatusGreaterThan(0);

    //threadCloseModules();
  }

  cout <<"bye bye" <<endl;
  return 0;
}
