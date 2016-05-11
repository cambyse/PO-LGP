    //-- create three tasks
//    CtrlTask position("endeffL", //name
//                  new DefaultTaskMap(posTMT, tcm.modelWorld.get()(), "endeffL", NoVector, "base_footprint"), //map
//                  1., .8, 1., 1.); //time-scale, damping-ratio, maxVel, maxAcc
//    position.map.phi(position.y, NoArr, tcm.modelWorld.get()()); //get the current value
//    position.y_ref = position.y + ARR(.3, 0., 0.);; //set a target

//    CtrlTask align1("align",
//                    new DefaultTaskMap(vecAlignTMT, tcm.modelWorld.get()(), "ellbowR", Vector_z, NULL, Vector_y),
//                    1., 1., 1., 1.);
//    align1.y_ref = {-1.};

//    CtrlTask align2("align",
//                    new DefaultTaskMap(vecAlignTMT, tcm.modelWorld.get()(), "ellbowL", Vector_z, "ellbowR", Vector_z),
//                    1., 1., 1., 1.);
//    align2.y_ref = {-1.};

//    //-- tell the controller to take care of them
//    tcm.ctrlTasks.set() = { /*&position,*/ /*&align1,*/ &align2 };


//        CtrlTask position("endeffL", //name
//                      new DefaultTaskMap(posTMT, tcm.modelWorld.get()(), "endeffL", NoVector, "base_footprint"), //map
//                      1., .8, 1., 1.); //time-scale, damping-ratio, maxVel, maxAcc
//        position.map.phi(position.y, NoArr, tcm.modelWorld.get()()); //get the current value
//        position.y_ref = position.y + ARR(.5, 0., 0.2); //set a target

//        arr posCen=ARR(.0,.0,.0);
//        posCen=position.y_ref;

//        mlr::wait(2.5);
//        tcm.ctrlTasks.set() = { &position };
//        mlr::wait(1.5);


//        float pi=3.1415;
//        float step = pi/20;
//        float radius = 0.3;
//        float i;

//        for(i=0;i<=2*pi;i+=step)
//        {
//            position.y_ref = posCen + ARR(.0, radius * cos(i),  radius * sin(i));
//            tcm.ctrlTasks.set() = { &position };
//           if(i==0)
//            mlr::wait(.9);
//           else
//               mlr::wait(.3);
//            cout << i<<endl;
//        }

         //-- tell the controller to take care of them
        // tcm.ctrlTasks.set() = { &position, };

    mlr::wait(2.);

    CtrlTask align("align",
        new DefaultTaskMap(vecAlignTMT, tcm.modelWorld.get()(), "endeffL", Vector_x, "shape1", Vector_z),
                        1., .8, 1., 1.);
        align.y_ref = {-1.};

    CtrlTask align1("align1",
         new DefaultTaskMap(vecAlignTMT, tcm.modelWorld.get()(), "endeffL", Vector_y, "shape1", Vector_y),
                            1., .8, 1., 1.);
         align1.y_ref = { 1.};
    CtrlTask align2("align2",
         new DefaultTaskMap(vecAlignTMT, tcm.modelWorld.get()(), "endeffL", Vector_z, "shape1", Vector_x),
                                 1., .8, 1., 1.);
          align2.y_ref = { 1.};




          CtrlTask grasp("grasp",
               new TaskMap_qItself( tcm.modelWorld.get()(), "l_gripper_l_finger_joint"),
                                       .3, .8, 1., 1.);
            //grasp.y_ref= {0.1};


     CtrlTask position("endeffL", //name
         new DefaultTaskMap(posTMT, tcm.modelWorld.get()(), "endeffL", NoVector, "shape1"), //map
                          1., .8, 1., 1.); //time-scale, damping-ratio, maxVel, maxAcc
        position.map.phi(position.y, NoArr, tcm.modelWorld.get()()); //get the current value

    position.y_ref = ARR(0., 0., 0.4); //set a target
    tcm.ctrlTasks.set() = { &position, &align ,&align1, &align2};
    mlr::wait(.1);
    grasp.y_ref= {0.1};
    tcm.ctrlTasks.set() = { &grasp};

    mlr::wait(3.);

    position.y_ref = ARR(0., 0., 0.05); //set a target
    tcm.ctrlTasks.set() = { &position, &align, &align1, &align2};
    mlr::wait(.1);
    grasp.y_ref= {0.0};
    tcm.ctrlTasks.set() = { &grasp};

    mlr::wait(3.);

    position.y_ref = ARR(0.0, 0.0, 0.3); //set a target
    tcm.ctrlTasks.set() = { &position, &align, &align1, &align2};

    mlr::wait(3.);

    position.y_ref = ARR(0.3, -0.3, 0.3); //set a target
    tcm.ctrlTasks.set() = { &position, &align, &align1, &align2};

    mlr::wait(3.);

    position.y_ref = ARR(0.3, -0.3, 0.1); //set a target
    tcm.ctrlTasks.set() = { &position, &align, &align1, &align2};
    mlr::wait(.1);
    grasp.y_ref= {0.1};
    tcm.ctrlTasks.set() = { &grasp};

    mlr::wait(3.);
