Include = '../../../data/keywords.g'
Include = 'model.kvg'

KOMO{
  T = 100
  duration = 5
}

(MinSumOfSqr qItself){ order=2 time=[0 1] }
#(EqualZero posDiff endeff target){ vec1=[.15 0 0] time=[1 1] scale=100 }
#(MinSumOfSqr pos endeff){ order=1 time=[1 1] scale=100 }
#(EqualZero vecAlign endeff target){ vec1=[1 0 0] vec2=[1 0 0] time=[1 1] target=[1] }
#(LowerEqualZero collisionIneq){ margin=0.05 scale=.1 }

(MakeJoint rigidZero graspRef obj1){ timeOfApplication=.49 }
(MakeJoint delete table obj1){ timeOfApplication=.49 }

#  ors::KinematicSwitch *op1 = new ors::KinematicSwitch();
#  op1->symbol = ors::KinematicSwitch::addJointZero;
#  op1->jointType = ors::JT_fixed;
#  op1->timeOfApplication = pickTime;
#  op1->fromId = G.getShapeByName("graspRef")->index;
#  op1->toId = G.getShapeByName("obj1")->index;
#  MP.switches.append(op1);

#  ors::KinematicSwitch *op2 = new ors::KinematicSwitch();
#  op2->symbol = ors::KinematicSwitch::deleteJoint;
#  op2->timeOfApplication = pickTime;
#  op2->fromId = G.getShapeByName("table")->index;
#  op2->toId = G.getShapeByName("obj1")->index;
#  MP.switches.append(op2);

#  //-- setup new motion problem
#  ors::Shape *grasp = G.getShapeByName("graspRef");
#  ors::Shape *obj = G.getShapeByName("obj1");
#  ors::Shape *tar = G.getShapeByName("target");

(MinSumOfSqr posDiff graspRef obj1){ time=[0.49 0.5] scale=1e3 }
#  t = MP.addTask("pos_mid",  new DefaultTaskMap(posDiffTMT, grasp->index, NoVector, obj->index, NoVector) );
#  t->setCostSpecs(pickTime-1, pickTime, {0.}, 1e3);

(MinSumOfSqr quatDiff graspRef obj1){ time=[0.49 0.5] scale=1e3 }
#  t = MP.addTask("quat", new DefaultTaskMap(quatDiffTMT, grasp->index, NoVector, obj->index) );
#  t->setCostSpecs(pickTime-1, pickTime, {0.}, 1e3);

(MinSumOfSqr qItself){ order=1 time=[0.49 0.5] scale=1e1 }
#  t = MP.addTask("q_vel_mid", new TaskMap_qItself());
#  t->map.order=1; //make this a velocity variable!
#  t->setCostSpecs(pickTime-1, pickTime, {0.}, 1e1);

(MinSumOfSqr pos obj1){ order=1 scale=1e1 time=[0.53 0.55] target=[0 0 .5] }
#  t = MP.addTask("lift", new DefaultTaskMap(posTMT, obj->index));
#  t->map.order=1; //make this a velocity variable!
#  t->setCostSpecs(pickTime+3, pickTime+5, {0.,0.,.5}, 1e1);

#  //target
(MinSumOfSqr posDiff obj1 target){ time=[1 1] scale=1e3 }
#  t = MP.addTask("pos", new DefaultTaskMap(posDiffTMT, obj->index, NoVector, tar->index) );
#  t->setCostSpecs(MP.T, MP.T, {0.}, 1e3);

(MinSumOfSqr qItself){ order=1 time=[1 1] scale=1e1 }
#  t = MP.addTask("q_vel", new TaskMap_qItself());
#  t->map.order=1; //make this a velocity variable!
#  t->setCostSpecs(MP.T, MP.T, {0.}, 1e1);

(MinSumOfSqr quatDiff obj1 target){ time=[1 1] target=[1 0 0 0] scale=1e3 }
#  t = MP.addTask("quat", new DefaultTaskMap(quatDiffTMT, grasp->index, NoVector, tar->index) );
#  t->setCostSpecs(MP.T, MP.T, ARR(1.,0.,0.,0.), 1e3);

(MinSumOfSqr qItself graspJoint){ order=1 time=[0.51 .99] scale=1e3 }
#  // zero grasp joint motion during holding
#  if(MP.z0.N==0){
#    ors::Joint *j_grasp = MP.world.getJointByName("graspJoint");
#    arr M(j_grasp->qDim(), MP.world.getJointStateDimension());
#    M.setZero();
#    for(uint i=0;i<j_grasp->qDim();i++) M(i,j_grasp->qIndex+i)=1.;
#    t = MP.addTask("graspJoint", new TaskMap_qItself(M));
#    t->map.order=1;
#    t->prec.resize(MP.T+1).setZero();
#    for(uint time=pickTime+1;time<placeTime;time++) t->prec(time)=1e3;
#  }
