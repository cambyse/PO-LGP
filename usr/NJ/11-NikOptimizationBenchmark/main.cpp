/*
 * main.cpp
 *
 *  Created on: Feb 22, 2011
 *      Author: nikolay
 */
#define MT_IMPLEMENTATION

#include <MT/soc.h>
#include <MT/util.h>
#include <NikUtilities.h>

//init in an awkward position and make movement by following init traj with IK
void problem1(){
	arr qhome = ARR(-1.32216, -0.731968 ,-1.725, -1.63269, -1.24208, 0.592419, -1.26379);
    ors::Vector target(-0.336507, -0.9, 0.95);
    setTargetAndQ(qhome,target);
    arr Clust = inerpolateDatabaseMotion(3,T+1);//a cluster traj that solves the problem
    arr q = IKTrajWithInit(Clust);
    setTargetAndQ(qhome,target);
	setKReachGoals();
    sys.totalCost(NULL, q, true);
    plotClear();
    plotLine(endeffTraj(q));
    sys.displayTrajectory(q,NULL,-1,STRING(" IK Init "));
    setTargetAndQ(qhome,target);
    q = planTrajectoryWithInit(q);
    sys.displayTrajectory(q,NULL,-1,STRING(" final "));
}

//same as problem,s 1, but using conditioning withing planner instead of IK
void problem2(){
	arr qhome = ARR(-1.32216, -0.731968 ,-1.725, -1.63269, -1.24208, 0.592419, -1.26379);
    ors::Vector target(-0.336507, -0.9, 0.95);
    setTargetAndQ(qhome,target);
    arr Clust = inerpolateDatabaseMotion(3,T+1);//a cluster traj that solves the problem
    setKReachGoals(Clust);
    arr q = planTrajectoryWithCondition(Clust);
    sys.displayTrajectory(q,NULL,-1,STRING(" final "));
}

int main(int argc,char **argv){
  MT::initCmdLine(argc,argv);
  init();
  int mode=MT::getParameter<int>("mode");
  switch(mode){
  case 1:  problem1();  break;
  case 2:  problem2();  break;
  default: NIY;
  }
  return 0;
}
