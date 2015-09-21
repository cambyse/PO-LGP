#include <Ors/ors.h>
#include <Gui/opengl.h>
#include <Ors/ors.h>
#include <Optim/optimization.h>

#include <LGP/LGP.h>

//===========================================================================

void ijcaiExperiment(){
  orsDrawJoints=false;
  orsDrawAlpha=1.;

  ofstream fil("data/samples.dat");
  fil <<"experiment #objects MCTS_time lev1_time f_bestEnd lev2_time lev3_time" <<endl;

  for(uint k=0;k<50;k++){
    TowerProblem towers; //generates a randomize towers problem

    ManipulationTree_Node *root = new ManipulationTree_Node(towers.world, towers.symbols);

    ors::KinematicWorld world_display;
    double MCTS_time=0., lev1_time=0., lev2_time=0.;

    uint repeat;
    for(repeat=0;repeat<200;repeat++){
      //generate a new 'symbolic node' (here by fully unrolling)
      ManipulationTree_Node *node = new ManipulationTree_Node(root);
      MT::timerRead(true);
      runMonteCarlo(node->symbols);
      MCTS_time += MT::timerRead(true);

      //generate the respective effective pose problem
      EffectivePoseProblem effectivePoseProblem(node->effKinematics, node->symbols, 0);
      node->effPoseCost = effectivePoseProblem.optimize(node->effPose);
      double rx = towers.reward(node->effKinematics, node->symbols);
      lev1_time += MT::timerRead(true);
      cout <<"fx=" <<node->effPoseCost <<endl;
      cout <<"reward=" <<rx <<endl;
      node->effPoseReward = rx-node->effPoseCost;
      world_display=node->effKinematics;
      world_display.gl().update();
    }
    cout <<"BEST:" <<endl;
    ManipulationTree_Node *best=NULL;
    for(auto* n:root->children) if(!best || n->effPoseReward>best->effPoseReward) best=n;
    best->effKinematics >>FILE("z.world_best.kvg");
    best->symbols >>FILE("z.symbols_best.kvg");

    world_display=best->effKinematics;
    world_display.gl().update();

    MT::timerRead(true);
    PathProblem pathProblem(towers.world, best->effKinematics, best->symbols, 20, 0);
    best->pathCosts = pathProblem.optimize(best->path);
    lev2_time = MT::timerRead(true);
    cout <<"f_path=" <<best->pathCosts <<endl;

    fil <<k <<' ' <<towers.nObjects <<' ' <<MCTS_time/repeat <<' ' <<lev1_time/repeat <<' ' <<best->effPoseReward <<' ' <<lev2_time <<endl;
  }
}

//===========================================================================

void newMethod(){
  TowerProblem_new towers; //generates a randomize towers problem

  //ManipulationTree T(towers.world, towers.symbols);

  auto actions = towers.fol_root.get_actions();
  uint c=0;
  for(auto& a:actions){ cout <<"(" <<c++ <<") DECISION: " <<*a <<endl; }

//  for(;;){
//    T.addRollout(); //uses a tree policy to walk to a leaf, expands, r
//    T.optimEffPose(T.getRndNode());
//    T.optimPath(T.getRndNode());
//  }

}

//===========================================================================

int main(int argc,char **argv){
  MT::initCmdLine(argc, argv);
//  rnd.clockSeed();
  rnd.seed(MT::getParameter<int>("seed",0));

  ijcaiExperiment();
  //  newMethod();

  return 0;
}
