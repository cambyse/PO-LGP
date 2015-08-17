#include <FOL/fol.h>
#include <MCTS/solver_marc.h>
#include <MCTS/solver_PlainMC.h>
#include <FOL/fol_mcts_world.h>


//===========================================================================

void TEST(MCTS){
  FOL_World world(FILE("boxes_new.kvg"));
  MCTS mcts(world);   //WARNING: this version only works for deterministic worlds!!
  world.verbose=0;
  world.verbFil=0;
  mcts.verbose=1;
  mcts.beta=100.;
//  Graph G = mcts.getGraph();
//  GraphView gv(G);
  for(uint k=0;k<100;k++){
    cout <<"******************************************** ROLLOUT " <<k <<endl;
    mcts.addRollout(100);
//    cout <<mcts.getGraph();
//    mcts.reportDecisions(cout);
//    world.reset_state();
//    world.get_actions();

//    G = mcts.getGraph();
//    if(!(k%1)) gv.update();
  }
  cout <<mcts.Qfunction() <<endl;
  mcts.reportQ(cout); cout <<endl;
  cout <<"MCTS #nodes=" <<mcts.Nnodes() <<endl;

  return;

  //--- generate some playouts of the optimal (non optimistic) policy
  world.fil.close();
  MT::open(world.fil,"z.demos");
  mcts.beta=1.;
  world.verbose=0;
  for(uint k=0;k<10;k++){
    cout <<"******************************************** ROLLOUT " <<k <<endl;
    mcts.addRollout(100);
  }
}

//===========================================================================

void TEST(MC){
  FOL_World world(FILE("boxes_new.kvg"));
  PlainMC mc(world);
  world.verbose=0;
  world.verbFil=0;
  mc.verbose=0;

  for(uint s=0;s<100;s++){
    cout <<"******************************************** STEP " <<s <<endl;
    mc.reset();
    for(uint k=0;k<100;k++) mc.addRollout(100);
    mc.report();
    auto a = mc.getBestAction();
    cout <<"******** ACTION " <<*a <<endl;
    world.reset_state();
    world.transition(a);
    if(world.is_terminal_state()) break;
    world.make_current_state_default();
  }
}

//===========================================================================

void TEST(FOL_World){
  FOL_World world(FILE("boxes_new.kvg"));

  auto actions = world.get_actions();
  for(auto& a:actions){ cout <<"DECISION: " <<*a <<endl; }

  for(uint k=0;k<10;k++){
    auto res=world.transition_randomly();
    cout <<"RND TRANSITION: obs=" <<*res.first <<" r=" <<res.second <<endl;
  }

  world.get_actions();

  world.make_current_state_default();

  world.reset_state();
  world.get_actions();
}

//===========================================================================

void TEST(Determinism){

  for(uint k=0;k<100;k++){
    FOL_World world(FILE("boxes_new.kvg"));

    //-- generate a random rollout
    world.reset_state();
    MT::Array<FOL_World::Handle> actions;
    MT::Array<FOL_World::Handle> observations;
    MT::Array<double> rewards;
    MT::Array<MT::String> states;
    for(;;){
      auto A = world.get_actions();
      FOL_World::Handle action = A[rnd()%A.size()];
      auto res = world.transition(action);
      actions.append(action);
      observations.append(res.first);
      rewards.append(res.second);
      states.append(STRING(*world.state));
      if(world.is_terminal_state()) break;
    }

    world.fil.close();
    MT::open(world.fil,"z.FOL_World2");

    //-- now repeat and check: same observations => same rollout
    world.reset_state();
    uint t=0;
    for(;;t++){
      world.make_current_state_default();
      std::pair<FOL_World::Handle, double> res;
      for(;;){ //repeat stochastic transition until you get the same observation
        res = world.transition(actions(t));
        if(*res.first==*observations(t)) break; //observations match... move on
        world.reset_state();
      }
      CHECK_EQ(*observations(t), *res.first, "");
      CHECK_EQ(rewards(t), res.second, "");
      CHECK_EQ(states(t), STRING(*world.state), "");
      if(world.is_terminal_state()) break;
    }
    CHECK_EQ(t+1, actions.N,"");
  }
}

//===========================================================================

int main(int argn, char** argv){
//    rnd.clockSeed();
//  srand(timenow)

//  testMCTS();
  testMC();
//  testFOL_World();
//  testDeterminism();
}
