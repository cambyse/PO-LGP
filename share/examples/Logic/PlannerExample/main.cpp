#include <Logic/fol.h>
#include <MCTS/solver_marc.h>
#include <MCTS/solver_PlainMC.h>
#include <MCTS/solver_AStar.h>
#include <Logic/fol_mcts_world.h>


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
  mlr::open(world.fil,"z.demos");
  mcts.beta=1.;
  world.verbose=0;
  for(uint k=0;k<10;k++){
    cout <<"******************************************** ROLLOUT " <<k <<endl;
    mcts.addRollout(100);
  }
}

//===========================================================================

void TEST(MC){
  mlr::String file=mlr::getParameter<mlr::String>("file","");
  if(file=="") file="boxes_new.g";
  FOL_World fol(FILE(file));

  PlainMC mc(fol);
  fol.verbose=0;
  fol.verbFil=0;
  mc.verbose=0;

#if 0
  Graph& dataset = fol.KB.newSubgraph({"dataset"}, {})->value;
  for(auto* s:getSymbolsOfScope(fol.KB)) dataset.newNode<bool>(s->keys, {}, true);
  for(auto* r:fol.KB.getNodes("DecisionRule"))   dataset.newNode<bool>(r->keys, {}, true);
#endif

  for(uint s=0;s<100;s++){
    cout <<"******************************************** STEP " <<s <<endl;
    mc.reset();
    for(uint k=0;k<100;k++) mc.addRollout(100);
    mc.report();

    //save all estimated returns
#if 0
    Graph &data = dataset.newSubgraph({}, {})->value;
    data.newSubgraph({"state"}, {}, *fol.state);
    for(uint i=0;i<mc.D.N;i++){
      const FOL_World::Decision *d = std::dynamic_pointer_cast<const FOL_World::Decision>(mc.A(i)).get();
      data.newNode<bool>({"action"}, d->getTuple(), true);
      data.newNode<double>({"return"}, {}, mc.D(i).X.first());
    }
#endif

    auto a = mc.getBestAction();
    cout <<"******** ACTION " <<*a <<endl;
    fol.reset_state();
    fol.transition(a);
    if(fol.is_terminal_state()) break;
    fol.make_current_state_default();

    break;
  }

  FILE("z.fol") <<fol;

#if 0
  FILE("z.data").getOs() <<dataset <<endl;
#endif
}

//===========================================================================

void TEST(FOL_World){
  mlr::String file=mlr::getParameter<mlr::String>("file");
  if(!file.N) file="boxes_new.g";
  FOL_World world(FILE(file));

  auto actions = world.get_actions();
  for(auto& a:actions){ cout <<"DECISION: " <<*a <<endl; }

  for(uint k=0;k<10;k++){
    auto res=world.transition_randomly();
    cout <<"RND TRANSITION: obs=" <<*res.observation <<" r=" <<res.reward <<endl;
  }

  world.get_actions();

  //-- test write_state/set_state
  mlr::String str;
  world.write_state(str);
  cout <<"\nBEFORE rndAction:" <<endl;  world.write_state(cout);
  world.transition_randomly();
  cout <<"\nAFTER rndAction:" <<endl;  world.write_state(cout);
  world.set_state(str);
  cout <<"\nAFTER set_state:" <<endl;  world.write_state(cout);


//  world.make_current_state_default();
//  world.reset_state();
//  world.get_actions();
}

//===========================================================================

void TEST(Determinism){
  for(uint k=0;k<100;k++){
    FOL_World world(FILE("boxes_new.kvg"));

    //-- generate a random rollout
    world.reset_state();
    mlr::Array<FOL_World::Handle> actions;
    mlr::Array<FOL_World::Handle> observations;
    mlr::Array<double> rewards;
    mlr::Array<mlr::String> states;
    for(;;){
      auto A = world.get_actions();
      FOL_World::Handle action = A[rnd()%A.size()];
      auto res = world.transition(action);
      actions.append(action);
      observations.append(res.observation);
      rewards.append(res.reward);
      states.append(STRING(*world.state));
      if(world.is_terminal_state()) break;
    }

    world.fil.close();
    mlr::open(world.fil,"z.FOL_World2");

    //-- now repeat and check: same observations => same rollout
    world.reset_state();
    uint t=0;
    for(;;t++){
      world.make_current_state_default();
      FOL_World::TransitionReturn res;
      for(;;){ //repeat stochastic transition until you get the same observation
        res = world.transition(actions(t));
        if(*res.observation==*observations(t)) break; //observations match... move on
        world.reset_state();
      }
      CHECK_EQ(*observations(t), *res.observation, "");
      CHECK_EQ(rewards(t), res.reward, "");
      CHECK_EQ(states(t), STRING(*world.state), "");
      if(world.is_terminal_state()) break;
    }
    CHECK_EQ(t+1, actions.N,"");
  }
}

//===========================================================================

void testAStar(){
  mlr::String file=mlr::getParameter<mlr::String>("file","");
  if(file=="") file="boxes_new.g";
  FOL_World fol(FILE(file));

  AStar A(fol);

  for(uint k=0;k<10;k++){
    A.step();
    Graph g = A.root->getGraph();
    g.displayDot();
    mlr::wait(.5);
  }

}

//===========================================================================

int main(int argn, char** argv){
  //  rnd.clockSeed();
  //srand(rnd());

//  testMCTS();
//  testMC();
//  testFOL_World();
//  testDeterminism();
  testAStar();
}
