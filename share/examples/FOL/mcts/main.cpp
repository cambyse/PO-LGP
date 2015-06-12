#include <FOL/fol.h>
#include <Gui/graphview.h>
#include <MCTS/solver_marc.h>
#include <FOL/fol_mcts_world.h>

//===========================================================================

void testMonteCarlo(){
  Graph Gorig;
  FILE("boxes_new.kvg") >>Gorig;
  MT::rnd.seed(3);
  uint verbose=3;

  for(uint k=0;k<10;k++){
    Graph KB = Gorig;
    KB.checkConsistency();
    Node *Terminate_keyword = KB["Terminate"];
    Graph& state = KB.getNode("STATE")->graph();
    NodeL rules = KB.getNodes("Rule");
    NodeL constants = KB.getNodes("Constant");
    Graph& terminal = KB.getNode("terminal")->graph();

    for(uint h=0;h<100;h++){
      if(verbose>2) cout <<"****************** " <<k <<" MonteCarlo rollout step " <<h <<endl;

      if(verbose>2){ cout <<"*** state = "; state.write(cout, " "); cout <<endl; }

      bool forceWait=false, decideWait=false;
      if(MT::rnd.uni()<.8){ //normal rule decision
        //-- get all possible decisions
        MT::Array<std::pair<Node*, NodeL> > decisions; //tuples of rule and substitution
        for(Node* rule:rules){
          NodeL subs = getRuleSubstitutions(state, rule, constants, (verbose>4) );
          for(uint s=0;s<subs.d0;s++){
            decisions.append(std::pair<Node*, NodeL>(rule, subs[s]));
          }
        }

        if(verbose>2) cout <<"*** # possible decisions: " <<decisions.N <<endl;
        if(verbose>3) for(auto d:decisions){
          cout <<"rule " <<d.first->keys(1) <<" SUB "; listWrite(d.second, cout); cout <<endl;
        }

        if(!decisions.N){
          forceWait=true;
        }else{
          //-- pick a random decision
          uint deci = MT::rnd(decisions.N);
          std::pair<Node*, NodeL>& d = decisions(deci);
          if(verbose>2){ cout <<"*** decision = " <<deci <<':' <<d.first->keys(1) <<" SUB "; listWrite(d.second, cout); cout <<endl; }

          Node *effect = d.first->graph().last();
          if(verbose>2){ cout <<"*** applying" <<*effect <<" SUBS"; listWrite(d.second, cout); cout <<endl; }
          applyEffectLiterals(state, effect->graph(), d.second, &d.first->graph());
        }
      }else{
        decideWait=true;
      }

      if(forceWait || decideWait){
        if(verbose>2){ cout <<"*** WAIT decision " <<endl; }

        //-- find minimal wait time
        double w=1e10;
        for(Node *i:state){
          if(i->getValueType()==typeid(double)){
            double wi = *i->getValue<double>();
            if(w>wi) w=wi;
          }
        }

        if(w==1e10){
          if(verbose>2) cout <<"*** not applicable" <<endl;
          if(forceWait){ cout <<"*** STUCK - NO FEASIBLE SOLUTION FOUND" <<endl;  break; }
        }else{
          //-- subtract w from all times and collect all activities with minimal wait time
          NodeL activities;
          for(Node *i:state){
            if(i->getValueType()==typeid(double)){
              double &wi = *i->getValue<double>();
              wi -= w;
              if(fabs(wi)<1e-10) activities.append(i);
            }
          }

          //-- for all these activities call the terminate operator
          for(Node *act:activities){
            Node *predicate = act->parents(0);
            Node *rule = KB.getChild(Terminate_keyword, predicate);
            if(!rule) HALT("No termination rule for '" <<*predicate <<"'");
            Node *effect = rule->graph().last();
            NodeL vars = getSymbolsOfScope(rule->graph());
            NodeL subs(vars.N); subs.setZero();
            CHECK(vars.N==act->parents.N-1,"");
            for(uint i=0;i<vars.N;i++) subs(i) = act->parents(i+1);

            if(verbose>2){ cout <<"*** applying" <<*effect <<" SUBS"; listWrite(subs, cout); cout <<endl; }
            applyEffectLiterals(state, effect->graph(), subs, &rule->graph());
          }
        }
      }

      //-- test the terminal state
      if(allFactsHaveEqualsInScope(state, terminal)){
        if(verbose>0) cout <<"************* TERMINAL STATE FOUND (h=" <<h <<") ************" <<endl;
        if(verbose>1){ cout <<"*** FINAL STATE = "; state.write(cout, " "); cout <<endl; }
        break;
      }
    }
  }
}

//===========================================================================


void testMCTS(){
  FOL_World world("boxes_new.kvg");
  MCTS mcts(world);
  world.verbose=0;
  mcts.verbose=1;
  mcts.beta=100.;
//  Graph G = mcts.getGraph();
//  GraphView gv(G);
  for(uint k=0;k<100;k++){
    cout <<"******************************************** ROLLOUT " <<k <<endl;
    mcts.addRollout(100);
//    G = mcts.getGraph();
//    if(!(k%1)) gv.update();
  }
  cout <<mcts.Qfunction() <<endl;
  mcts.reportQ(cout); cout <<endl;
  cout <<"MCTS #nodes=" <<mcts.Nnodes() <<endl;

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

int main(int argn, char** argv){
  rnd.clockSeed();
  srand(rnd());
//  testMonteCarlo();
  testMCTS();
}
