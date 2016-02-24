#include <FOL/fol.h>
//#include <Gui/graphview.h>

//===========================================================================

void testPolFwdChaining(){
  Graph G;
  FILE("pol.kvg") >>G;

  cout <<G <<endl;

  cout <<"Q=" <<(forwardChaining_propositional(G, G["Q"]) ?"true":"false") <<endl;
}

//===========================================================================

void testFolLoadFile(){
  Graph G;
  G.checkConsistency();
  FILE("fol0.kvg") >>G;
  G.checkConsistency();

//  cout <<"\n-----------------\n" <<G <<"\n-----------------\n" <<endl;

  NodeL consts = G.getNodes("Constant");
  Node *s = G["STATE"];
  Node *r = G["cruiseto"]->graph()["precond"];
  NodeL vars = G["cruiseto"]->graph().getNodes("Var");

  cout <<"symbols = " <<G.getNodes("Symbol") <<endl;

  cout <<"state = " <<*s <<"\nrule=" <<*r <<endl;


  G.checkConsistency();
  Node *sub = new Node_typed<Graph*>(G, new Graph);
  sub->graph().isNodeOfParentGraph = sub;
  G.checkConsistency();
  new Node_typed<bool>(sub->graph(), {}, {s, consts(0)}, true);
  G.checkConsistency();
  new Node_typed<bool>(sub->graph(), {}, {s, consts(2)}, true);
  G.checkConsistency();
}

//===========================================================================

void testFolFwdChaining(){
  Graph G;

  FILE("fol.kvg") >>G;

  Graph& state = G.getNode("STATE")->graph();

  cout <<"INIT STATE = " <<state <<endl;

  Node *query=G["Query"]->graph()(0);
  forwardChaining_FOL(G, state, query);
//  cout <<"FINAL STATE = " <<state <<endl;
}

//===========================================================================

#if 0
void testFolDisplay(){
  Graph G;
  FILE("fol.kvg") >>G;

  GraphView view(G);
  view.watch();

}
#endif

//===========================================================================

void testFolSubstitution(){
  Graph KB;

//  FILE("boxes.kvg") >>G;
  FILE("substTest.g") >>KB;

  NodeL rules = KB.getNodes("Rule");
  NodeL constants = KB.getNodes("Constant");
  Graph& state = KB.getNode("STATE")->graph();

  for(Node* rule:rules){
    cout <<"*** RULE: " <<*rule <<endl;
    cout <<  "Substitutions:" <<endl;
    NodeL subs = getRuleSubstitutions2(state, rule, 2);
    cout <<"BEFORE state="; state.write(cout, " "); cout <<endl;
    for(uint s=0;s<subs.d0;s++){
      Node *effect = rule->graph().last();
      { cout <<"*** applying" <<*effect <<" SUBS"; listWrite(subs[s], cout); cout <<endl; }
      applyEffectLiterals(state, effect->graph(), subs[s], &rule->graph());
      cout <<"AFTER state="; state.write(cout, " "); cout <<endl;
    }
  }
}

//===========================================================================

void testFolFunction(){
  Graph KB(FILE("functionTest.g"));

  Graph& state = KB.getNode("STATE")->graph();
  Graph& func = KB.getNode("func")->graph();

  cout <<"f=" <<evaluateFunction(func, state, 3) <<endl;
}

//===========================================================================

void testMonteCarlo(){
  Graph Gorig;
  FILE("boxes.kvg") >>Gorig;
  mlr::rnd.seed(3);
  int verbose=2;

  for(uint k=0;k<10;k++){
    Graph KB = Gorig;
    KB.checkConsistency();
    Node *Terminate_keyword = KB["Terminate"];
    Graph& state = KB.getNode("STATE")->graph();
    NodeL rules = KB.getNodes("Rule");
    Graph& terminal = KB.getNode("terminal")->graph();

    for(uint h=0;h<100;h++){
      if(verbose>2) cout <<"****************** " <<k <<" MonteCarlo rollout step " <<h <<endl;

      if(verbose>2){ cout <<"*** state = "; state.write(cout, " "); cout <<endl; }

      bool forceWait=false, decideWait=false;
      if(mlr::rnd.uni()<.8){ //normal rule decision
        //-- get all possible decisions
        mlr::Array<std::pair<Node*, NodeL> > decisions; //tuples of rule and substitution
        for(Node* rule:rules){
          NodeL subs = getRuleSubstitutions2(state, rule, verbose-2 );
//          NodeL subs = getRuleSubstitutions2(state, rule, verbose-2 );
          for(uint s=0;s<subs.d0;s++){
            decisions.append(std::pair<Node*, NodeL>(rule, subs[s]));
          }
        }

        if(verbose>2) cout <<"*** # possible decisions: " <<decisions.N <<endl;
        if(verbose>3) for(auto d:decisions){
          cout <<"rule " <<d.first->keys(1) <<" SUBS "; listWrite(d.second, cout); cout <<endl;
        }

        if(!decisions.N){
          forceWait=true;
        }else{
          //-- pick a random decision
          uint deci = mlr::rnd(decisions.N);
          std::pair<Node*, NodeL>& d = decisions(deci);
          if(verbose>2){ cout <<"*** decision = " <<deci <<':' <<d.first->keys(1) <<" SUBS "; listWrite(d.second, cout); cout <<endl; }

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
            Node *rule = KB.getEdge(Terminate_keyword, predicate);
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


int main(int argn, char** argv){
  testFolLoadFile();
  testPolFwdChaining();
  testFolFwdChaining();
//  testFolDisplay();
  testFolSubstitution();
  testFolFunction();
//  testMonteCarlo();
  cout <<"BYE BYE" <<endl;
}
