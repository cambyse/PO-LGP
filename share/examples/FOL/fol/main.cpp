#include <FOL/fol.h>
#include <Gui/graphview.h>

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

  NodeL consts = G.getItems("Constant");
  Node *s = G["STATE"];
  Node *r = G["cruiseto"]->graph()["precond"];
  NodeL vars = G["cruiseto"]->graph().getItems("Var");

  cout <<"symbols = " <<G.getItems("Symbol") <<endl;

  cout <<"state = " <<*s <<"\nrule=" <<*r <<endl;


  G.checkConsistency();
  Node *sub = new Node_typed<Graph>(G, new Graph, true);
  sub->graph().isItemOfParentKvg = sub;
  G.checkConsistency();
  new Node_typed<bool>(sub->graph(), {}, {s, consts(0)}, NULL, false);
  G.checkConsistency();
  new Node_typed<bool>(sub->graph(), {}, {s, consts(2)}, NULL, false);
  G.checkConsistency();
}

//===========================================================================

void testFolFwdChaining(){
  Graph G;

  FILE("fol.kvg") >>G;

//  cout <<"\n-----------------\n" <<G <<"\n-----------------\n" <<endl;

  NodeL consts = G.getItems("Constant");
  NodeL rules = G.getItems("Rule");
  Graph& state = G.getItem("STATE")->graph();

  cout <<"INIT STATE = " <<GRAPH(state) <<endl;

  Node *query=G["Query"]->graph()(0);
  forwardChaining_FOL(G, query);
}

//===========================================================================

void testFolDisplay(){
  Graph G;
  FILE("fol.kvg") >>G;

  GraphView view(G);
  view.watch();

}

//===========================================================================

void testFolSubstitution(){
  Graph KB;

//  FILE("boxes.kvg") >>G;
  FILE("substTest.kvg") >>KB;

  NodeL rules = KB.getItems("Rule");
  NodeL constants = KB.getItems("Constant");
  Graph& state = KB.getItem("STATE")->graph();

  for(Node* rule:rules){
    cout <<"*** RULE: " <<*rule <<endl;
    cout <<  "Substitutions:" <<endl;
    NodeL subs = getRuleSubstitutions(state, rule, constants, true);
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

void testMonteCarlo(){
  Graph Gorig;
  FILE("boxes.kvg") >>Gorig;
  MT::rnd.seed(3);
  uint verbose=3;

  for(uint k=0;k<10;k++){
    Graph KB = Gorig;
    KB.checkConsistency();
    Node *Terminate_keyword = KB["Terminate"];
    Graph& state = KB.getItem("STATE")->graph();
    NodeL rules = KB.getItems("Rule");
    NodeL constants = KB.getItems("Constant");
    Graph& terminal = KB.getItem("terminal")->graph();

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
          cout <<"rule " <<d.first->keys(1) <<" SUBS "; listWrite(d.second, cout); cout <<endl;
        }

        if(!decisions.N){
          forceWait=true;
        }else{
          //-- pick a random decision
          uint deci = MT::rnd(decisions.N);
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


int main(int argn, char** argv){
//  testFolLoadFile();
//  testPolFwdChaining();
//  testFolFwdChaining();
//  testFolDisplay();
  testFolSubstitution();
//  testMonteCarlo();
  cout <<"BYE BYE" <<endl;
}
