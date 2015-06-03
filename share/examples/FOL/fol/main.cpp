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

  ItemL consts = G.getItems("Constant");
  Item *s = G["STATE"];
  Item *r = G["cruiseto"]->kvg()["precond"];
  ItemL vars = G["cruiseto"]->kvg().getItems("Var");

  cout <<"symbols = " <<G.getItems("Symbol") <<endl;

  cout <<"state = " <<*s <<"\nrule=" <<*r <<endl;


  G.checkConsistency();
  Item *sub = new Item_typed<Graph>(G, new Graph, true);
  sub->kvg().isItemOfParentKvg = sub;
  G.checkConsistency();
  new Item_typed<bool>(sub->kvg(), {}, {s, consts(0)}, NULL, false);
  G.checkConsistency();
  new Item_typed<bool>(sub->kvg(), {}, {s, consts(2)}, NULL, false);
  G.checkConsistency();
}

//===========================================================================

void testFolFwdChaining(){
  Graph G;

  FILE("fol.kvg") >>G;

  Graph& state = G.getItem("STATE")->kvg();

  cout <<"INIT STATE = " <<state <<endl;

  Item *query=G["Query"]->kvg()(0);
  forwardChaining_FOL(G, query);
//  cout <<"FINAL STATE = " <<state <<endl;
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

  ItemL rules = KB.getItems("Rule");
  ItemL constants = KB.getItems("Constant");
  Graph& state = KB.getItem("STATE")->kvg();

  for(Item* rule:rules){
    cout <<"*** RULE: " <<*rule <<endl;
    cout <<  "Substitutions:" <<endl;
    ItemL subs = getRuleSubstitutions(state, rule, constants, true);
    cout <<"BEFORE state="; state.write(cout, " "); cout <<endl;
    for(uint s=0;s<subs.d0;s++){
      Item *effect = rule->kvg().last();
      { cout <<"*** applying" <<*effect <<" SUBS"; listWrite(subs[s], cout); cout <<endl; }
      applyEffectLiterals(state, effect->kvg(), subs[s], &rule->kvg());
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
    Item *Terminate_keyword = KB["Terminate"];
    Graph& state = KB.getItem("STATE")->kvg();
    ItemL rules = KB.getItems("Rule");
    ItemL constants = KB.getItems("Constant");
    Graph& terminal = KB.getItem("terminal")->kvg();

    for(uint h=0;h<100;h++){
      if(verbose>2) cout <<"****************** " <<k <<" MonteCarlo rollout step " <<h <<endl;

      if(verbose>2){ cout <<"*** state = "; state.write(cout, " "); cout <<endl; }

      bool forceWait=false, decideWait=false;
      if(MT::rnd.uni()<.8){ //normal rule decision
        //-- get all possible decisions
        MT::Array<std::pair<Item*, ItemL> > decisions; //tuples of rule and substitution
        for(Item* rule:rules){
          ItemL subs = getRuleSubstitutions(state, rule, constants, (verbose>4) );
          for(uint s=0;s<subs.d0;s++){
            decisions.append(std::pair<Item*, ItemL>(rule, subs[s]));
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
          std::pair<Item*, ItemL>& d = decisions(deci);
          if(verbose>2){ cout <<"*** decision = " <<deci <<':' <<d.first->keys(1) <<" SUBS "; listWrite(d.second, cout); cout <<endl; }

          Item *effect = d.first->kvg().last();
          if(verbose>2){ cout <<"*** applying" <<*effect <<" SUBS"; listWrite(d.second, cout); cout <<endl; }
          applyEffectLiterals(state, effect->kvg(), d.second, &d.first->kvg());
        }
      }else{
        decideWait=true;
      }

      if(forceWait || decideWait){
        if(verbose>2){ cout <<"*** WAIT decision " <<endl; }

        //-- find minimal wait time
        double w=1e10;
        for(Item *i:state){
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
          ItemL activities;
          for(Item *i:state){
            if(i->getValueType()==typeid(double)){
              double &wi = *i->getValue<double>();
              wi -= w;
              if(fabs(wi)<1e-10) activities.append(i);
            }
          }

          //-- for all these activities call the terminate operator
          for(Item *act:activities){
            Item *predicate = act->parents(0);
            Item *rule = KB.getChild(Terminate_keyword, predicate);
            if(!rule) HALT("No termination rule for '" <<*predicate <<"'");
            Item *effect = rule->kvg().last();
            ItemL vars = getSymbolsOfScope(rule->kvg());
            ItemL subs(vars.N); subs.setZero();
            CHECK(vars.N==act->parents.N-1,"");
            for(uint i=0;i<vars.N;i++) subs(i) = act->parents(i+1);

            if(verbose>2){ cout <<"*** applying" <<*effect <<" SUBS"; listWrite(subs, cout); cout <<endl; }
            applyEffectLiterals(state, effect->kvg(), subs, &rule->kvg());
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
  testFolFwdChaining();
//  testFolDisplay();
//  testFolSubstitution();
//  testMonteCarlo();
  cout <<"BYE BYE" <<endl;
}
