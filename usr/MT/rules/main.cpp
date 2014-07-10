#define MT_IMPLEMENTATION
#include <Core/array.h>
#include <MT/util.h>
#include <MT/infer.h>

#include "rules.h"

arr weighted_coupling(double weight){
  arr F(2,2);
  F(0,0)=1.; F(0,1)=1.;
  F(1,0)=exp(-weight); F(1,1)=exp(weight);
  return F;
}

arr change_coupling(){
  arr F(2,2,2);
  F=1.; //if mod=1: neutral
  //if mod=0: identity
  F(0,0,0)=1.; F(0,0,0)=0.;
  F(1,0,0)=0.; F(1,0,1)=1.;
  return F;
}

#define indName(j,plus) \
    if(s->dim==2){ \
      txt.clear() <<s->name plus; \
    }else{ \
      if(s->valueNames.N) txt.clear() <<s->name <<'=' <<s->valueNames(j) plus; \
      else                txt.clear() <<s->name <<'=' <<j plus; \
    }

void RulesToFactorGraph(infer::VariableList& vars, infer::FactorList& facs,
                        const StateVariableList& S, const RuleList& R){
  //CHECK(VarCount,"");
  uint i, j;
  StateVariable *s;
  infer::Factor *f;
  infer::Variable *v_pre,*v_mod,*v_post;
  MT::String txt;
  uint totalIndicators=0;
  for_list(Type,  s,  S){
    if(s->dim==2){
      //-- for each binary variable: an indicator, a modification indicator, and a next-time-step indicate
      indName(0, );
      vars.append(v_pre  = new infer::Variable(2, txt));
      vars.append(v_mod  = new infer::Variable(2, STRING(txt <<"_mod")));
      vars.append(v_post = new infer::Variable(2, STRING(txt <<"'")));
      totalIndicators++;
      
      //-- for each value of each variable: a factor enforcing equality or neutrality depending on change value
      f = new infer::Factor( ARRAY(v_pre,v_mod,v_post),  change_coupling(), "CHANGE" ); //factor's last variable is ``OR output''
      facs.append(f);
    }else for(j=0; j<s->dim; j++){
      //-- for each value of each variable: an indicator, a modification indicator, and a next-time-step indicate
      indName(j, );
      vars.append(v_pre  = new infer::Variable(2, txt));
      vars.append(v_mod  = new infer::Variable(2, STRING(txt <<"_mod")));
      vars.append(v_post = new infer::Variable(2, STRING(txt <<"'")));
      totalIndicators++;

      //-- for each value of each variable: a factor enforcing equality or neutrality depending on change value
      f = new infer::Factor( ARRAY(v_pre,v_mod,v_post),  change_coupling(), "CHANGE" ); //factor's last variable is ``OR output''
      facs.append(f);
    }
  }

  MT::Array<infer::VariableList> rulesThatModIndicator(3*totalIndicators); //memorize for each output var which rule modifies it potentially
  Rule *r;
  infer::Variable *rule_var,*v;
  infer::VariableList lhsVars;
  for_list(Type,  r,  R){
    //-- for each rule: a binary firing variable
    rule_var=new infer::Variable(2, STRING("rule"<<i));
    vars.append(rule_var);
    
    //-- for each rule: a factor taking AND of all LHS indicators and coupling with the rule
    //collect LHS indicators
    lhsVars.clear();
    for_list(Type,  s,  r->vars){
      if(j>=r->arrow) break; //we're on the RHS already
      //construct the name for this indicator
      indName(r->values(j),);
      v=listFindByName(vars, txt);
      lhsVars.append(v);
    }
    f = new infer::Factor( cat(lhsVars, ARRAY(rule_var)), "AND" ); //factor's last variable is ``AND output''
    f->specialType = infer::AND;
    facs.append(f);

    //-- for each rule and each RHS indicator: a factor the implies the predicated probability on the RHS indicators
    for(j=r->arrow; j<r->vars.N; j++){
      s=r->vars(j);
      //construct the name for this indicator (with ' for the next-time-step indicators)
      indName(r->values(j),<<'\'');
      v=listFindByName(vars, txt);
      //the pair-wise factor's table is determined by the coupling weight r->weight
      facs.append(new infer::Factor( ARRAY(rule_var,v), weighted_coupling(r->weight), "rule_out" ));
      rulesThatModIndicator(v->id).append(rule_var); //MT: Warning: this will fail if the global VarCount is not set zero before calling this...
    }
  }

  infer::VariableList ruleVars;
  for_list(Type,  s,  S){
    if(s->dim==2){
      indName(0,<<"_mod");
      v=listFindByName(vars, txt);
      //WARNING: s->id+1 requires that the value' has 'id-plus-1' than the value_mod...
      f = new infer::Factor( cat(rulesThatModIndicator(v->id+1), ARRAY(v)), "OR" ); //factor's last variable is ``OR output''
      f->specialType = infer::OR;
      facs.append(f);
    }else for(j=0; j<s->dim; j++){
      //-- for each change indicator: an OR factor getting input from all modifying rules
      indName(j,<<"_mod");
      v=listFindByName(vars, txt);
      //WARNING: s->id+1 requires that the value' has 'id-plus-1' than the value_mod...
      f = new infer::Factor( cat(rulesThatModIndicator(v->id+1), ARRAY(v)), "OR" ); //factor's last variable is ``OR output''
      f->specialType = infer::OR;
      facs.append(f);
    }
  }
  
  //=== dot order: append an attribute to each element which gives strict dot ordering
  uint dot_order=0;
  for_list(Type,  s,  S){
    if(s->dim==2){
      indName(0,);  v=listFindByName(vars, txt);  v->ats.append(anyNew<uint>("dot_order", dot_order++));
    }else for(j=0; j<s->dim; j++){
      indName(j,);  v=listFindByName(vars, txt);  v->ats.append(anyNew<uint>("dot_order", dot_order++));
    }
  }
  for_list(Type,  f,  facs){
    if(f->name=="AND") f->ats.append(anyNew<uint>("dot_order", dot_order++));
  }
  for_list(Type,  r,  R){
    v=listFindByName(vars, STRING("rule"<<i));  v->ats.append(anyNew<uint>("dot_order", dot_order++));
  }
  for_list(Type,  f,  facs){
    if(f->name=="OR") f->ats.append(anyNew<uint>("dot_order", dot_order++));
  }
  for_list(Type,  s,  S){
    if(s->dim==2){
      indName(0,<<"_mod");  v=listFindByName(vars, txt);  v->ats.append(anyNew<uint>("dot_order", dot_order++));
    }else for(j=0; j<s->dim; j++){
      indName(j,<<"_mod");  v=listFindByName(vars, txt);  v->ats.append(anyNew<uint>("dot_order", dot_order++));
    }
  }
  for_list(Type,  f,  facs){
    if(f->name=="CHANGE") f->ats.append(anyNew<uint>("dot_order", dot_order++));
  }
  for_list(Type,  f,  facs){
    if(f->name=="rule_out") f->ats.append(anyNew<uint>("dot_order", dot_order++));
  }
  for_list(Type,  s,  S){
    if(s->dim==2){
      indName(0,<<'\'');  v=listFindByName(vars, txt);  v->ats.append(anyNew<uint>("dot_order", dot_order++));
    }else for(j=0; j<s->dim; j++){
      indName(j,<<'\'');  v=listFindByName(vars, txt);  v->ats.append(anyNew<uint>("dot_order", dot_order++));
    }
  }
  
}

int main(int argc, char** argv){
  MT::initCmdLine(argc, argv);
  
  StateVariableList S;
  RuleList R;
  
  ifstream fil("coffee_shop.world");
  fil >>"StateVariables";
  listRead(S, fil, "{}");
  fil >>"Rules";
  listRead(R, fil, "{}");
  fil.close();
  
  cout <<"\n\n*** StateVariables\n";  listWrite(S, cout, "  ", "{}");
  cout <<"\n\n*** Rules\n";  listWrite(R, cout, "  ", "{}");
  
  //--------------
  infer::VariableList vars;
  infer::FactorList facs;
  RulesToFactorGraph(vars, facs, S, R);
  
  cout <<"\n\n*** Variables\n" <<vars <<endl;
  cout <<"\n\n*** Factors\n" <<facs <<endl;
  
  ofstream os("coffee_shop.fg");
  os <<vars <<endl <<facs <<endl;
}
