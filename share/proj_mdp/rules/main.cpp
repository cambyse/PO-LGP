#include <MT/array.h>
#include <MT/util.h>
#include <MT/infer.h>

#include "rules.h"

arr coupling(double weight){
  arr F(2,2);
  F(0,0)=1.; F(0,1)=1.;
  F(1,0)=exp(-weight); F(1,1)=exp(weight);
  return F;
}

void RulesToFactorGraph(infer::VariableList& vars, infer::FactorList& facs,
                        const VariableList& V, const RuleList& R){
  uint i, j;
  Variable *v;
  for_list(i, v, V){
    if(v->valueNames.N){
      for(j=0; j<v->dim; j++){
        vars.append(new infer::Variable(2, STRING(v->name <<'=' <<v->valueNames(j))));
        vars.append(new infer::Variable(2, STRING(v->name <<'=' <<v->valueNames(j) <<'\'')));
      }
    }else{
      for(j=0; j<v->dim; j++){
        vars.append(new infer::Variable(2, STRING(v->name <<'=' <<j)));
        vars.append(new infer::Variable(2, STRING(v->name <<'=' <<j <<'\'')));
      }
    }
  }
  
  Rule *r;
  //infer::VariableList FG_vars;
  infer::Factor *f;
  infer::Variable *rule_var,*value_var;
  MT::String value_var_name;
  for_list(i, r, R){
    //cout <<"rule="; r->write(cout); cout <<endl;
    vars.append(rule_var=new infer::Variable(2, STRING("rule"<<i)));
    
    //FG_vars.clear();
    for_list(j, v, r->vars){
      //construct the RV_name for this token
      value_var_name = v->name;
      if(v->valueNames.N) value_var_name <<"=" <<v->valueNames(r->values(j));
      else                value_var_name <<"=" <<r->values(j);
      if(j>=r->arrow) value_var_name <<'\''; //is on the RHS -> refer to RHS RV
      //cout <<var_name <<endl;
      value_var=listFindByName(vars, value_var_name);
      facs.append(new infer::Factor(ARRAY(rule_var,value_var), coupling(r->weight)));
      //FG_vars.append(value_var);
    }
    //cout <<ivars <<endl;
    //facs.append(new infer::Factor(FG_vars));
  }
}

int main(int argn, char** argv){
  MT::initCmdLine(argn, argv);
  
  VariableList V;
  RuleList R;
  
  ifstream fil("coffee_shop.world");
  fil >>"Variables";
  listRead(V, fil, "{}");
  fil >>"Rules";
  listRead(R, fil, "{}");
  fil.close();
  
  cout <<"Variables";  listWrite(V, cout, "  ", "{}");
  cout <<"\n\nRules";  listWrite(R, cout, "  ", "{}");
  
  
  //--------------
  infer::VariableList vars;
  infer::FactorList facs;
  RulesToFactorGraph(vars, facs, V, R);
  
  cout <<vars <<endl;
  cout <<facs <<endl;
}
