#define MT_IMPLEMENT_TEMPLATES

#include <relational/reason.h>


void test_basics() {
  cout<<"********************************"<<endl;
  cout<<" libPRADA basics demo"<<endl;
  cout<<" This demo shows how to define symbols and rules."<<endl;
  cout<<"********************************"<<endl;
  
  // ----------------------------------
  // SOME SYMBOLS
  PRADA::Symbol* s_incity = PRADA::Symbol::get(MT::String("incity"), 1);
  PRADA::Symbol* s_cruiseto = PRADA::Symbol::get(MT::String("cruiseto"), 1, PRADA::Symbol::action);
  PRADA::Symbol* s_gas = PRADA::Symbol::get(MT::String("gas"), 0, PRADA::Symbol::primitive, PRADA::Symbol::integer_set);
	uintA gas_values;  gas_values.append(0);  gas_values.append(1);  gas_values.append(2);  gas_values.append(3);  gas_values.append(4);
	s_gas->range = gas_values;
  
  cout<<"SYMBOLS:"<<endl;
  cout<<*s_incity<<endl;
  cout<<*s_cruiseto<<endl;
  cout<<*s_gas<<endl;
  cout<<endl;
  
	PRADA::writeSymbolsAndTypes("used_symbols.dat");
	
  
  // ----------------------------------
  // SOME CONSTANTS
  // (represented by large uints)
  uintA constants;
  constants.append(20); // e.g. city 1 which might be Berlin
  constants.append(21); // e.g. city 2 which might be Frankfurt
  constants.append(22); // e.g. a person which might be Angela Merkel
  
  // Tell the reason-component which uints represent constants
  PRADA::reason::setConstants(constants);
    
  
  
  // ----------------------------------
  // LITERALS, FUNCTION-VALUES and ATOMS
  uintA arguments(1);
  
  arguments(0) = 0;   // Convention if constants are not set (like above): small uints (<10) denote variables.
  PRADA::Literal* l1 = PRADA::Literal::get(s_incity, arguments, 1);
  
	// or write
  PRADA::Literal* l2 = PRADA::Literal::get(s_incity, TUP(0), 0);
  
  arguments(0) = constants(2);  // Convention if constants are not set (like above): larger uints (>=10) denote constants.
  PRADA::Literal* l3 = PRADA::Literal::get(s_incity, arguments, 1);
  
  arguments(0) = 0;
  PRADA::Literal* l_action = PRADA::Literal::get(s_cruiseto, arguments, 1.0);
  
  arguments.clear();  // arguments = {}
  PRADA::Literal* l4 = PRADA::Literal::get(s_gas, arguments, 3);
  PRADA::Literal* l5 = PRADA::Literal::get(s_gas, arguments, 0);
  PRADA::Literal* l_comp =  PRADA::Literal::get(s_gas, arguments, 1., PRADA::Literal::comparison_greater);
   
	PRADA::LitL lits_all;   //  list of literals
	lits_all.append(l1);  lits_all.append(l2);  lits_all.append(l3);  lits_all.append(l4);  lits_all.append(l5);  lits_all.append(l_comp);
	
  cout<<"LITERALS: "<<lits_all<<endl<<endl;
  
  
	
	// ----------------------------------
  // SYMBOLIC STATE
	PRADA::LitL lits_ground;  lits_ground.append(l3);  lits_ground.append(l4);
	PRADA::SymbolicState state(lits_ground);
	cout<<"STATE:  "<<state<<endl<<endl;
  
	
	
  // ----------------------------------
  // RULES
  
  // RULE 1: noisy default rule
  //  --> Always required!!
  double probability_noise_outcome = 0.7;
  PRADA::Rule* rule1 = PRADA::Rule::generateDefaultRule(probability_noise_outcome);
  rule1->noise_changes = 1.7;
  
  // RULE 2
  PRADA::Rule* rule2 = new PRADA::Rule;
	rule2->context.append(l_comp);
	rule2->context.append(PRADA::Literal::get("incity(Y)"));
  rule2->context.append(l2);
  
  rule2->action = l_action;
  
  // Outcome 1:  action succeeded: different city
  PRADA::LitL outcome1;
  outcome1.append(l1);
	outcome1.append(PRADA::Literal::get("-incity(Y)"));
  rule2->outcomes.append(outcome1);
  
  // Outcome 2:  action failed: still in same city
  //  (We've driving around, but could not find our way out of the city.)
  PRADA::LitL outcome2;
  rule2->outcomes.append(outcome2);
  
  // Outcome 3:  noise outcome
  //  (E.g. we ended in a different city, or our car broke on the Autobahn or...)
  PRADA::LitL outcome3; // empty dummy
  rule2->outcomes.append(outcome3);
  rule2->noise_changes = 2.4; // for PRADA's noise outcome heuristic
  
  arr outcome_probabilities(3);
  outcome_probabilities(0) = 0.7;
  outcome_probabilities(1) = 0.2;
  outcome_probabilities(2) = 0.1;
  rule2->probs = outcome_probabilities;
  
	PRADA::RuleSet abstract_rules;
	abstract_rules.append(rule1);
	abstract_rules.append(rule2);
	abstract_rules.sanityCheck();
  // The data-structure RuleSet will take care of deleting
	// the rule pointers.
  cout<<"RULES:"<<endl;
	abstract_rules.write();
	
	
	// Rule coverage
  PRADA::RuleSet coveringGroundRules;
  PRADA::reason::calc_coveringRules(coveringGroundRules, abstract_rules, state);
  cout<<endl<<endl<<"COVERING RULE GROUNDINGS FOR START STATE (#="<<coveringGroundRules.num()<<"):"<<endl;
  coveringGroundRules.write();
  cout<<endl;
	
	
	// Successor state
	cout<<"SAMPLE SUCESSOR STATE"<<endl;
	cout<<"Current state: "<<state<<endl;
// 	PRADA::Literal* ground_action = PRADA::Literal::get("cruiseto(21)");
	MT::String name__ground_action;  name__ground_action<<"cruiseto("<<constants(0)<<")";
	PRADA::Literal* ground_action = PRADA::Literal::get(name__ground_action);
	cout<<"Ground action: "<<*ground_action<<endl;
  PRADA::SymbolicState state_successor;
	PRADA::reason::sampleSuccessorState_abstractRules(state_successor, state, abstract_rules, ground_action);
	cout<<"Sampled successor state:  "<<state_successor<<endl;
  
}



int main(int argc, char** argv) {
  test_basics();
}

