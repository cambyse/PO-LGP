#include <relational/learn.h>


void test_learn() {
	cout<<"********************************"<<endl;
  cout<<" libPRADA learn demo"<<endl;
  cout<<" This demo shows how to learn probabilistic relational rules"<<endl
			<<" with the algorithm by Pasula et al., JAIR (2007)."<<endl
      <<" The demo uses the robot manipulation domain of the experiments in"<<endl
      <<" the paper Lang & Toussaint, JAIR (2010)."<<endl;
  cout<<"********************************"<<endl;
	
	// Rule learning algorithm is heuristic and makes some random choices.
  rnd.seed(12345);
	
	
	// -------------------------------------
  //  PARAMETERS
  // -------------------------------------
  // Regularizer
  double alpha_pen = 0.5;
  // Lower bounds for probabilities of states in case of noise outcome
  double prob_state_given_NoisyOutcome = 1e-8; // p_min
  // ... same, only for noisy default rule
  double prob_state_given_NoisyOutcome__in_noisyDefaultRule = 1e-9;
  // Log-file
  MT::String logfile("learn.log");
	
	// Symbols
  PRADA::SymL symbols;
  PRADA::ArgumentTypeL types;
  PRADA::readSymbolsAndTypes(symbols, types, "symbols.dat");
	
	// Data
  PRADA::StateTransitionL transitions = PRADA::StateTransition::read("data.dat");
  PRINT(transitions.N);
//   write(transitions);
	
 
	// -------------------------------------
  //  LEARN
  // -------------------------------------
	
  PRADA::learn::set_penalty(alpha_pen);
  PRADA::learn::set_p_min(prob_state_given_NoisyOutcome, prob_state_given_NoisyOutcome__in_noisyDefaultRule);
  PRADA::RuleSetContainer rulesC;
	cout<<"Starting rule-learning... (might take quite a while; watch '"<<logfile<<"')"<<endl;
  PRADA::learn::learn_rules(rulesC, transitions); 
  
  PRADA::write(rulesC.rules, "learned_rules.dat");
	cout<<"Learned rules have been written to 'learned_rules.dat'."<<endl;
}



int main(int argc, char** argv){
  cout.precision(2);
  test_learn();
  
  return 0;
}

