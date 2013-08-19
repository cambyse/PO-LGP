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
  rnd.seed(time(NULL));
	
	
  // -------------------------------------
  //  PARAMETERS
  // -------------------------------------
  // Regularizer
  double alpha_pen = MT::getParameter<double>("alpha", 0.5);
  // Lower bounds for probabilities of states in case of noise outcome
  // ... same, only for noisy default rule
  double prob_state_given_NoisyOutcome__in_noisyDefaultRule = 1e-11;
  // Log-file
  //
  PRINT(alpha_pen);
  PRINT(prob_state_given_NoisyOutcome);

  MT::String logfile("learn.log");
	
  // Symbols
  relational::SymL symbols;
  relational::ArgumentTypeL types;
  relational::readSymbolsAndTypes(symbols, types, "symbols.dat");
	
  // Data
  relational::StateTransitionL transitions = relational::StateTransition::read("transition.dat");
  PRINT(transitions.N);
//   write(transitions);
	
 
  // -------------------------------------
  //  LEARN
  // -------------------------------------
	
  relational::learn::set_penalty(alpha_pen);
  relational::learn::set_p_min(prob_state_given_NoisyOutcome, prob_state_given_NoisyOutcome__in_noisyDefaultRule);
  relational::RuleSetContainer rulesC;
  cout<<"Starting rule-learning... (might take quite a while; watch '"<<logfile<<"')"<<endl;
  relational::learn::learn_rules(rulesC, transitions); 
  
  relational::write(rulesC.rules, "learned_rules.dat");
  cout<<"Learned rules have been written to 'learned_rules.dat'."<<endl;
}



int main(int argc, char** argv){
  MT::initCmdLine(argc,argv);
  cout.precision(2);
  test_learn();
  
  return 0;
}

