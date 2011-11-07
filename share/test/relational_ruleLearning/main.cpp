#include <relational/ruleLearner.h>


void learn_rules() {
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
  MT::String logfile("rulelearn.log");
        
 
  
  // -------------------------------------
  //  SET UP LOGIC
  // -------------------------------------
    
  TL::logicObjectManager::setPredicatesAndFunctions("language.dat");
  
  
  // -------------------------------------
  //  READ DATA
  // -------------------------------------
  
  TL::Trial* trial = TL::logicObjectManager::readTrial_withConstants("samples.dat", true);
  
  ExperienceL experiences;
  uint i;
  FOR1D(trial->actions, i) {
    experiences.append(new TL::Experience(*trial->states(i), trial->actions(i), *trial->states(i+1)));
  }
  
  cout << "#Experiences: " << experiences.N <<endl;
  
  
  // -------------------------------------
  //  RULE LEARNER
  // -------------------------------------
  
  TL::RuleLearner learner(alpha_pen, prob_state_given_NoisyOutcome, prob_state_given_NoisyOutcome__in_noisyDefaultRule);

  TL::RuleSetContainer rulesC;
  cout<<"Starting rule-learning... (might take quite a while; watch 'ruleLearn.log')"<<endl;
  learner.learn_rules(rulesC, experiences, logfile); 
  
  TL::RuleSet& learned_rules = rulesC.rules;
  TL::write(learned_rules, "learned_rules.dat");
  cout<<"Learned rules have been written to 'learned_rules.dat'."<<endl;
  
  cout<<endl<<endl<<"TEST WAS SUCCESSFUL!"<<endl<<"Adeus."<<endl;
}




int main(int argc, char** argv){
  cout.precision(2);
  learn_rules();
  
  return 0;
}

