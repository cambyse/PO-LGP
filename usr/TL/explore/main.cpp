#include <iostream>
#include <map>
#include <ctime>
#include <stdlib.h>
#include <cstdlib>
#include <MT/ors.h>
#include <MT/opengl.h>
#include <MT/util.h>
#include <TL/robotManipulationDomain.h>
#include <TL/logicReasoning.h>
#include <TL/ruleReasoning.h>
#include <TL/plan.h>
#include <TL/ruleExplorer.h>
#include <TL/logicObjectManager.h>


#define REWARD_SUCCESS 1
#define REWARD_FAILED_INTERRUPT 2
#define REWARD_FAILED_TIMEOUT 3

// #define MAKE_MOVIE
// Verstelle ausserdem:  behavior-type --> 3,  rule-confidence-threshold --> 1

#define MAKE_FAST

RobotManipulationSimulator sim;

#ifndef MAKE_MOVIE // NORMAL
void initSimulator(const char* configurationFile, bool takeMovie = false) {
  sim.loadConfiguration(configurationFile);
  sim.startOde();
  sim.startSwift();
  if (takeMovie)
    sim.startRevel();
// 	sim.watch();
  sim.simulate(60);
// 	sim.relaxPosition();
// 	sim.watch();
}
#else  // mit Texturen
// Bilder: muessen 512x512 sein
// Konvertierung JPG->ppm mittels convert auf Shell

// static GLuint texture_name__wall_back,texture_name__wall_left;
static uint texture_name__wall_back,texture_name__wall_left;
static byteA texImg1,texImg2,texImg3;
static bool useTextures=false;

void initTextures(void){
  useTextures=true;
  MT::String texture1, texture2, texture3;
  MT::getParameter(texture1, MT::String("texture_wall_back"), MT::String(""));
  MT::getParameter(texture2, MT::String("texture_wall_left"), MT::String(""));
	PRINT(texture1);
	PRINT(texture2);
//  MT::getParameter(texture3, MT::String("texture_ground"), MT::String(""));
  read_ppm(texImg1,texture1);  texture_name__wall_back=glImageTexture(texImg1);
  read_ppm(texImg2,texture2);  texture_name__wall_left=glImageTexture(texImg2);
//  read_ppm(texImg3,texture3);  texName3=glImageTexture(texImg3);
	
}

void draw(void* horst){
  glStandardLight(horst);

	glColor(1,1,1);
    glDrawTexQuad(texture_name__wall_back, 
          .5,  1.0, 2.0,
        -1.5,  1.0, 2.0,
			  -1.5,  1.0, 0.0,
          .5,  1.0, 0.0);

  glDrawTexQuad(texture_name__wall_left,
        -1.5,   1.0, 2.0,
        -1.5,  -1.0, 2.0,
        -1.5,  -1.0, 0.0,
			  -1.5,   1.0, 0.0);
}

void initSimulator(const char* configurationFile, bool takeMovie = false) {
  sim.loadConfiguration(configurationFile);
  initTextures();
  sim.gl->add(draw,0);
  orsDrawProxies = false;
  orsDrawJoints = false;
	sim.startOde();
  sim.startSwift();
	if (takeMovie)
    sim.startRevel();
  sim.simulate(100);
//   sim.watch();
}
#endif




void interpret_color(MT::String& name, double* color) {
  double red[3];  red[0]=1.0;  red[1]=0.0;   red[2]=0.0;
  double green[3];  green[0]=0.2;  green[1]=1.0;   green[2]=0.0;
  double orange[3];  orange[0]=1.0;  orange[1]=0.5;   orange[2]=0.0;
  double yellow[3];  yellow[0]=1.0;  yellow[1]=1.0;   yellow[2]=0.0;
  double blue[3];  blue[0]=.0;  blue[1]=.0;   blue[2]=1.0;
  double brown[3];  brown[0]=.5;  brown[1]=.3;   brown[2]=.15;
  double yellow_green[3];  yellow_green[0]=.8;  yellow_green[1]=1.;   yellow_green[2]=.0;
  double grey[3];  grey[0]=.6;  grey[1]=.5;   grey[2]=.5;
  double light_blue[3];  light_blue[0]=.4;  light_blue[1]=1.;   light_blue[2]=1.;
  double purple[3];  purple[0]=.4;  purple[1]=0.;   purple[2]=.5;
  double dark_red[3];  dark_red[0]=.7;  dark_red[1]=0.05;   dark_red[2]=.05;
  double dark_blue[3];  dark_blue[0]=.05;  dark_blue[1]=0.;   dark_blue[2]=.7;
  double rose[3];  rose[0]=1.0;  rose[1]=0.5;   rose[2]=.75;

  uint i;

  for (i=0; i<3; i++)
    if(!TL::areEqual(color[i], red[i])) break;
  if (i==3) {name = "red"; return;}
  
  for (i=0; i<3; i++)
    if(!TL::areEqual(color[i], green[i])) break;
  if (i==3) {name = "green"; return;}
  
  for (i=0; i<3; i++)
    if(!TL::areEqual(color[i], orange[i])) break;
  if (i==3) {name = "orange"; return;}
  
  for (i=0; i<3; i++)
    if(!TL::areEqual(color[i], yellow[i])) break;
  if (i==3) {name = "yellow"; return;}
  
  for (i=0; i<3; i++)
    if(!TL::areEqual(color[i], blue[i])) break;
  if (i==3) {name = "blue"; return;}
  
  for (i=0; i<3; i++)
    if(!TL::areEqual(color[i], brown[i])) break;
  if (i==3) {name = "brown"; return;}
  
  for (i=0; i<3; i++)
    if(!TL::areEqual(color[i], yellow_green[i])) break;
  if (i==3) {name = "yellow-green"; return;}
  
  for (i=0; i<3; i++)
    if(!TL::areEqual(color[i], grey[i])) break;
  if (i==3) {name = "grey"; return;}
  
  for (i=0; i<3; i++)
    if(!TL::areEqual(color[i], light_blue[i])) break;
  if (i==3) {name = "light blue"; return;}
  
  for (i=0; i<3; i++)
    if(!TL::areEqual(color[i], rose[i])) break;
  if (i==3) {name = "rose"; return;}
  
  for (i=0; i<3; i++)
    if(!TL::areEqual(color[i], purple[i])) break;
  if (i==3) {name = "purple"; return;}
  
  for (i=0; i<3; i++)
    if(!TL::areEqual(color[i], dark_red[i])) break;
  if (i==3) {name = "dark red"; return;}
  
  for (i=0; i<3; i++)
    if(!TL::areEqual(color[i], dark_blue[i])) break;
  if (i==3) {name = "dark blue"; return;}
  
  name = "";
}






void showMovie(const char* filename) {
  
  bool watch;
  MT::getParameter(watch, "watch");
  PRINT(watch);
  
 
  uint secs_wait;
  MT::getParameter(secs_wait, "secs_wait");
  PRINT(secs_wait);
  
  MT::String file_ors;
  MT::getParameter(file_ors, "file_ors");
  PRINT(file_ors);
  
  
  MT::String file_reward;
  MT::getParameter(file_reward, "file_reward");
  PRINT(file_reward);
  
  MT::String languageFile_name;
  MT::getParameter(languageFile_name, "file_language");
  PRINT(languageFile_name);
  
  bool take_movie;
  MT::getParameter(take_movie, "movie", false);
  if (take_movie) cout<<"We gonna take a movie of that!"<<endl;
  
  
  // -------------------------------------
  //  SET UP LOGIC
  // -------------------------------------
    
  // Create LogicEngine object
  TL::logicObjectManager::setPredicatesAndFunctions(languageFile_name);
  
  // -------------------------------------
  // INIT SIMULATOR
  // -------------------------------------
  MT::String start_config_path;
  start_config_path = file_ors;
  initSimulator(start_config_path, take_movie);
  uintA objects;
  sim.getObjects(objects);
  TermTypeL objects_types;
  sim.getTypes(objects_types, objects, TL::logicObjectManager::types);
  TL::logicObjectManager::setConstants(objects, objects_types);
  
  // object colors
  arr sizes;
  uint i;
  FOR1D(objects, i) {
    double* shape = sim.getSize(objects(i));
    sizes.append(shape[0]);
  }

  cout<<"OBJECTS:"<<endl;
  FOR1D(objects, i) {
    cout<<objects(i)<<"  " <<objects_types(i)->name<<"   "<<sizes(i)<<endl;
  }

  
  // -------------------------------------
  // MESSAGES
  // -------------------------------------

  MT::Array< MT::String > messages;
//   messages.append(MT::String("Direct explore\ngrab(66) (yellow)"));
//   messages.append(MT::String("Direct explore\nputon(67) (red)"));
//   messages.append(MT::String("Exploit\ngrab(65) (green)"));
//   messages.append(MT::String("Exploit\nputon(66) (yellow)"));
//   messages.append(MT::String("Exploit\ngrab(65) (green)"));
//   messages.append(MT::String("Exploit\nputon(67) (red)"));
//   messages.append(MT::String("Exploit\ngrab(66) (yellow)"));
//   messages.append(MT::String("Exploit\nputon(65) (green)"));

  
  // -------------------------------------
  //  Reward
  // -------------------------------------
  
  cout<<"Reading reward from file \""<<file_reward<<"\"... "<<flush;
  TL::Reward* reward = TL::readReward(file_reward);
  cout<<"done!"<<endl;
  cout<<"REWARD: ";  reward->writeNice(); cout<<endl;cout<<endl;

  
  ifstream in(filename);
  PRINT(filename);
  AtomL actions;
  while (MT::skip(in) != -1) {
    MT::String line;
    in >> line;
    actions.append(TL::logicObjectManager::getAtom(line));
  }
  cout<<"MOVIE will show:  "<<actions<<endl;
  
  #ifdef MT_FREEGLUT
  if (watch) {
    cerr<<"Wait..."<<flush;
    sim.watch();
  }
  #endif
  sim.simulate(50);
  
  FOR1D(actions, i) {
    cout<<"ACTION #"<<i<<":  "<<*actions(i)<<endl;
    if (i < messages.N)
      TL::RobotManipulationDomain::performAction(actions(i), &sim, 10, messages(i));
    else
      TL::RobotManipulationDomain::performAction(actions(i), &sim, 10);
    #ifdef MT_FREEGLUT
    if (watch) {
            cerr<<"Wait..."<<flush;
            sim.watch();
    }
    #endif
  }
  
  
  sim.simulate(100);
}











// ------------------------------------------------------------------
// ------------------------------------------------------------------
// ------------------------------------------------------------------
// ------------------------------------------------------------------
// ------------------------------------------------------------------
// ------------------------------------------------------------------
// ------------------------------------------------------------------
// ------------------------------------------------------------------







void experiment_exploration() {
  cout<<"***** experiment_exploration *****"<<endl;
  
  // -------------------------------------
  // READ CONFIG
  // -------------------------------------
  uint randSeed;
  MT::getParameter(randSeed, "randSeed");
  rnd.seed(randSeed);
  PRINT_(randSeed);

 
  uint representation_uint;
  MT::getParameter(representation_uint, "representation");
  PRINT_(representation_uint);
  TL::RuleExplorer::RepresentationType representation;
  switch(representation_uint) {
    case TL::RuleExplorer::relational: representation = TL::RuleExplorer::relational;  break;
    case TL::RuleExplorer::factored: representation = TL::RuleExplorer::factored;  break;
    case TL::RuleExplorer::flat: representation = TL::RuleExplorer::flat;  break;
    default: NIY;
  }
  PRINT(representation);
  
  
  uint behavior_type;
  MT::getParameter(behavior_type, "behavior_type");
  PRINT_(behavior_type);
  
  bool use_known_state_partial;
  MT::getParameter(use_known_state_partial, "use_known_state_partial");
  PRINT_(use_known_state_partial);
  
  
  TL::RuleExplorer::DensityEstimationType density_estimation_type;
  uint uint__density_estimation_type;
  MT::getParameter(uint__density_estimation_type, "density_estimation_type");
  if (uint__density_estimation_type == 0)
    density_estimation_type = TL::RuleExplorer::density_simple;
  else if (uint__density_estimation_type == 1)
    density_estimation_type = TL::RuleExplorer::density_entropy;
  else
    NIY;
  PRINT(density_estimation_type);
 
  
  uint PRADA_horizon;
  MT::getParameter(PRADA_horizon, "PRADA_horizon");
  PRINT_(PRADA_horizon);
  
  uint PRADA_num_samples;
  MT::getParameter(PRADA_num_samples, "PRADA_num_samples");
  PRINT_(PRADA_num_samples);
  
  double PRADA_noise_softener;
  MT::getParameter(PRADA_noise_softener, "PRADA_noise_softener");
  PRINT_(PRADA_noise_softener);
  
  double PRADA_threshold_reward_achieved;
  MT::getParameter(PRADA_threshold_reward_achieved, "PRADA_threshold_reward_achieved");
  PRINT_(PRADA_threshold_reward_achieved);
  
  double discountFactor;
  MT::getParameter(discountFactor, "discountFactor");
  PRINT_(discountFactor);
  
    
  double rule_learning__alpha_coeff__abstract;
  MT::getParameter(rule_learning__alpha_coeff__abstract, "abstract__complexity_penalty_coeff");
  PRINT_(rule_learning__alpha_coeff__abstract);
  
  double abstract__p_lower_bound__noise_outcome;
  MT::getParameter(abstract__p_lower_bound__noise_outcome, "abstract__p_lower_bound__noise_outcome");
  PRINT_(abstract__p_lower_bound__noise_outcome);
  
  double abstract__p_lower_bound__noise_outcome_in_default_rule;
  MT::getParameter(abstract__p_lower_bound__noise_outcome_in_default_rule, "abstract__p_lower_bound__noise_outcome_in_default_rule");
  PRINT_(abstract__p_lower_bound__noise_outcome_in_default_rule);
  
  double rule_learning__alpha_coeff__factored;
  MT::getParameter(rule_learning__alpha_coeff__factored, "factored__complexity_penalty_coeff");
  PRINT_(rule_learning__alpha_coeff__factored);
  
  double factored__p_lower_bound__noise_outcome;
  MT::getParameter(factored__p_lower_bound__noise_outcome, "factored__p_lower_bound__noise_outcome");
  PRINT_(factored__p_lower_bound__noise_outcome);
  
  double factored__p_lower_bound__noise_outcome_in_default_rule;
  MT::getParameter(factored__p_lower_bound__noise_outcome_in_default_rule, "factored__p_lower_bound__noise_outcome_in_default_rule");
  PRINT_(factored__p_lower_bound__noise_outcome_in_default_rule);

  double rule_learning__alpha_coeff, p_lower_bound__noise_outcome, p_lower_bound__noise_outcome_in_default_rule;
  if (representation == TL::RuleExplorer::relational) {
    rule_learning__alpha_coeff = rule_learning__alpha_coeff__abstract;
    p_lower_bound__noise_outcome = abstract__p_lower_bound__noise_outcome;
    p_lower_bound__noise_outcome_in_default_rule = abstract__p_lower_bound__noise_outcome_in_default_rule;
  }
  else if (representation == TL::RuleExplorer::factored) {
    rule_learning__alpha_coeff = rule_learning__alpha_coeff__factored;
    p_lower_bound__noise_outcome = factored__p_lower_bound__noise_outcome;
    p_lower_bound__noise_outcome_in_default_rule = factored__p_lower_bound__noise_outcome_in_default_rule;
  }
  else if (representation == TL::RuleExplorer::flat) {
    rule_learning__alpha_coeff = 0.;
    p_lower_bound__noise_outcome = 0.;
    p_lower_bound__noise_outcome_in_default_rule = 0.;
  }
  else
    NIY;
  PRINT_(rule_learning__alpha_coeff);
  PRINT_(p_lower_bound__noise_outcome);
  PRINT_(p_lower_bound__noise_outcome_in_default_rule);
  
  
  bool watch;
  MT::getParameter(watch, "watch");
  PRINT_(watch);
  
  uint num_trials;
  MT::getParameter(num_trials, "num_trials");
  PRINT_(num_trials);
  
  int experience_id;
  MT::getParameter(experience_id, "experience_id");
  PRINT_(experience_id);
  
  uint num_rounds;
  MT::getParameter(num_rounds, "num_rounds");
  PRINT_(num_rounds);
  
  uint max_actions;
  MT::getParameter(max_actions, "max_actions");
  PRINT_(max_actions);
  
  uint secs_wait;
  MT::getParameter(secs_wait, "secs_wait");
  PRINT_(secs_wait);
  
  MT::String languageFile_name;
  MT::getParameter(languageFile_name, "file_language");
  PRINT_(languageFile_name);
  
  MT::String filename_results;
  MT::getParameter(filename_results, "file_results");
  PRINT_(filename_results);
  
   
    
  bool take_movie;
  MT::getParameter(take_movie, "movie", false);
  if (take_movie) cout<<"We gonna take a movie of that!"<<endl;
  
  bool fixed_contexts;
  MT::getParameter(fixed_contexts, "fixed_contexts", false);
  PRINT(fixed_contexts);
  
  
  
  
  bool do_rounds_with_same_file;
  MT::getParameter(do_rounds_with_same_file, "do_rounds_with_same_file");
  PRINT_(do_rounds_with_same_file);
  
  MT::String file_ors;
  MT::String file_reward;
  
  MT::Array< MT::String > files_ors;
  MT::String file_ors_1, file_ors_2, file_ors_3, file_ors_4, file_ors_5,
             file_ors_6, file_ors_7, file_ors_8, file_ors_9, file_ors_10;

  MT::Array< MT::String > files_reward;
  MT::String file_reward_1, file_reward_2, file_reward_3, file_reward_4, file_reward_5,
             file_reward_6, file_reward_7, file_reward_8, file_reward_9, file_reward_10;
             
  arr p_lower_bound__noise_outcomeA;
  double p_lower_bound__noise_outcome_1, p_lower_bound__noise_outcome_2, p_lower_bound__noise_outcome_3,
         p_lower_bound__noise_outcome_4, p_lower_bound__noise_outcome_5, p_lower_bound__noise_outcome_6,
         p_lower_bound__noise_outcome_7, p_lower_bound__noise_outcome_8, p_lower_bound__noise_outcome_9;
         
  arr p_lower_bound__noise_outcome_in_default_ruleA;
  double p_lower_bound__noise_outcome_in_default_rule_1, p_lower_bound__noise_outcome_in_default_rule_2, p_lower_bound__noise_outcome_in_default_rule_3,
         p_lower_bound__noise_outcome_in_default_rule_4, p_lower_bound__noise_outcome_in_default_rule_5, p_lower_bound__noise_outcome_in_default_rule_6,
         p_lower_bound__noise_outcome_in_default_rule_7, p_lower_bound__noise_outcome_in_default_rule_8, p_lower_bound__noise_outcome_in_default_rule_9;
         
  
  if (do_rounds_with_same_file) {
    MT::getParameter(file_ors, "file_ors");
    PRINT_(file_ors);
  
    MT::getParameter(file_reward, "file_reward");
    PRINT_(file_reward);
  }
  else {
    MT::getParameter(file_ors_1, "file_ors_1");
    PRINT_(file_ors_1);
    files_ors.append(file_ors_1);
  
    MT::getParameter(file_ors_2, "file_ors_2");
    PRINT_(file_ors_2);
    files_ors.append(file_ors_2);
  
    MT::getParameter(file_ors_3, "file_ors_3");
    PRINT_(file_ors_3);
    files_ors.append(file_ors_3);
  
    MT::getParameter(file_ors_4, "file_ors_4");
    PRINT_(file_ors_4);
    files_ors.append(file_ors_4);
  
    MT::getParameter(file_ors_5, "file_ors_5");
    PRINT_(file_ors_5);
    files_ors.append(file_ors_5);
  
    MT::getParameter(file_ors_6, "file_ors_6");
    PRINT_(file_ors_6);
    files_ors.append(file_ors_6);
  
    MT::getParameter(file_ors_7, "file_ors_7");
    PRINT_(file_ors_7);
    files_ors.append(file_ors_7);
  
    MT::getParameter(file_ors_8, "file_ors_8");
    PRINT_(file_ors_8);
    files_ors.append(file_ors_8);
  
    MT::getParameter(file_ors_9, "file_ors_9");
    PRINT_(file_ors_9);
    files_ors.append(file_ors_9);
  
    MT::getParameter(file_ors_10, "file_ors_10");
    PRINT_(file_ors_10);
    files_ors.append(file_ors_10);
    
    
    MT::getParameter(file_reward_1, "file_reward_1");
    PRINT_(file_reward_1);
    files_reward.append(file_reward_1);
  
    MT::getParameter(file_reward_2, "file_reward_2");
    PRINT_(file_reward_2);
    files_reward.append(file_reward_2);
  
    MT::getParameter(file_reward_3, "file_reward_3");
    PRINT_(file_reward_3);
    files_reward.append(file_reward_3);
  
    MT::getParameter(file_reward_4, "file_reward_4");
    PRINT_(file_reward_4);
    files_reward.append(file_reward_4);
  
    MT::getParameter(file_reward_5, "file_reward_5");
    PRINT_(file_reward_5);
    files_reward.append(file_reward_5);
   
    MT::getParameter(file_reward_6, "file_reward_6");
    PRINT_(file_reward_6);
    files_reward.append(file_reward_6);
  
    MT::getParameter(file_reward_7, "file_reward_7");
    PRINT_(file_reward_7);
    files_reward.append(file_reward_7);
  
    MT::getParameter(file_reward_8, "file_reward_8");
    PRINT_(file_reward_8);
    files_reward.append(file_reward_8);
  
    MT::getParameter(file_reward_9, "file_reward_9");
    PRINT_(file_reward_9);
    files_reward.append(file_reward_9);
    
    MT::getParameter(file_reward_10, "file_reward_10");
    PRINT_(file_reward_10);
    files_reward.append(file_reward_10);
  
  
    if (files_reward.N != files_ors.N)
      HALT("files_reward.N != files_ors.N");
    if (files_reward.N < num_rounds)
      HALT("files_reward.N < num_rounds");
    
    
    MT::getParameter(p_lower_bound__noise_outcome_1, "p_lower_bound__noise_outcome_1");
    PRINT_(p_lower_bound__noise_outcome_1);
    p_lower_bound__noise_outcomeA.append(p_lower_bound__noise_outcome_1);
    
    MT::getParameter(p_lower_bound__noise_outcome_2, "p_lower_bound__noise_outcome_2");
    PRINT_(p_lower_bound__noise_outcome_2);
    p_lower_bound__noise_outcomeA.append(p_lower_bound__noise_outcome_2);
    
    MT::getParameter(p_lower_bound__noise_outcome_3, "p_lower_bound__noise_outcome_3");
    PRINT_(p_lower_bound__noise_outcome_3);
    p_lower_bound__noise_outcomeA.append(p_lower_bound__noise_outcome_3);
    
    MT::getParameter(p_lower_bound__noise_outcome_4, "p_lower_bound__noise_outcome_4");
    PRINT_(p_lower_bound__noise_outcome_4);
    p_lower_bound__noise_outcomeA.append(p_lower_bound__noise_outcome_4);
    
    MT::getParameter(p_lower_bound__noise_outcome_5, "p_lower_bound__noise_outcome_5");
    PRINT_(p_lower_bound__noise_outcome_5);
    p_lower_bound__noise_outcomeA.append(p_lower_bound__noise_outcome_5);
    
    MT::getParameter(p_lower_bound__noise_outcome_6, "p_lower_bound__noise_outcome_6");
    PRINT_(p_lower_bound__noise_outcome_6);
    p_lower_bound__noise_outcomeA.append(p_lower_bound__noise_outcome_6);
    
    MT::getParameter(p_lower_bound__noise_outcome_7, "p_lower_bound__noise_outcome_7");
    PRINT_(p_lower_bound__noise_outcome_7);
    p_lower_bound__noise_outcomeA.append(p_lower_bound__noise_outcome_7);
    
    MT::getParameter(p_lower_bound__noise_outcome_8, "p_lower_bound__noise_outcome_8");
    PRINT_(p_lower_bound__noise_outcome_8);
    p_lower_bound__noise_outcomeA.append(p_lower_bound__noise_outcome_8);
    
    MT::getParameter(p_lower_bound__noise_outcome_9, "p_lower_bound__noise_outcome_9");
    PRINT_(p_lower_bound__noise_outcome_9);
    p_lower_bound__noise_outcomeA.append(p_lower_bound__noise_outcome_9);
    
    
    MT::getParameter(p_lower_bound__noise_outcome_in_default_rule_1, "p_lower_bound__noise_outcome_in_default_rule_1");
    PRINT_(p_lower_bound__noise_outcome_in_default_rule_1);
    p_lower_bound__noise_outcome_in_default_ruleA.append(p_lower_bound__noise_outcome_in_default_rule_1);
    
    MT::getParameter(p_lower_bound__noise_outcome_in_default_rule_2, "p_lower_bound__noise_outcome_in_default_rule_2");
    PRINT_(p_lower_bound__noise_outcome_in_default_rule_2);
    p_lower_bound__noise_outcome_in_default_ruleA.append(p_lower_bound__noise_outcome_in_default_rule_2);
    
    MT::getParameter(p_lower_bound__noise_outcome_in_default_rule_3, "p_lower_bound__noise_outcome_in_default_rule_3");
    PRINT_(p_lower_bound__noise_outcome_in_default_rule_3);
    p_lower_bound__noise_outcome_in_default_ruleA.append(p_lower_bound__noise_outcome_in_default_rule_3);
    
    MT::getParameter(p_lower_bound__noise_outcome_in_default_rule_4, "p_lower_bound__noise_outcome_in_default_rule_4");
    PRINT_(p_lower_bound__noise_outcome_in_default_rule_4);
    p_lower_bound__noise_outcome_in_default_ruleA.append(p_lower_bound__noise_outcome_in_default_rule_4);
    
    MT::getParameter(p_lower_bound__noise_outcome_in_default_rule_5, "p_lower_bound__noise_outcome_in_default_rule_5");
    PRINT_(p_lower_bound__noise_outcome_in_default_rule_5);
    p_lower_bound__noise_outcome_in_default_ruleA.append(p_lower_bound__noise_outcome_in_default_rule_5);
    
    MT::getParameter(p_lower_bound__noise_outcome_in_default_rule_6, "p_lower_bound__noise_outcome_in_default_rule_6");
    PRINT_(p_lower_bound__noise_outcome_in_default_rule_6);
    p_lower_bound__noise_outcome_in_default_ruleA.append(p_lower_bound__noise_outcome_in_default_rule_6);
    
    MT::getParameter(p_lower_bound__noise_outcome_in_default_rule_7, "p_lower_bound__noise_outcome_in_default_rule_7");
    PRINT_(p_lower_bound__noise_outcome_in_default_rule_7);
    p_lower_bound__noise_outcome_in_default_ruleA.append(p_lower_bound__noise_outcome_in_default_rule_7);
    
    MT::getParameter(p_lower_bound__noise_outcome_in_default_rule_8, "p_lower_bound__noise_outcome_in_default_rule_8");
    PRINT_(p_lower_bound__noise_outcome_in_default_rule_8);
    p_lower_bound__noise_outcome_in_default_ruleA.append(p_lower_bound__noise_outcome_in_default_rule_8);
    
    MT::getParameter(p_lower_bound__noise_outcome_in_default_rule_9, "p_lower_bound__noise_outcome_in_default_rule_9");
    PRINT_(p_lower_bound__noise_outcome_in_default_rule_9);
    p_lower_bound__noise_outcome_in_default_ruleA.append(p_lower_bound__noise_outcome_in_default_rule_9);
  }
  
  
  
  
  
  
 
  
  // -------------------------------------
  //  SET UP LOGIC
  // -------------------------------------
    
  // Create LogicEngine object
  TL::logicObjectManager::setPredicatesAndFunctions(languageFile_name);
  TL::logicObjectManager::writeLanguage("used_language.dat");
  
   
  TL::RuleSet fixed_rules;
  // doNothing rule
  fixed_rules.append(TL::ruleReasoning::getDoNothingRule());
  
  
  bool STACK_EXPERIMENT = false;
  if (TL::logicObjectManager::getFunction(MT::String("sum_height")) != NULL) {
    STACK_EXPERIMENT = true;
  }
  
  
  // -------------------------------------
  // EXPERIMENT
  // -------------------------------------
  
  double t_start, t_finish;
  
  FILE* f_results;
  f_results = fopen(filename_results, "w");
  fprintf(f_results, "#  Trial  Round  Success  Actions  Rules  Explores_Direct   Explores_Planned   Exploits    ExploitEnd  ExploitEndSeq    NumObjs  TotalReward\n");
  fflush(f_results);
  
  uint i;
  uint t_state, i_round, i_trial;
  uintA num_actions;
  uintA num_successes;
  for (i_trial=0; i_trial<num_trials; i_trial++) {
    cout<<endl<<endl<<endl<<endl<<"===== TRIAL "<<i_trial<<" ====="<<endl;
    cerr<<endl<<endl<<endl<<endl<<"===== TRIAL "<<i_trial<<" ====="<<endl;
    
    TL::RuleExplorer* explorer;
    if (representation == TL::RuleExplorer::relational) {
      if (!fixed_contexts) {
        explorer = new TL::AbstractRuleExplorer(rule_learning__alpha_coeff__abstract,
                                              abstract__p_lower_bound__noise_outcome, abstract__p_lower_bound__noise_outcome_in_default_rule,
                                              fixed_rules, density_estimation_type);
      }
      else {
        NIY;
      }
    }
    else if (representation == TL::RuleExplorer::factored) {
      explorer = new TL::FactoredRuleExplorer(rule_learning__alpha_coeff__factored, 
                                              factored__p_lower_bound__noise_outcome, factored__p_lower_bound__noise_outcome_in_default_rule, fixed_rules);
    }
    else if (representation == TL::RuleExplorer::flat) {
      explorer = new TL::FlatExplorer(fixed_rules);
    }
    else
      NIY;
    if (experience_id >= 0) {
      MT::String data_file;
      data_file << "experiences_" << experience_id << ".dat";
      cout<<"Reading experience file " << data_file << endl;
      ifstream in(data_file);
      TL::Trial* read_trial = TL::logicObjectManager::readTrial_withConstants(data_file, false);
      for (i=1; i<read_trial->states.N; i++) {
        TL::logicObjectManager::makeOriginal(*read_trial->states(i-1));
        read_trial->actions(i-1) = TL::logicObjectManager::getAtomOrig(read_trial->actions(i-1));
        if (i==read_trial->states.N-1)
          TL::logicObjectManager::makeOriginal(*read_trial->states(i));
      }
      explorer->addObservations(*read_trial);
      delete read_trial;
    }
    
    TL::Trial trial;
    trial.constants = TL::logicObjectManager::constants;
    
//     try {

    // -------------------------------------
    //   ROUND  [start]
    // -------------------------------------
    for (i_round= 0; i_round < num_rounds; i_round++) {
      if (num_rounds> 0) {
        cout<<endl<<endl<<endl<<endl<<"----- ROUND "<<i_round<<" -----"<<endl;
        cerr<<endl<<endl<<endl<<endl<<"----- ROUND "<<i_round<<" -----"<<endl;
      }
      
      // -------------------------------------
      // init simulator
      // -------------------------------------
      if (do_rounds_with_same_file)
        initSimulator(file_ors, take_movie);
      else {
        initSimulator(files_ors(i_round), take_movie);
        cout<<"Ors-file:  "<<files_ors(i_round)<<endl;
        
        if (STACK_EXPERIMENT  &&  representation == TL::RuleExplorer::relational) {
          ((TL::AbstractRuleExplorer*) explorer)->set_p_lower_bounds(p_lower_bound__noise_outcomeA(i_round), p_lower_bound__noise_outcome_in_default_ruleA(i_round));
        }
      }
      
      uintA objects;
      sim.getObjects(objects);
      TermTypeL objects_types;
      sim.getTypes(objects_types, objects, TL::logicObjectManager::types);
      uintA blocks;
      sim.getBlocks(blocks);
      PRINT(blocks);
      uintA balls;
      sim.getBalls(balls);
      PRINT(balls);
      uintA boxes;
      sim.getBoxes(boxes);
      PRINT(boxes);
      TL::logicObjectManager::setConstants(objects, objects_types);
      PRINT(objects);
        
      arr sizes;
      FOR1D(objects, i) {
        double* shape = sim.getSize(objects(i));
        sizes.append(shape[0]);
      }
      MT::Array< MT::String > color_names;
      FOR1D(objects, i) {
        double* color = sim.getColor(objects(i));
        MT::String color_name;
        interpret_color(color_name, color);
        color_names.append(color_name);
      }

      cout<<"OBJECTS:"<<endl;
      FOR1D(objects, i) {
        cout<<objects(i)<<"  " <<objects_types(i)->name<<"   "<<sizes(i)<<"   "<<color_names(i)<<endl;
      }

      // -------------------------------------
      //   REWARD
      // -------------------------------------
      
      cout<<"Reading reward... "<<flush;
      TL::Reward* reward = NULL;
      if (do_rounds_with_same_file) {
        cout<<"from file \""<<file_reward<<"\"... "<<flush;
        reward = TL::readReward(file_reward);
      }
      else {
        cout<<"from file \""<<files_reward(i_round)<<"\"... "<<flush;
        reward = TL::readReward(files_reward(i_round));
        cout<<"Ors-file:  "<<files_reward(i_round)<<endl;
      }
      cout<<"done!"<<endl;
      cout<<"Candidate reward: ";  reward->writeNice(); cout<<endl;
      // Postprocess reward [START]
      TL::LiteralReward* lr = dynamic_cast<TL::LiteralReward*>(reward);
      if (lr != NULL) {
        TL::Literal* lr_better = NULL;
        // ON-Reward
        if (lr->lit->atom->pred->id == TL::logicObjectManager::getPredicate(MT::String("on"))->id) {
          if (balls.findValue(lr->lit->atom->args(1)) >= 0) {
            uintA better_args(2);
            better_args(0) = lr->lit->atom->args(0);
            if (lr->lit->atom->args(0) == blocks(0))
              better_args(1) = blocks(1);
            else
              better_args(1) = blocks(0);
            lr_better = TL::logicObjectManager::getLiteral(lr->lit->atom->pred, true, better_args);
          }
        }
        if (lr_better != NULL)
          lr->lit->atom = lr_better->atom;
      }
      else {
        TL::LiteralListReward* llr = dynamic_cast<TL::LiteralListReward*>(reward);
        if (llr != NULL) {
          bool changed;
          do {
            changed = false;
            uintA args;
            TL::logicReasoning::calcTerms(llr->lits, args);
            for (i=0; i<llr->lits.N && !changed; i++) {
              if (llr->lits(i)->atom->pred->id == TL::logicObjectManager::getPredicate(MT::String("on"))->id) {
                if (balls.findValue(llr->lits(i)->atom->args(1)) >= 0) { // auf nem Ball
                  uint better_arg_1;
                  do {
                    better_arg_1 = blocks(rnd.num(blocks.N));
                  } while (args.findValue(better_arg_1) >= 0); // Noch nicht benutzter Block
                  TL::Substitution sub;
                  sub.addSubs(llr->lits(i)->atom->args(1), better_arg_1);
                  LitL better_lits;
                  TL::logicReasoning::applyOriginalSub(sub, llr->lits, better_lits);
                  cout<<"Reward replacement:  ";  sub.write();  cout<<endl;
                  changed = true;
                  llr->lits = better_lits;
                }
              }
            }
          } while (changed);
        }
      }
      // Postprocess reward [END]
      if (representation == TL::RuleExplorer::relational) {
        explorer->updateLogicEngineConstants();
      }
      else if (representation == TL::RuleExplorer::factored  ||  representation == TL::RuleExplorer::flat) {
        if (i_round==0)
          explorer->updateLogicEngineConstants();
        else if (!do_rounds_with_same_file && i_round > 0)
          explorer->reset();
        else {
          // don't do anything
        }
      }
      else
        NIY;
      cout<<"REWARD: ";  reward->writeNice(); cout<<endl;
      
      
      bool BOX_CLEARANCE_EXPERIMENT = false;
      if (TL::logicObjectManager::getFunction(MT::String("count_boxCleared")) != NULL  &&  reward->reward_type == TL::Reward::reward_maximize_function
        &&  ((TL::MaximizeFunctionReward*) reward)->fa->f->name == "count_boxCleared"  ) {
        BOX_CLEARANCE_EXPERIMENT = true;
        max_actions = 100;
        MT_MSG("SETTING max_actions = 100 FOR BOX_CLEARANCE_EXPERIMENT");
      }
      
      // display objects and reward in GL [START]
      MT::String message_reward;
      message_reward<<"OBJECTS:"<<endl;
      FOR1D(objects, i) {
        message_reward<<objects(i)<<"  " <<objects_types(i)->name<<"   "<<color_names(i)<<endl;
      }
//       message_reward << endl << "REWARD:   ";  reward->writeNice(message_reward);   message_reward <<endl;
      message_reward << endl << "REWARD:   build tower"; message_reward <<endl;
      #ifndef MAKE_FAST
      sim.simulate(100, message_reward);
      #endif
      // display objects and reward in GL [END]
      
    
      TL::Atom* action = NULL;
      TL::State* current_state = NULL;
      AtomL executed_actions;
      uintA exploits;
      uintA explores_planned;
      uintA explores_direct;
      
      ofstream actions_file("performed_actions.dat");
      
      arr state_rewards;
      
      
      // -------------------------------------
      //   ROUND-RUN  [start]
      // -------------------------------------
      
      bool failed = false;
      
      AtomL good_old_plan;
      
      for (t_state=0; t_state<=max_actions; t_state++) {
        // Observe current state and update explorer
        TL::State* old_state = current_state;
        current_state = TL::RobotManipulationDomain::observeLogic(&sim);
        trial.states.append(current_state);
        double current_state_reward = reward->evaluate(*current_state);
        state_rewards.append(current_state_reward);
        
//         if (sim.gl != NULL) {cout<<"Camera position:  "<<*sim.gl->camera.X<<endl;}
//         sim.write("testinger.ors");
        
        cout<<endl<<endl<<endl<<"+++++ STATE "<<t_state<<" +++++    (round "<<i_round<<", #E "<<explorer->all_experiences.N<<")"<<endl;
        cerr<<endl<<endl<<endl<<"+++++ STATE "<<t_state<<" +++++    (round "<<i_round<<", #E "<<explorer->all_experiences.N<<")"<<endl;
        current_state->write(cout); cout<<endl;
        cout<<"Reward:  ";  reward->writeNice(cout);  cout<<endl;
        cerr<<"Reward:  ";  reward->writeNice(cerr);  cerr<<endl;
        TL::RobotManipulationDomain::writeStateInfo(*current_state, cout);
        TL::RobotManipulationDomain::writeStateInfo(*current_state, cerr);
        PRINT(current_state_reward);
        PRINT2(current_state_reward, cerr);
        PRINT(state_rewards);
        PRINT2(state_rewards, cerr);
        
        if (t_state>0) {
          explorer->addObservation(old_state, action, current_state);
          cout<<"DEL: "<<explorer->all_experiences.last()->del<<endl;
          cout<<"ADD: "<<explorer->all_experiences.last()->add<<endl;
          cerr<<"DEL: "<<explorer->all_experiences.last()->del<<endl;
          cerr<<"ADD: "<<explorer->all_experiences.last()->add<<endl;
          uint experience_changes = explorer->all_experiences.last()->add.N + explorer->all_experiences.last()->del.N;
          cout<<"--> Experience changes = "<<experience_changes<<endl;
          if (experience_changes>5) cout<<"Difficult experience (many changes)"<<endl;
        }
        
        // ++++++++++ Learn new rules ++++++++++
        if (t_state>0) {
//           double t_start, t_finish;
          t_start = MT::cpuTime();
          cerr<<"Learning new rules..."<<endl;
          explorer->updateRules(false);  // potentially don't relearn!!
          t_finish = MT::cpuTime();
          cout<<"Rule learning took " << (t_finish - t_start) << "s"<<endl;
          cerr<<"Rule learning took " << (t_finish - t_start) << "s"<<endl;
          if (explorer->is_major_experience.last()) {
            cout <<"INSTRUCTIVE EXPERIENCE" << endl << endl;
            cerr <<"INSTRUCTIVE EXPERIENCE" << endl << endl;
          }
          else
            cout <<"Experience as expected" << endl << endl;
//           explorer->rulesC.write_experiencesWithRules();
          // Display learned rule for latest experience [START]
          // (1) Display on shell
          const TL::RuleSet& rules = explorer->getRules();
          uintA nonDefaultRules_for_last_experience;
          explorer->get_nonDefaultRules_for_last_experience(nonDefaultRules_for_last_experience);
          cout<<endl<<nonDefaultRules_for_last_experience.N<<" uptodate rule(s) explaining last experience " << *action << ":   rules="<<nonDefaultRules_for_last_experience<<endl;
          cerr<<endl<<nonDefaultRules_for_last_experience.N<<" uptodate rule(s) explaining last experience: "<< *action << ":   rules="<<nonDefaultRules_for_last_experience<< endl;
          FOR1D(nonDefaultRules_for_last_experience, i) {
            rules.elem(nonDefaultRules_for_last_experience(i))->write();
            rules.elem(nonDefaultRules_for_last_experience(i))->write(cerr);
          }
          if (nonDefaultRules_for_last_experience.N != 1) {
            cout << " ---> No unique covering rule and thus default rule has to be used."<<endl;
            cerr << " ---> No unique covering rule and thus default rule has to be used."<<endl;
          }
          // (2) Display in GL
          MT::String message;
          message << "ACTION #" << (t_state-1) << ":    " << explorer->message << endl << endl;
          uint msg_time = 30;
          if (explorer->is_major_experience.last()) {
            message <<"INSTRUCTIVE EXPERIENCE" << endl << endl;
            message << "New rule explaining last experience:" << endl<<endl;
            if (nonDefaultRules_for_last_experience.N == 1) {
              rules.elem(nonDefaultRules_for_last_experience(0))->write(message, true);
              msg_time = 200;
            }
            else
              message << " -- (only default rule)"<<endl;
          }
          else {
            message <<"Experience as expected" << endl << endl;
          }
          message << endl;
          #ifndef MAKE_FAST
          sim.simulate(msg_time, message);
          #endif
          // Display learned rule for latest experience [END]
        }
        
        if (reward->satisfied(*current_state)) {
          cerr<<"ULTRAKORREKT!!!  Reward has been achieved :D."<<endl;
          cout<<"ULTRAKORREKT!!!  Reward has been achieved :D."<<endl;
          cerr<<"#used actions = "<<t_state<<endl;
          cout<<"#used actions = "<<t_state<<endl;
          if (t_state == 0) {HALT("Reward achieved in start situation!");}
          break;
        }
        else if (!reward->possible(*current_state)) {
          MT_MSG("OH MY GOD!!!  Reward no more possible.");
          cerr<<"OH MY GOD!!!  Reward no more possible."<<endl;
          cout<<"OH MY GOD!!!  Reward no more possible."<<endl;
          failed = true;
          break;
        }
        
        if (STACK_EXPERIMENT  &&  TL::RobotManipulationDomain::has_maximum_stack_value(*current_state)) {
          cerr<<"ULTRAKORREKT!!!  Maximum stack reward has been achieved :D."<<endl;
          cout<<"ULTRAKORREKT!!!  Maximum stack reward has been achieved :D."<<endl;
          cerr<<"#used actions = "<<t_state<<endl;
          cout<<"#used actions = "<<t_state<<endl;
          if (t_state == 0) {HALT("shouldn't happen!");}
          break;
        }
        
        if (BOX_CLEARANCE_EXPERIMENT) {
          uint target_value = blocks.N + balls.N;
          if (target_value != boxes.N) HALT("something strange");
          if (TL::areEqual(current_state_reward, (double) target_value)) {
            cerr<<"ULTRAKORREKT!!!  Everything box-cleared :D."<<endl;
            cout<<"ULTRAKORREKT!!!  Everything box-cleared :D."<<endl;
            cerr<<"#used actions = "<<t_state<<endl;
            cout<<"#used actions = "<<t_state<<endl;
            if (t_state == 0) {HALT("shouldn't happen!");}
            break;
          }
        }
        
        if (t_state == max_actions) {
          cout<<"Time's up..."<<endl;
          cerr<<"Time's up..."<<endl;
          break;
        }
        
        if (t_state%1 == 0) {
  //         MT::String data_file_name;
  //         data_file_name << "experiences_"<<a<<".dat";
  //         ofstream data_file(data_file_name);
  //         trial.write(data_file);
  //         data_file.close();
        }
        
        
        cout<<endl<<endl<<"+++++ ACTION "<<t_state<<" +++++    (round "<<i_round<<")"<<endl;
        cerr<<endl<<endl<<"+++++ ACTION "<<t_state<<" +++++    (round "<<i_round<<")"<<endl;
        cout<<exploits.N<<" exploits"<<endl;
        cout<<explores_planned.N<<" planned explores"<<endl;
        cout<<explores_direct.N<<" direct explores"<<endl;
        cerr<<exploits.N<<" exploits"<<endl;
        cerr<<explores_planned.N<<" planned explores"<<endl;
        cerr<<explores_direct.N<<" direct explores"<<endl;
        
    
        TL::A_PRADA planner;
        planner.setNumberOfSamples(PRADA_num_samples);
//         if (representation == TL::RuleExplorer::relational) {
//           planner.setNumberOfSamples(PRADA_num_samples + 100 * explorer->rules__confidences.N);
//         }
//         else if (representation == TL::RuleExplorer::relational  || representation == TL::RuleExplorer::flat) {
//           planner.setNumberOfSamples(PRADA_num_samples + 10 * explorer->rules__confidences.N);
//         }
//         else
//           NIY;
        planner.setNoiseSoftener(PRADA_noise_softener);
        planner.setDiscount(discountFactor);
        planner.setHorizon(PRADA_horizon);
        planner.setReward(reward);
        planner.setThresholdReward(PRADA_threshold_reward_achieved);
        PRINT(good_old_plan);
        planner.good_old_plans.clear();
        if (good_old_plan.N > 0)
          planner.good_old_plans.append(good_old_plan);
        PRINT(planner.good_old_plans);
        if (planner.good_old_plans.N > 0)
          cout<<"PRADA has good old plans."<<endl;

        cout<<"Number of rules = "<<explorer->getRules().num()<<endl;
        cerr<<"Number of rules = "<<explorer->getRules().num()<<endl;
        
        
        // -------------------------------
        // +++++++++ THE DECISION ++++++++++
        
        t_start = MT::cpuTime();
        cerr<<"Decision making..."<<endl;
        action = explorer->decideAction(*current_state, (TL::NID_Planner*) &planner, behavior_type, use_known_state_partial);
        t_finish = MT::cpuTime();
        cout<<"Decision making took " << (t_finish - t_start) << "s"<<endl;
                                       
        executed_actions.append(action);
        action->write(actions_file);  actions_file << endl;
        switch (explorer->moves.last()) {
          case MOVE_TYPE__EXPLOIT: exploits.append(t_state);  break;
          case MOVE_TYPE__EXPLORE_PLANNED: explores_planned.append(t_state); break;
          case MOVE_TYPE__EXPLORE_DIRECT: explores_direct.append(t_state); break;
          default: NIY;
        }
        
        // remember good old plan
        if (explorer->moves.last() == MOVE_TYPE__EXPLOIT) {
          good_old_plan.clear();
          good_old_plan = explorer->last_exploit_plan;
          PRINT(good_old_plan);
        }

//         explorer->message << "     (" << a << ")";
        
        cout<<endl<<"=====  Action #" << t_state << ": " << *action;
        cerr<<endl<<"=====  Action #" << t_state << ": " << *action;
        switch (explorer->moves.last()) {
            case MOVE_TYPE__EXPLOIT: cout<<"  exploit";  cerr<<"  exploit";  break;
            case MOVE_TYPE__EXPLORE_PLANNED: cout<<"  planned explore";  cerr<<"  planned explore";  break;
            case MOVE_TYPE__EXPLORE_DIRECT: cout<<"  direct explore";  cerr<<"  direct explore";  break;
            default: NIY;
        }
        cout<<"  ====="<<endl;  cerr<<"  ====="<<endl;
        
        cout<<"Executed actions thus far:  ";
        cerr<<"Executed actions thus far:  ";
        FOR1D(executed_actions, i) {
          cout<<"  ";  cerr<<"  ";
          if (i%10 == 0) {cout<<"["<<i<<"] ";  cerr<<"["<<i<<"] ";}
          executed_actions(i)->write(cout);  executed_actions(i)->write(cerr);
          switch (explorer->moves(i + explorer->moves.N-t_state-1)) {
            case MOVE_TYPE__EXPLOIT: cout<<"$";  cerr<<"$";  break;
            case MOVE_TYPE__EXPLORE_PLANNED: cout<<"!";  cerr<<"!";  break;
            case MOVE_TYPE__EXPLORE_DIRECT: cout<<"?";  cerr<<"?";  break;
            default: NIY;
          }
        }
        cout<<endl;  cerr<<endl;
        if (watch) {
          cerr<<"Wait..."<<flush;
          #ifdef MT_FREEGLUT
          sim.watch();
          #else
          std::cin.get();
          #endif
        }
        
        MT::String message_prefix;
        message_prefix << "ACTION #" << t_state << ":    " << explorer->message << endl << endl;
        // Display old rule that predicts action [START]
        MT::String message_old_rule;
        message_old_rule << message_prefix << "Current rule for prediction:"<<endl <<endl;
        cout << "Current rule for prediction:"<<endl;
        TL::Rule* old_covering_rule = NULL;
        if (representation == TL::RuleExplorer::relational) {
          old_covering_rule = explorer->getRules().elem(TL::ruleReasoning::uniqueAbstractCoveringRule_groundedAction(explorer->getRules(), *current_state, action));
        }
        else if (representation == TL::RuleExplorer::factored  ||  representation == TL::RuleExplorer::flat) {
          old_covering_rule = explorer->getRules().elem(TL::ruleReasoning::uniqueCoveringRule_groundedRules_groundedAction(explorer->getRules(), *current_state, action));
        }
        else
          NIY;
        
        if (old_covering_rule == NULL  ||  TL::ruleReasoning::isDefaultRule(old_covering_rule)) {
          message_old_rule << " -- "<<endl;
          cout << " -- "<<endl;
        }
        else {
          old_covering_rule->write(message_old_rule, true);
          old_covering_rule->write(cout, false);
        }
        sim.displayText(message_old_rule.p, 100);
        // Display old rule that predicts action [END]
        
        // -------------------------------
        //  Executing the action
        cerr<<"Execute action... ";
        t_start = MT::cpuTime();
//         explorer->message << "     (" << a << ")";
        TL::RobotManipulationDomain::performAction(action, &sim, 0, message_old_rule);
        sim.simulate(secs_wait, message_old_rule);
        t_finish = MT::cpuTime();
        cerr<<"Action execution took " << (t_finish - t_start) << "s."<<endl;
        
        trial.actions.append(action);
      }
      // -------------------------------------
      //   ROUND-RUN  [end]
      // -------------------------------------
      
      uint round__num_actions = t_state;  // was anderes als nummer!
      
      if (!failed  &&  round__num_actions < max_actions)
        num_successes.append(1);
      else
        num_successes.append(0);
      num_actions.append(t_state);
      
      
      cout<<"Executed action types:  ";   cerr<<"Executed action types:  ";
      for (i=explorer->moves.N-round__num_actions; i<explorer->moves.N; i++) {
        cout<<"  ";  cerr<<"  ";
//           executed_actions(i)->writeNice(cout);  executed_actions(i)->writeNice(cerr);
        switch (explorer->moves(i)) {
          case MOVE_TYPE__EXPLOIT: cout<<"$";  cerr<<"$";  break;
          case MOVE_TYPE__EXPLORE_PLANNED: cout<<"!";  cerr<<"!";  break;
          case MOVE_TYPE__EXPLORE_DIRECT: cout<<"?";  cerr<<"?";  break;
          default: NIY;
        }
      }

      
      cout<<endl;  cerr<<endl;
      
      if (exploits.findValue(round__num_actions-1) >= 0) {
        cerr<<"Round has been finished with exploit."<<endl;
        cout<<"Round has been finished with exploit."<<endl;
      }
      
      
      // Rewards
      if (reward->reward_type == TL::Reward::reward_maximize_function) {
        if (state_rewards.N == 0) HALT("");
        while (state_rewards.N <= max_actions) {
          state_rewards.append(state_rewards.last());
        }
      }
      
      double total_reward = 0.;
      if (reward->reward_type == TL::Reward::reward_maximize_function) {
        for (t_state=0; t_state<=max_actions; t_state++) {
          total_reward += pow(discountFactor, t_state) * state_rewards(t_state);
        }
      
        cout<<"total_reward = "<<total_reward<<"  "<<state_rewards<<endl;
        cerr<<"total_reward = "<<total_reward<<"  "<<state_rewards<<endl;
      }
      
      uint num_finishing_exploits = 0;
      for (i = explorer->moves.N;  i-- > explorer->moves.N-round__num_actions; ) {
        if (explorer->moves(i) == MOVE_TYPE__EXPLOIT)
          num_finishing_exploits++;
        else
          break;
      }
      
      PRINT(num_finishing_exploits);
      PRINT2(num_finishing_exploits, cerr);
      
      fprintf(f_results, "%2i %i %i %2i %2i %2i %2i %2i     %2i %2i   %2i  %5.2f\n",
                i_trial, i_round, num_successes.last(),
                round__num_actions, explorer->getRules().num(),
                explores_direct.N, explores_planned.N, exploits.N,
                exploits.findValue(round__num_actions-1) >= 0, num_finishing_exploits,
                TL::logicObjectManager::constants.N, total_reward);
      fflush(f_results);
      delete reward;
    }
    // -------------------------------------
    //   ROUND  [end]
    // -------------------------------------
//     }
//     catch(...) {
// //       cerr<<"TRIAL failed: error / exception caught for some reason."<<endl;
//       cout<<"TRIAL failed: error / exception caught for some reason."<<endl;
//     }
    
    // Epilogue of trial
    
    FOR1D(trial.states, i) {
      delete trial.states(i);
    }
    
    delete explorer;
  }
  
  sim.simulate(200, MT::String(""));
  
  fprintf(f_results, "# mean(num_actions)   =  %5.2f\n", ((1.0 * sum(num_actions)) / num_actions.N));
  fprintf(f_results, "# mean(num_successes) =  %5.2f\n", ((1.0 * sum(num_successes)) / num_successes.N));
  fclose(f_results);
}














int main(int argc, char** argv) {
  MT::initCmdLine(argc, argv);
  cout.precision(3);
  cerr.precision(3);
  MT::String config_file("config");
//   MT::String config_file("config_exp4");
  cout << "Config-file: " << config_file << endl;
  MT::openConfigFile(config_file);
  
// 	showMovie("film_aktionen.dat");
//   showMovie("film_aktionen_ICML_exp3.dat");
//   showMovie("film_aktionen_ecml10papier.dat");
// 	exit(0);

  experiment_exploration();
  
  return 0;
}



