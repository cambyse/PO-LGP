#define MT_IMPLEMENT_TEMPLATES
#include <stdlib.h>
#include <relational/robotManipulationDomain.h>

RobotManipulationSimulator sim;


void initSimulator(const char* configurationFile, bool takeMovie) {
  sim.shutdownAll();
  sim.loadConfiguration(configurationFile);
#ifdef MT_FREEGLUT
  orsDrawProxies = false;
  orsDrawJoints = false;
#endif
  sim.startOde();
  if (takeMovie)
    sim.startRevel();
  sim.startSwift();
  sim.simulate(50);
}





// state: 20
// 5 Objekte, 5. Objekt ist die Roboterhand
// (1-3): x,y,z-Koords
// (4): Groesse

// successor state: 20
// genauso

// action: 2
// (1): Aktionstyp grasp 1, puton 2
// (2): 

// reward: 1

struct Experience_State {
  MT::Array< arr > continous_data;
//   TL::State symbolic_data;
  MT::Array< MT::String > symbolic_data;
};

struct Experience {
  Experience_State state_pre;
  Experience_State state_post;
  uint action_type;
  uint action_target;
  double reward;
};


  
//   MT::String line;
//   line.read(in, NULL, "\n");
  


void read_data(MT::Array< Experience* >& experiences, const char* file_name) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"read_data [START]"<<endl;}
  ifstream in(file_name);
  CHECK(in.is_open(), "File "<<file_name<<" can't be opened!");
  MT::String line;
  uint OBJ_NUM = 5;
  uint o;
  Experience* exp = new Experience;
  while (MT::skip(in) != -1) {
    in >> line;
    if (DEBUG>0) PRINT(line);
    for (o=0; o<OBJ_NUM; o++) {
      arr data(4);
      line >> data(0);  line >> data(1);  line >> data(2);  line >> data(3);
      if (DEBUG>0) PRINT(data);
      exp->state_pre.continous_data.append(data);
    }
    for (o=0; o<OBJ_NUM; o++) {
      arr data(4);
      line >> data(0);  line >> data(1);  line >> data(2);  line >> data(3);
      if (DEBUG>0) PRINT(data);
      exp->state_post.continous_data.append(data);
    }
    line >> exp->action_type;
    line >> exp->action_target;
    line >> exp->reward;
    experiences.append(exp);
  }
  if (DEBUG>0) {cout<<"read_data [END]"<<endl;}
}




struct PredicateNetwork {
  MT::String name;
  TL::Predicate* p;
  uint arity;
  arr w1a;
  arr w1b;
  arr w2a;
  arr w2b;
  
  void write() {PRINT(name);  PRINT(arity);  PRINT(w1a);  PRINT(w1b);  PRINT(w2a);  PRINT(w2b);}
};


bool holds(PredicateNetwork& pn, arr& x) {
  uint DEBUG = 1;
  if (DEBUG>0) {PRINT(x);  pn.write();}
  // apply network
  arr z_argument = ~pn.w1a * x + pn.w1b;
  arr z;
  uint k;
  FOR1D(z_argument, k) {
    z.append((1. / ( 1. + exp( z_argument(k) ) )));
  }
  if (DEBUG>0) {PRINT(z);}
  arr g_argument = ~pn.w2a * z + pn.w2b;
  if (DEBUG>0) {PRINT(g_argument);}
  double g = (1. / ( 1. + exp( g_argument(0)) ));
  double p = g - 0.5;
  if (DEBUG>0) {PRINT(g);  PRINT(p);}
  return p>0.;
}


void calculateSymbols(MT::Array<PredicateNetwork*>& pns, Experience_State& s) {
  uint DEBUG = 0;
  uint i, k;
  FOR1D(pns, i) {
    uintA objs;
    for (k=0; k<s.continous_data.N; k++) {objs.append(k);}
    MT::Array< uintA > lists;
    TL::allPossibleLists(lists, objs, pns(i)->arity, true, true);
    FOR1D(lists, k) {
      arr x;
      if (pns(i)->arity == 1) {
        x.append(s.continous_data(lists(k)(0)));
      }
      else if (pns(i)->arity == 2) {
        arr x_obj_1 = s.continous_data(lists(k)(0));
        arr x_obj_2 = s.continous_data(lists(k)(1));
        arr diff = x_obj_1 - x_obj_2;
        x.append(diff);  // TODO absolute value??
      }
      else NIY;
      bool is_true = holds(*pns(i), x);
      MT::String atom_name;  atom_name<<pns(i)->name<<lists(k);
      if (is_true) {s.symbolic_data.append(atom_name);}
    }
  }
}


void read_PredicateNetworks(MT::Array<PredicateNetwork*>& pns, const char* prefix) {
  
  MT::Array< MT::String > pn_names;
  pn_names.append(MT::String("u1"));//  pn_names.append(MT::String("u2"));
  pn_names.append(MT::String("b1"));//  pn_names.append(MT::String("b2"));
  
  uint i;
  FOR1D(pn_names, i) {
    PredicateNetwork* pn = new PredicateNetwork;
    pn->arity = 1;
    pn->name = pn_names(i);
    
    ifstream in;
    MT::String file_name;
  
    file_name<<prefix<<pn_names(i)<<"w1a.txt";
    PRINT(file_name);
    in.open(file_name);
    CHECK(in.is_open(), "File "<<file_name<<" can't be opened!");
    pn->w1a.read(in);
    in.close();
  
    file_name.clr();
    file_name<<prefix<<pn_names(i)<<"w1b.txt";
    PRINT(file_name);
    in.open(file_name);
    CHECK(in.is_open(), "File "<<file_name<<" can't be opened!");
    pn->w1b.read(in);
  
    file_name.clr();
    file_name<<prefix<<pn_names(i)<<"w2a.txt";
    PRINT(file_name);
    in.open(file_name);
    CHECK(in.is_open(), "File "<<file_name<<" can't be opened!");
    pn->w2a.read(in);
    PRINT(pn->w2a);
  
    file_name.clr();
    file_name<<prefix<<pn_names(i)<<"w2b.txt";
    PRINT(file_name);
    in.open(file_name);
    CHECK(in.is_open(), "File "<<file_name<<" can't be opened!");
    pn->w2b.read(in);
    
    pns.append(pn);
  }
  
}






#if 1
void test(int argn,char** argv) {
  uint DEBUG = 2;
  
  uint t, i, k, l;
  uint randSeed = 12345;
  rnd.seed(randSeed);

  // Set up logic
  TL::logicObjectManager::setPredicatesAndFunctions("language.dat");
  TL::logicObjectManager::writeLanguage("used_language.dat");
  
  
  // ==================================================================
  // Read in Nikolay's network [START]
  MT::Array< PredicateNetwork* > pns;
  // Read in Nikolay's network [END]
  // ==================================================================
  

  // Set up simulator
  MT::String sim_file("data_grounding_project/situationNik.ors");
  initSimulator(sim_file, false);
  sim.simulate(50);
  
  
  // Add predicates for predicate networks
  PredL new_predicates;
  FOR1D(pns, i) {
    TL::Predicate* p = new TL::Predicate;
    p->id = TL::logicObjectManager::getLowestFreeConceptID() + i;
    p->name = pns(i)->name;
    p->d = pns(i)->arity;
    new_predicates.append(p);
    pns(i)->p = p;
    if (DEBUG>1) {cout<<*p<<endl;}
  }
  TL::logicObjectManager::addStatePredicates(new_predicates);
  
  
  // Get objects and set in logic database
  uintA objs;
  sim.getObjects(objs);
  objs.append(sim.getHandID());
  TL::logicObjectManager::setConstants(objs);
  
  cout<<"Table-ID="<<sim.getTableID()<<endl;
  cout<<"Hand-ID="<<sim.getHandID()<<endl;

  
  AtomL new_atoms;
  FOR1D(new_predicates, i) {
    AtomL new_atoms_p;
    TL::logicObjectManager::getAtoms(new_atoms_p, new_predicates(i), objs);
    new_atoms.append(new_atoms_p);
  }
  cout<<"NEW ATOMS:  "<<new_atoms<<endl;
  
  
  
  
  // Action predicates
  TL::Predicate* p_grab = TL::logicObjectManager::getPredicate(MT::String("grab"));
  TL::Predicate* p_puton = TL::logicObjectManager::getPredicate(MT::String("puton"));
  
  // Perform
  for (t=0; t<10; t++) {
    cout<<"TIME-STEP t="<<t<<endl;
    // OBSERVE STATE
    TL::State* s = TL::RobotManipulationDomain::observeLogic(&sim);
    cout<<endl<<"OBSERVED STATE:"<<endl<<*s<<endl;
    
    // ========================================================
    // Calculate Nikolay's predicates [START]
    
    MT::Array< arr > x_objs;
    FOR1D(objs, i) {
      double* position_array = sim.getPosition(objs(i));
      arr position(3);
      position(0) = position_array[0];   position(1) = position_array[1];   position(2) = position_array[2];
      double shape = sim.getSize(objs(i))[0];
      arr x;  x.append(position);  x.append(shape); 
      x_objs.append(x);
      if (DEBUG>1) {cout<<objs(i)<<":  "<<x<<endl;}
    }
    
    
    LitL true_new_literals;
    FOR1D(pns, l) {
      MT::Array< uintA > lists;
      uintA objs_indices;  FOR1D(objs, k) {objs_indices.append(k);}
      TL::allPossibleLists(lists, objs_indices, pns(l)->arity, true, true);
//       PRINT(lists);
      arr g_values;
      
      // Evaluate Nikolay's net
      FOR1D(lists, i) {
//         PRINT(lists(i));
        arr x;
        if (pns(l)->arity == 1) {
          arr x_obj_1 = x_objs(lists(i)(0));
          x.append(x_obj_1);
        }
        else if (pns(l)->arity == 2) {
          arr x_obj_1 = x_objs(lists(i)(0));
          arr x_obj_2 = x_objs(lists(i)(1));
          arr diff = x_obj_1 - x_obj_2;
          x.append(diff);  // TODO absolute value??
        }
        else
          NIY;
        
        PRINT(x);
        
//         FOR1D(lists(i), k) {
//           x.append(x_objs(lists(i)(k)));
//         }
        
        // apply network
        arr z_argument = ~pns(l)->w1a * x + pns(l)->w1b;
        arr z;
        FOR1D(z_argument, k) {
          z.append((1. / ( 1. + exp( z_argument(k) ) )));
        }
        if (DEBUG>2) {PRINT(z);}
        
        arr g_argument = ~pns(l)->w2a * z + pns(l)->w2b;
        if (DEBUG>2) {PRINT(g_argument);}
        
        double g = (1. / ( 1. + exp( g_argument(0)) ));
        double p = g - 0.5;
        
        g_values.append(g);
        
        if (p>0.) {
          uintA args;  FOR1D(lists(i), k) {args.append(objs(lists(i)(k)));}
          true_new_literals.append(TL::logicObjectManager::getLiteral(pns(l)->p, true, args));
        }
        
        if (DEBUG>2) {PRINT(g);  PRINT(p);}
//         exit(0);
      }
      
      FOR1D(lists, i) {
        uintA args;  FOR1D(lists(i), k) {args.append(objs(lists(i)(k)));}
        TL::Atom* atom = TL::logicObjectManager::getAtom(pns(l)->p, args);
        cout<<*atom<<"="<<(g_values(i) - 0.5 > 0)<<"  (g="<<g_values(i)<<") "<<endl;
      }
      
      
      
#if 0
      cout<<"SPECIAL OBJECT HAND"<<endl;
      uint hand_id = sim.getHandId();
      double* hand_position_array = sim.getPosition(hand_id);
      arr hand_x;
      hand_x.append(hand_position_array[0]);  hand_x.append(hand_position_array[1]);  hand_x.append(hand_position_array[2]);
      hand_x.append(sim.getSize(hand_id)[0]);
      cout<<"Hand data vector:  "<<hand_x<<endl;
      FOR1D(pns, l) {
        if (pns(l).arity == 1) {
          // apply network
          arr z_argument = ~pns(l).w1a * hand_x + pns(l).w1b;
          arr z;
          FOR1D(z_argument, k) {
            z.append((1. / ( 1. + exp( z_argument(k) ) )));
          }
          if (DEBUG>2) {PRINT(z);}
          
          arr g_argument = ~pns(l).w2a * z + pns(l).w2b;
          if (DEBUG>2) {PRINT(g_argument);}
          
          double g = (1. / ( 1. + exp( g_argument(0)) ));
          double p = g - 0.5;
          
          g_values.append(g);
          
          if (p>0.) {
            cout<<"HAND:   "<<pns(l).name<<"(hand)=1"<<endl;
          }
        
          if (DEBUG>2) {PRINT(g);  PRINT(p);}
        }
      }
#endif
    }
    
    cout<<endl<<"TRUE new literals ("<<true_new_literals.N<<"):  "<<true_new_literals<<endl;
    
    cout<<endl<<"FALSE new literals:  ";
    FOR1D(new_atoms, i) {
      TL::Literal* lit = TL::logicObjectManager::getLiteral(new_atoms(i));
      if (true_new_literals.findValue(lit) < 0) cout<<*new_atoms(i)<<" ";
      if (i<new_atoms.N-1  &&  new_atoms(i+1)->pred != new_atoms(i)->pred) cout<<endl;
    }
    cout<<endl;
    
    
    // Calculate Nikolay's predicates [END]
    // ========================================================
   
   
   cout<<"Please press button to continue."<<endl;
    sim.watch();
   
    // ACTION
    uintA args;
    args.append(objs(rnd.num(objs.N)));
    TL::Atom* action;
    if (t % 2 == 0) {
      action = TL::logicObjectManager::getAtom(p_grab, args);
    }
    else {
      action = TL::logicObjectManager::getAtom(p_puton, args);
    }
    cout<<endl<<"ACTION #"<<t<<" "<<*action<<endl;
    TL::RobotManipulationDomain::performAction(action, &sim, 50);
    cout<<"----------------------"<<endl;
  }
 
  cout<<"Please press button to continue."<<endl;
  sim.watch();
  
  sim.shutdownAll();
}
#endif











void experiment() {
  // Set up simulator
  MT::String sim_file("situation.ors");
  
  MT::String file_ors;
  MT::getParameter(file_ors, "file_ors");
  
  initSimulator(file_ors, false);
  sim.simulate(50);
//   sim.watch();
  
  // Set up logic
  TL::logicObjectManager::setPredicatesAndFunctions("language.dat");
  TL::logicObjectManager::writeLanguage("used_language.dat");
  
  // Get objects and set in logic database
  uintA objs;
  sim.getObjects(objs);
  objs.append(sim.getHandID());
  TL::logicObjectManager::setConstants(objs);
  
  cout<<"Table-ID="<<sim.getTableID()<<endl;
  cout<<"Hand-ID="<<sim.getHandID()<<endl;
  
  
#if 0
  // Add predicates for predicate networks
  PredL new_predicates;
  FOR1D(pns, i) {
    TL::Predicate* p = new TL::Predicate;
    p->id = TL::logicObjectManager::getLowestFreeConceptID() + i;
    p->name = pns(i).name;
    p->d = pns(i).arity;
    new_predicates.append(p);
    pns(i).p = p;
    if (DEBUG>1) {cout<<*p<<endl;}
  }
  TL::logicObjectManager::addStatePredicates(new_predicates);
#endif
  


  
//   AtomL new_atoms;
//   uint i;
//   FOR1D(new_predicates, i) {
//     AtomL new_atoms_p;
//     TL::logicObjectManager::getAtoms(new_atoms_p, new_predicates(i),
// objs);
//     new_atoms.append(new_atoms_p);
//   }
//   cout<<"NEW ATOMS:  "<<new_atoms<<endl;
  
  
  AtomL actions;
  TL::logicObjectManager::getAtoms_actions(actions, objs); 
  PRINT(actions);
  
  
  // Action predicates
  TL::Predicate* p_grab =
    TL::logicObjectManager::getPredicate(MT::String("grab"));
  TL::Predicate* p_puton =
    TL::logicObjectManager::getPredicate(MT::String("puton"));

  uint t=0;
  
  for (t=0; t<10; t++) {
    TL::Atom* action = actions(rnd.num(actions.N));
    cout<<"ACTION:  "<<*action<<endl;
    TL::RobotManipulationDomain::performAction(action, &sim, 100);
  }

  
}


int main(int argc, char** argv){
  MT::Array< Experience* > experiences;
  read_data(experiences, "data/E1T1-.txt");

  MT::Array<PredicateNetwork*> pns;
  read_PredicateNetworks(pns, "parameters/E1T1-");
  cout<<"PredicateNetworks have been read."<<endl;
  
  uint i;
  FOR1D(experiences, i) {
    calculateSymbols(pns, experiences(i)->state_pre);
  }
  
  exit(0);
  
  MT::String config_file("config");
  cout << "Config-file: " << config_file << endl;
  MT::openConfigFile(config_file);
  
//   test(argc, argv);
  experiment();
  return 0;
}
