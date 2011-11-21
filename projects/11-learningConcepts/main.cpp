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


struct PredicateNetwork {
  MT::String name;
  TL::Predicate* p;
  uint arity;;
  arr w1a;
  arr w1b;
  arr w2a;
  arr w2b;
};


#if 0
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
  
#if 0
  // Unary predicate 1
  PredicateNetwork pn_u1;
  pn_u1.arity = 1;
  pn_u1.name << "u1";
  
  ifstream in_p1a1w1a("data_grounding_project/p1a1w1a.txt");
  pn_u1.w1a.read(in_p1a1w1a);
  
  ifstream in_p1a1w1b("data_grounding_project/p1a1w1b.txt");
  pn_u1.w1b.read(in_p1a1w1b);
  
  ifstream in_p1a1w2a("data_grounding_project/p1a1w2a.txt");
  pn_u1.w2a.read(in_p1a1w2a);
  
  ifstream in_p1a1w2b("data_grounding_project/p1a1w2b.txt");
  pn_u1.w2b.read(in_p1a1w2b);
  
  PRINT(pn_u1.w1a);
  PRINT(pn_u1.w1b);
  PRINT(pn_u1.w2a);
  PRINT(pn_u1.w2b);
  
  pns.append(pn_u1); 
#endif
  
  
  // Binary predicate 1
  PredicateNetwork pn_b1;
  pn_b1.arity = 2;
  pn_b1.name << "b1";
  
  ifstream in_p1a2w1a("data_grounding_project/p1a2w1a.txt");
  pn_b1.w1a.read(in_p1a2w1a);
  
  ifstream in_p1a2w1b("data_grounding_project/p1a2w1b.txt");
  pn_b1.w1b.read(in_p1a2w1b);
  
  ifstream in_p1a2w2a("data_grounding_project/p1a2w2a.txt");
  pn_b1.w2a.read(in_p1a2w2a);
  
  ifstream in_p1a2w2b("data_grounding_project/p1a2w2b.txt");
  pn_b1.w2b.read(in_p1a2w2b);
  
  PRINT(pn_b1.w1a);
  PRINT(pn_b1.w1b);
  PRINT(pn_b1.w2a);
  PRINT(pn_b1.w2b);
  
  pns.append(&pn_b1);
  
#if 0
  // Unary predicate 2
  PredicateNetwork pn_u2;
  pn_u2.arity = 1;
  pn_u2.name << "u2";
  
  ifstream in_p2a1w1a("data_grounding_project/p2a1w1a.txt");
  pn_u2.w1a.read(in_p2a1w1a);
  
  ifstream in_p2a1w1b("data_grounding_project/p2a1w1b.txt");
  pn_u2.w1b.read(in_p2a1w1b);
  
  ifstream in_p2a1w2a("data_grounding_project/p2a1w2a.txt");
  pn_u2.w2a.read(in_p2a1w2a);
  
  ifstream in_p2a1w2b("data_grounding_project/p2a1w2b.txt");
  pn_u2.w2b.read(in_p2a1w2b);
  
  PRINT(pn_u2.w1a);
  PRINT(pn_u2.w1b);
  PRINT(pn_u2.w2a);
  PRINT(pn_u2.w2b);
  
  pns.append(pn_u2);
  
  
  // Binary predicate 2
  PredicateNetwork pn_b2;
  pn_b2.arity = 2;
  pn_b2.name << "b2";
  
  ifstream in_p2a2w1a("data_grounding_project/p2a2w1a.txt");
  pn_b2.w1a.read(in_p2a2w1a);
  
  ifstream in_p2a2w1b("data_grounding_project/p2a2w1b.txt");
  pn_b2.w1b.read(in_p2a2w1b);
  
  ifstream in_p2a2w2a("data_grounding_project/p2a2w2a.txt");
  pn_b2.w2a.read(in_p2a2w2a);
  
  ifstream in_p2a2w2b("data_grounding_project/p2a2w2b.txt");
  pn_b2.w2b.read(in_p2a2w2b);
  
  PRINT(pn_b2.w1a);
  PRINT(pn_b2.w1b);
  PRINT(pn_b2.w2a);
  PRINT(pn_b2.w2b);
  
  pns.append(pn_b2);
#endif
  
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
  MT::String config_file("config");
  cout << "Config-file: " << config_file << endl;
  MT::openConfigFile(config_file);
  
//   test(argc, argv);
  experiment();
  return 0;
}
