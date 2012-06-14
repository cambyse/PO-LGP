#define MT_IMPLEMENT_TEMPLATES

#include <relational/reason.h>
#include <relational/robotManipulationSymbols.h>
#include <relational/robotManipulationInterface.h>


// -------------------------------------
// The simulator object
RobotManipulationSimulator sim;


// -------------------------------------
// Init routine
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


void test() {
  uint randSeed = 12345;
  rnd.seed(randSeed);
  
  // -------------------------------------
  // Start simulator
  MT::String sim_file("situation_box.ors");
  initSimulator(sim_file, false);
  sim.simulate(50);
  
  
  // -------------------------------------
  // Read logic symbols
  relational::SymL symbols;
  relational::ArgumentTypeL types;
  relational::readSymbolsAndTypes(symbols, types, "symbols_box.dat");
  
  relational::writeSymbolsAndTypes("used_symbols.dat");
  
  // -------------------------------------
  // Get objects and set as logic constants
  uintA constants;
  sim.getObjects(constants);
  relational::reason::setConstants(constants);
  
  // -------------------------------------
  // Sequence of actions to execute
  relational::LitL actions;
  relational::Literal::get(actions, "openBox(67) closeBox(66) grab(69) puton(67) grab(70) puton(68)");
  cout<<"Actions: "<<actions<<endl;
  
  // -------------------------------------
  // Execute actions
  uint a;
  relational::SymbolicState* s_old;
  FOR1D(actions, a) {
    cout<<"Time-step t="<<a<<endl;
    // observe state
    relational::SymbolicState* s = relational::RobotManipulationInterface::calculateSymbolicState(&sim);
    cout<<endl<<"Observed symbolic state:"<<endl<<*s<<endl;
    relational::RobotManipulationSymbols::writeStateInfo(*s);
    if (a>0) { // just for visualisation here
      relational::StateTransition transition(*s_old, actions(a-1), *s);
      cout<<endl<<"Changes: "<<transition.changes<<endl;
    }
//     cout<<"Please press button to continue."<<endl;
//     sim.watch();
    sim.simulate(100);

    // execute action
    cout<<endl<<"Action #"<<a<<" "<<*actions(a)<<endl;
    relational::RobotManipulationInterface::executeAction(actions(a), &sim, 50);
    cout<<endl<<"----------------------"<<endl;
    s_old = s;
  }
 
  cout<<"Please press button to continue."<<endl;
  sim.watch();
  
  sim.shutdownAll();
}


int main(int argc, char** argv){
  test();
  return 0;
}

