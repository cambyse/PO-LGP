#define MT_IMPLEMENT_TEMPLATES

#include <relational/robotManipulationSimulator.h>
#include <relational/utilTL.h>


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



void test_simple() {
	
  uint randSeed = 1234;
  rnd.seed(randSeed);
  
  // -------------------------------------
  // Simulator ORS-configuration file
  MT::String sim_file("situation_simple.ors");
  
  // -------------------------------------
  // Start simulator
  initSimulator(sim_file, false);
  sim.simulate(50);

  // -------------------------------------
  // read out some information
  cout<<"GENERAL SIMULATOR INFORMATION:"<<endl;
  uintA objects;
  sim.getObjects(objects);
  cout<<"Objects: "<<objects<<endl;

  uint id_table = sim.getTableID();
  cout<<"Table: "<<id_table<<endl;
  
  uintA blocks;
  sim.getBlocks(blocks);
  cout<<"Blocks: "<<blocks<<endl;
	
  uintA balls;
  sim.getBalls(balls);
  cout<<"Balls: "<<balls<<endl;
  
  cout<<"Sizes:"<<endl;
  uint i;
  FOR1D(objects, i) {
    double* size = sim.getSize(objects(i));
    cout<<objects(i)<<":  "<<size[0]<<endl;
  }

  
  // -------------------------------------
  // perform random actions
  
  uint target_object;
  uint a;
  uint num_actions = 50;
  for (a=0; a<num_actions; a++) {
    // STATE INFO
    cout<<endl<<"STATE:"<<endl;
    FOR1D(objects, i) {
      cout<<objects(i)<<": ";
      uintA on_objects;
      sim.getObjectsOn(on_objects, objects(i));
      cout<<"on_objects="<<on_objects<<"  ";
      if (sim.isUpright(objects(i))) {cout<<"upright  ";}
      if (sim.onGround(objects(i))) {cout<<"onground  ";}
      if (sim.isClear(objects(i))) {cout<<"clear  ";}
      cout<<endl;
    }
    uint inhand_object = sim.getInhand();
    if (sim.getInhand() == TL::UINT_NIL)
      cout<<"Inhand: -"<<endl;
    else
      cout<<"Inhand: "<<inhand_object<<endl;
    cout<<"Please press button to continue."<<endl;
    sim.watch();
    // ACTION
    cout<<endl<<"ACTION:"<<endl;
    // random object
    target_object = objects(rnd.num(objects.N));
    if (a%2 == 0) { // grab
      cout<<"Grabbing "<<target_object<<"."<<endl;
      sim.grab(target_object);
    }
    else { // put on
      cout<<"Putting on "<<target_object<<"."<<endl;
      sim.dropObjectAbove(target_object);
    }
    sim.relaxPosition();
    sim.simulate(50);
  }
  
  cout<<"Please press button to continue."<<endl;
  sim.watch();
  
  sim.shutdownAll();
}


void test_box() {
  
  uint randSeed = 1234;
  rnd.seed(randSeed);
  
  // -------------------------------------
  // Simulator ORS-configuration file
  MT::String sim_file("situation_box.ors");
  
  // -------------------------------------
  // Start simulator
  initSimulator(sim_file, false);
  sim.simulate(50);

  // -------------------------------------
  // read out some information
  cout<<"GENERAL SIMULATOR INFORMATION:"<<endl;
  uintA objects;
  sim.getObjects(objects);
  cout<<"Objects: "<<objects<<endl;

  uint id_table = sim.getTableID();
  cout<<"Table: "<<id_table<<endl;
  
  uintA blocks;
  sim.getBlocks(blocks);
  cout<<"Blocks: "<<blocks<<endl;
  
  uintA balls;
  sim.getBalls(balls);
  cout<<"Balls: "<<balls<<endl;
  
  uintA boxes;
  sim.getBoxes(boxes);
  cout<<"Boxes: "<<boxes<<endl;
  
  cout<<"Sizes:"<<endl;
  uint i;
  FOR1D(objects, i) {
    double* size = sim.getSize(objects(i));
    cout<<objects(i)<<":  "<<size[0]<<endl;
  }

  
  // -------------------------------------
  // perform actions
  
  sim.openBox(67, "opening box 67");
  sim.relaxPosition();
  
  sim.closeBox(66);
  sim.relaxPosition();
  
  sim.grab(70);
  sim.relaxPosition();
  sim.simulate(50);
  
  sim.dropObjectAbove(67);
  sim.relaxPosition();
  sim.simulate(50);
  
  sim.grab(71);
  
  sim.grab(68);
  sim.relaxPosition();
  sim.simulate(50);
  
  sim.dropObjectAbove(66);
  sim.relaxPosition();
  sim.simulate(50);
  
  cout<<"Please press button to finish."<<endl;
  sim.watch();
  
  sim.shutdownAll();
}



int main(int argc, char** argv){
//   test_simple();
  test_box();
  return 0;
}

