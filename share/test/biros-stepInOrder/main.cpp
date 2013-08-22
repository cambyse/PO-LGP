/*
 * Setup a simple graph of processe:
 *
 *    A
 *   / \
 *  B   C
 *   \ /
 *    D
 *
 * Execute the graph in a given order.
 * The parallelism of biros Processes is explicitly not used!
 */
#include <System/biros.h>
#include <biros/biros_views.h>
#include <string>


// ============================================================================
// VARIBALES
/** This is just an int */
struct IntVar : public Variable {
  FIELD(int, x);
  
  IntVar(const char* name) : Variable(name) {
    reg_x();
    x = 0;
  }
};


// ============================================================================
// PROCESSES

/** Provider simply provides two IntVar. That's it. */
struct ProviderP : public Process {
  IntVar& var4B;
  IntVar& var4C;
  
  ProviderP(const char* name, IntVar& var4B_, IntVar& var4C_)
    : Process(name)
    , var4B(var4B_)
    , var4C(var4C_)
  {}
  
  void open() {}
  void close() {}
  void step() {
    var4B.set_x(1, this);
    var4C.set_x(1, this);
  }
};

/** ForwarderP simply forwards the input variable into the output variable. */
struct ForwarderP : public Process {
  IntVar& in;
  IntVar& out;
  
  ForwarderP(const char* name, IntVar& in_, IntVar& out_)
    : Process(name)
    , in(in_)
    , out(out_)
  {}
  
  void open() {}
  void close() {}
  void step() {
    out.set_x(in.get_x(this), NULL);
  }
};

/** Adder adds the two input variable to the current result. */
struct AdderP : public Process {
  IntVar& in1;
  IntVar& in2;
  IntVar& result;
  
  AdderP(const char* name, IntVar& in1_, IntVar& in2_, IntVar& result_)
    : Process(name)
    , in1(in1_)
    , in2(in2_)
    , result(result_)
  {}
  
  void open() {}
  void close() {}
  void step() {
    int tmp = result.get_x(this) + in1.get_x(this) + in2.get_x(this);
    // math is hard...so wait
    // MT::wait(.7);
    cout << "result is " << tmp << endl;
    result.set_x(tmp , this);
  }
};


// ============================================================================
int main(int argn, char** argv) {
  // print out all biros
  cout << "Biros should be empty" << endl;
  biros().dump();
  
  // create Variables -> variables are connected to biros now
  IntVar varA2B("varA2B");
  IntVar varA2C("varA2C");
  IntVar varB2D("varB2D");
  IntVar varC2D("varC2D");
  IntVar varTotal("varTotal");
  
  // create processes -> processes are connected to biros now
  ProviderP provider("ProviderP", varA2B, varA2C);
  ForwarderP forwarderB("forwarderB", varA2B, varB2D);
  ForwarderP forwarderC("forwarderC", varA2C, varC2D);
  AdderP adderP("adderP", varB2D, varC2D, varTotal);
  
  // put processes in the appropriate execution order
  ProcessL processes(4);
  processes(0) = &provider;
  processes(1) = &forwarderB;
  processes(2) = &forwarderC;
  processes(3) = &adderP;
  
  // create VIEW
  InsideOut insideOutView;
  
  // execute in sequence; does not use any thread functionality.
  Metronome ticcer("ticcer", 1000);
  for (uint j = 0; j < 1000; j++) {
    stepInSequenceThreaded(processes);
    
    cout << "DONE" << endl;
    ticcer.waitForTic();
    cout << "DONE WAITING" << endl;
  }
  
  // INFO
  cout << "\n\nBiros should have some objects" << endl;
  biros().dump();
  
  return 0;
}
