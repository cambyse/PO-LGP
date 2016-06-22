

#include "code.h"

//===========================================================================

void coop(){
  Coop C;

  C.prepareKin();
  C.prepareFol();
  C.prepareTree();
  C.prepareDisplay();

//  C.fol.verbose=5;

  C.expandNode();
//  C.displayTree();

  StringA cmds={ "p", "0", "3", "1"};//, "p", "4", "p", "s", "q" };
  cmds={ "1", "1", "0", "x", "q" };
  bool interactive = mlr::getParameter<bool>("intact", false);

  for(uint s=0;;s++){
    C.updateDisplay();
    C.printChoices();

    if(interactive){
      mlr::String cmd = C.queryForChoice();
      if(!C.execChoice(cmd)) break;
    }else{
      if(!C.execChoice(cmds(s))) break;
    }
  }

  mlr::wait();
  threadCloseModules();
  cout <<"BYE BYE" <<endl;
}

//===========================================================================

int main(int argc,char **argv){
  mlr::initCmdLine(argc,argv);

  orsDrawAlpha = 1.;
  coop();

  return 0;
}
