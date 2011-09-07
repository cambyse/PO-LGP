#define MT_IMPLEMENTATION

#include <MT/schunk.h>
#include <MT/guiModule.h>
#include <signal.h>
#include<SD/skin.h>
#include<SD/utils.h>

/*
 * Example use of the wrapper around the skin module.
 */

int usrbreak = 0;
static void
usrbreak_callback(int){
  usrbreak=1;
}

/* read skin from file */
void
fakesense(grobi_skin_t &gs){
  byteA tht,thl,f3t,f3l,f2t,f2l;
  uintA itht,ithl,if3t,if3l,if2t,if2l;
  uint i,j;

  MT::openConfigFile();

  getFromCfgFile(itht,"th-tip");
  getFromCfgFile(ithl,"th-link");
  getFromCfgFile(if2t,"f2-tip");
  getFromCfgFile(if2l,"f2-link");
  getFromCfgFile(if3t,"f3-tip");
  getFromCfgFile(if3l,"f3-link");
  itht.reshape(13,6); if2t.resizeAs(itht); if3t.resizeAs(itht);
  tht.resize(13,6); f2t.resizeAs(tht); f3t.resizeAs(tht);
  FOR2D(itht,i,j){
    tht(i,j)=(byte)itht(i,j);
    f2t(i,j)=(byte)if2t(i,j);
    f3t(i,j)=(byte)if3t(i,j);
  }
  ithl.reshape(14,6); if2l.resizeAs(ithl); if3l.resizeAs(ithl);
  thl.resize(14,6); f2l.resizeAs(thl); f3l.resizeAs(thl);
  FOR2D(ithl,i,j){
    thl(i,j)=(byte)ithl(i,j);
    f2l(i,j)=(byte)if2l(i,j);
    f3l(i,j)=(byte)if3l(i,j);
  }

  gs.th->tip.sensebypass(tht);
  gs.th->link.sensebypass(thl);
  gs.f2->tip.sensebypass(f2t);
  gs.f2->link.sensebypass(f2l);
  gs.f3->tip.sensebypass(f3t);
  gs.f3->link.sensebypass(f3l);

  SD_DBG("th-tip: "<<tht);
}

void
test_hardware_skin(GuiModule &gui, grobi_skin_t &gs){

  SchunkSkinModule skin;
  byteA img;
  
  skin.open();
  for(;!usrbreak;){ //catches the ^C key
    skin.step();
    gui.step();

    skin.getImage(img); /* we can either get the img once and provide it to gs... */
    gs.sense(NULL, &img, NULL);

    gs.update_shapes();
  }
  skin.close();
}

void
test_fake_skin(GuiModule &gui, grobi_skin_t &gs){
  for(;!usrbreak;){ //catches the ^C key
    gui.step();
    fakesense(gs);
    gs.update_shapes();
  }
}

int
main(int argn,char** argv){
  GuiModule gui;
  ors::Graph ors;

  MT::initCmdLine(argn,argv);
  signal(SIGINT,usrbreak_callback);
  
  MT::load(ors,"schunk.ors",true);
  ors.calcBodyFramesFromJoints();
  gui.createOrsClones(&ors);
  
  grobi_skin_t gs(*gui.ors);

  gui.open();

  // loop 
  if (MT::getParameter<int>("openSkin"))
    test_hardware_skin(gui, gs);
  else 
    test_fake_skin(gui, gs);

  gui.close();
}
