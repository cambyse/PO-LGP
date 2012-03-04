#ifndef _GUI_H_
#define _GUI_H_

#include <MT/process.h>

class sGui;

class GuiDataV : public Variable {
  public:
    GuiDataV() : Variable("Gui Data Variable"), sample(NULL) {} ;
    MT::Array<arr>* sample;
};

class Gui : public Process {
  private:
    sGui* s;

  public:
    Gui(const char* orsFile);
      ~Gui();
    void open();
    void step();
    void close();

    GuiDataV* guiData;
};

#endif
