#ifndef _CELL_LIFE_H_
#define _CELL_LIFE_H_

#include <Core/module.h>
#include <Core/array.h>
#include "cell.h"

struct CellLife : Module {
  Access_typed<Cell> own;
  MT::Array<Access_typed<Cell> > neighbours;

 CellLife() : Module("Cell Life Process"){ neighbors.resize(4); }
  
  void open();
  void step();
  void close();
};


#endif
