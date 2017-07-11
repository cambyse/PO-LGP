#ifndef _CELL_LIFE_H_
#define _CELL_LIFE_H_

#include <Core/thread.h>
#include <Core/array.h>
#include "cell.h"

struct CellLife : Thread {
  Access<Cell> own;
  mlr::Array<Access<Cell> > neighbours;

 CellLife() : Thread("Cell Life Process"){ neighbors.resize(4); }
  
  void open();
  void step();
  void close();
};


#endif
