#ifndef _CELL_VIEWER_H_
#define _CELL_VIEWER_H_

#include <System/biros.h>
#include <Core/array.h>

class Cell;

class CellViewer: public Process {
	public:
    CellViewer() : Process("Cell Viewer Process") {};

    MT::Array<Cell*> cells;

    void open();
    void step();
    void close();
};

#endif

