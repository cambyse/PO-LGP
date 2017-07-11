#ifndef _CELL_H_
#define _CELL_H_

#include <biros/biros.h>

class Cell : public VariableData {
	public:
		Cell() : VariableData("Cell") {}
		bool alive;
};

#endif
