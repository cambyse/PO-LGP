#ifndef _CELL_H_
#define _CELL_H_

#include <biros/biros.h>

class Cell : public Variable {
	public:
		Cell() : Variable("Cell") {}
		FIELD(bool, alive);
};

#endif
