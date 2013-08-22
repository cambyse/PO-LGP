#ifndef _CELL_H_
#define _CELL_H_

#include <System/biros.h>

class Cell : public Variable {
	public:
		Cell() : Variable("Cell") {reg_alive();}
		FIELD(bool, alive);
};

#endif
