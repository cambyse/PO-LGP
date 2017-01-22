#ifndef _CELL_H_
#define _CELL_H_

#include <biros/biros.h>

class Cell : public AccessData {
	public:
		Cell() : AccessData("Cell") {}
		bool alive;
};

#endif
