#include "cell_life.h"
#include "cell.h"

void CellLife::open() {}

void CellLife::step() {
	uint n_alive=0;
	for(uint i=0;i<neighbours.N;++i) {
		bool alive;
	  neighbours(i)->get_alive(alive, this);
		if(alive) n_alive++;
	}
	if(n_alive<3 || n_alive>4) own->set_alive(false, this);
	if(n_alive==3) own->set_alive(true, this);
}

void CellLife::close() {}
