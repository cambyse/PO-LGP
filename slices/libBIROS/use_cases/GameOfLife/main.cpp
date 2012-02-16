#include "cell.h"
#include "cell_life.h"
#include "cell_viewer.h"

int main(int argc, char** argv) {
	srand ( time(NULL) );
	CellViewer viewer;
	for(uint x=0;x<10;++x){
		for(uint y=0;y<10;++y){
			Cell* n = new Cell;
		  viewer.cells.append(n);	
			if(rand()%10<4) n->set_alive(true,NULL);
	  }
  }
	viewer.cells.reshape(10,10);
	for(int x=0;x<10;++x){
		std::cout << x << std::endl;
		for(int y=0;y<10;++y){
			CellLife* life = new CellLife;
			life->own = viewer.cells(x,y);
			for(int nx=x-1;nx<x+2;++x){
				for(int ny=y-1;ny<y+2;++y){
					if(x >= 0 && y >= 0 && x < 10 && y < 10 ) {
			      life->neighbours.append(viewer.cells(nx,ny));
					}
					std::cout<<y<<std::endl;
				}
			}
			life->threadLoop();
		}
	}
	viewer.threadLoop();
	MT::wait();
	

}
