#include "cell.h"
#include "cell_life.h"
#include "cell_viewer.h"

int main(int argc, char** argv) {

  const int grid_x = 40;
  const int grid_y = 80;
	srand ( time(NULL) );
	CellViewer viewer;
	for(int x = 0; x < grid_x; ++x){
		for(int y = 0; y < grid_y; ++y){
			Cell* n = new Cell;
		  viewer.cells.append(n);	
			if(rand() % 100 < 7) 
        n->set_alive(true, NULL);
      else
        n->set_alive(false, NULL);
	  }
  }
	viewer.cells.reshape(grid_x, grid_y);
  MT::Array<CellLife*> lifes;
	for(int x = 0; x < grid_x; ++x){
		for(int y = 0; y < grid_y; ++y){
			CellLife* life = new CellLife;
			life->own = viewer.cells(x,y);
			for(int nx = x - 1; nx < x + 2; ++nx) {
				for(int ny = y - 1; ny < y + 2; ++ny) {
					if(nx >= 0 && ny >= 0 && nx < grid_x && ny < grid_y ) {
			      life->neighbours.append(viewer.cells(nx,ny));
					}
				}
			}
			lifes.append(life);
		}
	}
  for (uint i = 0; i < lifes.N; ++i)
    lifes(i)->threadLoopWithBeat(0.3);
	viewer.threadLoopWithBeat(0.3);
	MT::wait();
	viewer.threadStop();
	viewer.threadClose();
  for (uint i = 0; i < lifes.N; ++i) {
    lifes(i)->threadStop();
    lifes(i)->threadClose();
  }

  for (uint i = 0; i < lifes.N; ++i)
    delete lifes(i);
}
