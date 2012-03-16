#include "cell_viewer.h"
#include "cell.h"

void CellViewer::open() {}
void CellViewer::step() {
  std::cout << std::endl << "----" <<std::endl << std::endl;
  for(uint x=0;x<cells.d0;++x) {
    std::cout<<"|";
    for(uint y=0;y<cells.d1;++y){
      bool alive=false;
      cells(x,y)->get_alive(alive, this);
      if(alive) std::cout<<"X";
      else std::cout<<" ";
    }
    std::cout<<"|\n";
  }
}
void CellViewer::close() {}
