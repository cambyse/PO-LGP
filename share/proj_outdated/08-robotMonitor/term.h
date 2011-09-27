#include <curses.h>
#include <signal.h>

void termInit(){
  initscr();
  cbreak();
  noecho();
}
void termClose(){
  endwin();
}
