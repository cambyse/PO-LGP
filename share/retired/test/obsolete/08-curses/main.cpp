#define MT_IMPLEMENTATION

#include <curses.h>
#include <signal.h>
#include <Core/util.h>

struct Curses{
  WINDOW *win;
  
  void open(){
    initscr();
    cbreak();
    noecho();

    win = newwin(100,100,10,10);//, width, starty, startx);
    box(win, 0 , 0);          /* 0, 0 gives default characters
    * for the vertical and horizontal
                                    * lines                        */
    wrefresh(win);            /* Show that box                */
  }
  void close(){
    endwin();
  }
  void step(){
    wrefresh(win);            /* Show that box                */
  }

};


int main(int argc,char** argv){
  Curses curses;
  curses.open();
  mvwprintw(curses.win,0,0,"*** MONITOR ***");
  curses.step();
  MT::wait(2.);
  curses.close();
  return 0;
}
