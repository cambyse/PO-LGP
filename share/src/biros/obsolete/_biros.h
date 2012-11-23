//===========================================================================
//
// very basic low-level X11 Monitor
//

struct ThreadInfoWin:public Process {
  struct sThreadInfoWin *s;
  
  ThreadInfoWin();
  ~ThreadInfoWin();
  
  void open();
  void close();
  void step();
};


