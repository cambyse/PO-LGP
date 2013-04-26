#include "qt.h"
#include "util.h"
#include <QtGui/QApplication>

struct QtThread *global_qtThread=NULL;

//void gtkLock(bool checkInitialized){
//  if(checkInitialized) gtkCheckInitialized();
//  if(callbackMutex.state && callbackMutex.state==syscall(SYS_gettid)) return;
//  gdk_threads_enter();
//}

//void gtkUnlock(){
//  if(callbackMutex.state && callbackMutex.state==syscall(SYS_gettid)) return;
//  gdk_threads_leave();
//}

struct QtThread:Thread{
  QApplication *app;
  void main(){
    app->exec();
  }
};

void qtCheckInitialized(){
  static Mutex m;
  if(!global_qtThread){
    m.lock();
    if(!global_qtThread){
      int argc=1;
      char **argv = new char*[1];
      argv[0] = (char*)"x.exe";

//      XInitThreads();
//      g_thread_init(NULL);
//      gdk_threads_init();
//      gdk_threads_enter();
//      gtk_init(&argc, &argv);
//      gtk_gl_init(&argc, &argv);
//      glutInit(&argc, argv);

      global_qtThread = new QtThread();
      global_qtThread->app = new QApplication(argc, argv);
      global_qtThread->open("--QT-loop");
    }
    m.unlock();
  }
}

