
#include <MT/process.h>
#include <MT/robot_variables.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <pty.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <pty.h>
#include <utmp.h>
#include <ctype.h>
#include <errno.h>
  #include <sys/select.h>

extern VariableL globalVariables;

static int global = 42;
static double* addr;

struct GDB:public Process{
  int fil;
  int main_pid;
  char buf[256];

  GDB():Process("GDB"){
    fil=NULL;
  }

  void grapOutput(){
    cout <<"gdbOUT: " <<std::flush;
    for(;;){
      fd_set readFds;
      timeval tv; tv.tv_sec=1; tv.tv_usec = 0;
      FD_ZERO (&readFds);
      FD_SET (fil, &readFds);
      select(fil+1,&readFds, NULL, NULL, &tv);
      if(!FD_ISSET(fil, &readFds)) break;
      int ret=read(fil, buf, 255);
      if(!ret) break; //nothing was read
      if(ret==-1) HALT("reading from gdb output failed: " <<strerror(errno) <<'(' <<errno <<')');
      buf[ret] = '\0';
      puts(buf);
      if(!strcmp("(gdb) \r\n",buf+ret-8)) break;
    }
  }

//     int pos=ftell(out);
//     fseek(out,0,SEEK_END);
//     int len=ftell(out);
//     cout <<"pos=" <<pos <<"len=" <<len <<endl;
//     while(ftell(out)<len-2){
//       int c=fgetc(out);
//       //if(c==EOF) break;
//       cout <<(char)c;
//     }
//     clearerr(out);
//   }

  void sendCommand(const char* cmd){
    cout <<"gdbIN : " <<cmd;
    write(fil, cmd, strlen(cmd));
    //fflush(fil);
    //int r=fread(buffer,1,1000,out);
    //<<buffer <<endl;
    grapOutput();
  }
  
  void open(){

    static const char gdbInitialize[] =
      "-gdb-set pagination off\n"
      "-gdb-set non-stop on\n"
      "-gdb-set target-async on\n"
      "-list-target-features\n"
      ;

//      "set prompt\n"
//       "set verbose off\n"
//       "set editing off\n"
//       "set confirm off\n"
//       "set print static-members off\n"
// 	"set unwindonsignal on\n"

    int pid=forkpty(&fil, NULL, NULL, NULL);
    if(pid==-1) HALT("forking the gdb into a PTY failed!");

    if(!pid){
      //inside the child process: start gdb, execlp will not return until gdb quits
      //--switch off terminal echo
      struct termios orig_termios;
      if (tcgetattr (STDIN_FILENO, &orig_termios) < 0) {
        perror ("ERROR getting current terminal's attributes");
        return;
      }
      orig_termios.c_lflag &= ~(ECHO | ECHOE | ECHOK | ECHONL);
      orig_termios.c_oflag &= ~(ONLCR);
      if (tcsetattr (STDIN_FILENO, TCSANOW, &orig_termios) < 0) {
        perror ("ERROR setting current terminal's attributes");
        return;
      }
      system("gdb --interpreter=mi2");
      exit(0);
    }

    cout <<"GDB pid = " <<pid <<endl;
    //(outside the child process):
    grapOutput();
    sendCommand(gdbInitialize);
    sendCommand(STRING("-target-attach " <<main_pid <<"&\n"));
    sendCommand("-list-target-features\n");
  }

  void step(){
    sendCommand("-thread-select 1\n");
    sendCommand("-exec-interrupt\n");
    MT::wait(1.);
    sendCommand("-stack-list-frames\n");
    sendCommand("-stack-select-frame 2\n");
    sendCommand("-var-create varList * globalVariables\n");
    sendCommand("-var-create var1 * *globalVariables.p[0]\n");
    //fprintf(fil,"-var-create myvar * *%p\n",addr);
    //printf("-var-create myvar * *%p\n",addr);
    sendCommand("-exec-continue\n");
    sendCommand("-var-list-children varList\n");
    sendCommand("-var-list-children varList.public\n");
    sendCommand("-var-evaluate-expression varList.public.p\n");
    sendCommand("-var-evaluate-expression var1\n");
    sendCommand("-var-list-children var1\n"); 
    sendCommand("-var-list-children var1.public\n");
    sendCommand("-var-evaluate-expression var1.public.name\n");

  }

  void close(){
    sendCommand("-target-detatch");
    //MT::wait();
  }
  
  void info(){
    sendCommand("info types\n");
    //fil <<"interrupt" <<endl;
    //fil <<"" <<endl;
    //fprintf(fil,"-var-create - * global\n");
    //fflush(fil);
    //fprintf(fil,"-exec-continue\n");
    //fflush(fil);
  }
};


int main(int argc, char *argv[]){

  double bluba=1.2;
  addr = &bluba;
  
  cout <<&bluba <<endl;

  q_currentReferenceVar q;

#if 0
  if(argc==1){
    cout <<"this pid = " <<getpid() <<endl;
    for(uint t=0;t<100;t++){
      cout <<t <<endl;
      MT::wait(1.);
    }
    return 0;
  }
#endif

  GDB gdb;
  gdb.main_pid = getpid();
  if(argc>1) gdb.main_pid = atoi(argv[1]);
  
  cout <<"main pid = " << gdb.main_pid <<endl;
  gdb.threadOpen();
  
  for(uint t=0;t<10;t++){  MT::wait(1);    cout <<t <<endl;  }
  gdb.threadStep();
  for(uint t=0;t<10;t++){  MT::wait(1);    cout <<t <<endl;  }

  cout <<"wait" <<endl;
  MT::wait(5.);
  
  gdb.threadClose();
  
  cout <<"still here :-) " <<endl;
  
  return 0;
}

