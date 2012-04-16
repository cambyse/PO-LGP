
#include <biros/biros.h>
#include <MT/robot_variables.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

extern VariableL globalVariables;

static int global = 42;
static double* addr;

struct GDB:public Process{
  FILE *fil;
  FILE *out;
  int main_pid;
  char buf[6];

  GDB():Process("GDB"){
    fil=NULL;
  }

  void grapOutput(){
    cout <<"gdbOUT: " <<std::flush;
    int c;
    out=fopen("z.gdbout","r");
    while ((c = fgetc(out)) != EOF){
      putchar (c);
      memmove(buf,buf+1,4);
      buf[4]=c; buf[5]=0;
      if(!strcmp(buf,"(gdb)")){ cout <<":-)" <<endl; break; }
      if(!strcmp(buf,"nning")){ cout <<":-)" <<endl; break; }
    }
    fclose(out);
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
    fprintf(fil,cmd);
    fflush(fil);
    //int r=fread(buffer,1,1000,out);
    //<<buffer <<endl;
    grapOutput();
  }
  void open(){

    static const char gdbInitialize[] =
      "set pagination off\n"
      "set target-async on\n"
      "set non-stop on\n";

//      "set prompt\n"
//       "set verbose off\n"
//       "set editing off\n"
//       "set confirm off\n"
//       "set print static-members off\n"
// 	"set unwindonsignal on\n"

    
    openpty(&fil, NULL, NULL, NULL);

    mkfifo("z.gdbout",S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
    fil=popen("gdb --interpreter=mi2","w");
    grapOutput();
    sendCommand(gdbInitialize);
    sendCommand(STRING("attach " <<main_pid <<"\n"));
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
    sendCommand("interrupt\n detatch");
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

  GDB gdb;
  gdb.main_pid = getpid();
  cout <<"main pid = " << gdb.main_pid <<endl;
  gdb.open();
  cout <<"HERE" <<endl;
  gdb.close();
  cout <<"HERE2" <<endl;
  return 0;
  //gdb.threadOpen();
  
  //for(uint t=0;t<10;t++){  MT::wait(.1);    cout <<t <<endl;  }
  //gdb.threadStep();
  //for(uint t=0;t<10;t++){  MT::wait(.1);    cout <<t <<endl;  }

  //gdb.threadClose();

  cout <<"still here :-) " <<endl;
  
  MT::wait(100.);
  
  return 0;
}

