
#include <MT/process.h>
#include <MT/robot_variables.h>


static int global = 42;
static double* addr;

struct GDB:public Process{
  FILE *fil;
  int pid;

  GDB():Process("GDB"){
    fil=NULL;
  }

  void open(){
    fil=popen("gdb --interpreter=mi2","w");
    fprintf(fil,"set target-async on\n");
    //fprintf(fil,"set pagination off\n");
    fprintf(fil,"set non-stop on\n");
    fflush(fil);
    fprintf(fil,"attach %i &\n",pid);
    fflush(fil);
  }

  void step(){
    fprintf(fil,"-thread-select 1\n");
    fprintf(fil,"interrupt\n");
    fflush(fil);
    MT::wait(1.);
    fprintf(fil,"-stack-list-frames\n");
    fflush(fil);
    fprintf(fil,"frame 2\n");
    fflush(fil);
    fprintf(fil,"info address bluba\n");
    cout <<addr <<endl;
    fprintf(fil,"-var-create myvar * bluba\n");
    //fprintf(fil,"-var-create myvar * *%p\n",addr);
    //printf("-var-create myvar * *%p\n",addr);
    fflush(fil);
    fprintf(fil,"continue &\n");
    fflush(fil);
    fprintf(fil,"-var-evaluate-expression myvar\n");
    fflush(fil);
  }

  void close(){
    //fil <<"interrupt\n detatch" <<endl;
    //MT::wait();
  }
  
  void info(){
    fprintf(fil,"info types\n");
    fflush(fil);
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

  GDB gdb;
  gdb.pid = getpid();
  cout <<"PID = " << gdb.pid <<endl;
  gdb.threadOpen();
  
  for(uint t=0;t<10;t++){  MT::wait(.1);    cout <<t <<endl;  }
  gdb.threadStep();
  for(uint t=0;t<10;t++){  MT::wait(.1);    cout <<t <<endl;  }

  gdb.threadClose();

  cout <<"still here :-) " <<endl;
  
  return 0;
}

