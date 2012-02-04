#define MT_IMPLEMENT_TEMPLATES
#include <MT/util.h>
#include <MT/process.h>
//#include "process_monitor.h"
#include <gtk/gtk.h>
#include <MT/process_internal.h>


using namespace std;

//===========================================================================
//
// test excessive access to Variables
//

struct IntVar:public Variable{
  //BIR_VARIABLE;
  
  FIELD(int, x);
  
  //BIR_FIELD(bool, mybool);
  
  IntVar():Variable("IntVar"){ x=rnd(1000); reg_x(); }
};

//int IntVar::bir_typeId=-1;

uint PC=0;

struct Maxxer:public Process{
  IntVar *a,*b;
  
  Maxxer():Process("Maxxer"){};
  
  void open (){}
  void close(){}
  void step (){
    int xa=a->get_x(this);
    int xb=b->get_x(this);
    if(xa>xb) b->set_x(xa, this);
    else a->set_x(xb, this);
  }
};

void addVariablesToTree(GtkTreeStore *store){
  GtkTreeIter iter1;  /* Parent iter */
  GtkTreeIter iter2;  /* Child iter  */

  uint i,j;
  Variable *v;
  _Variable_field_info_base *vi;
  global.readAccess(NULL);
  for_list(i, v, global.variables){
    gtk_tree_store_append (store, &iter1, NULL);  /* Acquire a top-level iterator */
    gtk_tree_store_set (store, &iter1,
                        0, v->name,
                        1, STRING("id=" <<v->id <<" type=" <<typeid(*v).name() <<" state=" <<v->lockState()),
                        -1);
    for_list(j, vi, v->fields){
      gtk_tree_store_append (store, &iter2, &iter1);  /* Acquire a child iterator */
      gtk_tree_store_set (store, &iter2,
                          0, vi->name,
                          1, vi->type().p,
                          -1);
    }
  }
  global.deAccess(NULL);
}

void gui(){
  //load glade gui
  GtkBuilder *builder = gtk_builder_new ();
  gtk_builder_add_from_file (builder, "win.glade", NULL);
  GtkWidget *win2 = GTK_WIDGET(gtk_builder_get_object (builder, "window"));
  GtkWidget *tree = (GtkWidget*) gtk_builder_get_object(builder, "VariableTree");
  GtkTreeStore *store = (GtkTreeStore*) gtk_builder_get_object (builder, "VariableTreeStore");
  gtk_builder_connect_signals (builder, NULL);          
  g_object_unref (G_OBJECT (builder));
  
  //add data
  addVariablesToTree(store);
  
  //show
  gtk_widget_show (win2);       
  
  //loop
  gtk_main ();
}

void testMultiAccess(){
  uint n=MT::getParameter<uint>("n",100);
  MT::Array<IntVar> vars(n);
  MT::Array<Maxxer> procs(2*n);
  
  for(uint i=0;i<procs.N;i++){
    procs(i).a = &vars.rndElem();
    procs(i).b = &vars.rndElem();
  }
  
  for(uint i=0;i<procs.N;i++) procs(i).threadLoopWithBeat(rnd.uni(.001,.01));
  MT::wait(1.);
  for(uint i=0;i<procs.N;i++) procs(i).threadClose();
  
  for(uint i=0;i<vars.N;i++) cout <<vars(i).x <<' ';
  cout <<endl;
  
  gui();
}


int main(int argc,char **argv){
  gtk_init (&argc, &argv);
  
  testMultiAccess();
  
  return 0;
}
    