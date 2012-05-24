#include <motion/motion.h>
#include <perception/perception.h>
#include <hardware/hardware.h>

#include <views/views.h>
#include <MT/ors.h>
#include <MT/gtk.h>

/* What doesn't work yet:
 
 - collisions with the grasped object UNTIL the 4/5 time using a special proxy variable
 - feedback tasks (like open hand) have not termination criterion - fixed time is not ok!
*/

#include "behaviors.h"

int main(int argn,char** argv){
  MT::initCmdLine(argn, argv);
  //ThreadInfoWin win;
  //win.threadLoopWithBeat(.1);
  
  ProcessL P;

  //-- motion
  // variables
  GeometricState geometricState;
  MotionFuture motions;
  HardwareReference hardwareReference;
  SkinPressure skinPressure;
  JoystickState joystickState;
  
  // processes
  Controller controller;
  ActionProgressor actionProgressor;
  
  // viewers
  OrsViewer<GeometricState>     view0(geometricState);
  PoseViewer<HardwareReference> view8(hardwareReference);

  //-- hardware
  // variables
  //(none)
  // processes
  Joystick joystick;
  SchunkArm schunkArm;
  SchunkHand schunkHand;
  SchunkSkin schunkSkin;
  // viewers
  //(none)

  //-- perception
  // variables
  Image camL("CameraL"), camR("CameraR");
  Image hsvL("HsvL"), hsvR("HsvR");
  FloatImage hsvEviL("hsvEviL"), hsvEviR("hsvEviR");
  PerceptionOutput percOut;
  // processes
  Camera cam;
  CvtHsv cvtHsv1(camL, hsvL);
  CvtHsv cvtHsv2(camR, hsvR);
  HsvFilter hsvFilterL(hsvL, hsvEviL);
  HsvFilter hsvFilterR(hsvR, hsvEviR);
  ShapeFitter shapeFitter(hsvEviL, hsvEviR, percOut);
  // viewers
  ImageViewer<Image> view1(camL), view2(camR);
  ImageViewer<Image> view3(hsvL), view4(hsvR);
  ImageViewer<FloatImage> view5(hsvEviL), view6(hsvEviR);

  P.append(LIST<Process>(controller));
  //P.append(LIST<Process>(joystick, schunkArm, schunkHand, schunkSkin));
  //P.append(LIST<Process>(cvtHsv1, cvtHsv2, hsvFilterL, hsvFilterR, shapeFitter));

  //views don't need to be started -- they now listen!
  ProcessL PV;
  PV.append(LIST<Process>(view0));
  //PV.append(LIST<Process>(view));
  PV.append(LIST<Process>(view8));
  //PV.append(LIST<Process>(view1, view2, view5, view6)); //view3, view4, 
  
  //step(PV);
  //loopWithBeat(PV,.1);

  /////////////////////////////////////////////////////////////////////////////
  //new view stuff

  View *v0 = newView(geometricState, 0);

  gtk_init(&argn, &argv);
  /*GtkBuilder *builder = gtk_builder_new ();
  gtk_builder_add_from_file (builder, "win.glade", NULL);
  GtkWidget *win = GTK_WIDGET(gtk_builder_get_object (builder, "window"));
  GtkWidget *container = GTK_WIDGET(gtk_builder_get_object (builder, "vbox1"));
  gtk_builder_connect_signals (builder, NULL);
  g_object_unref (G_OBJECT (builder));
  gtk_widget_show(win);
  */

  GtkWidget *win = gtk_window_new(GTK_WINDOW_TOPLEVEL);
  gtk_window_set_title(GTK_WINDOW(win), "big window");
  gtk_window_set_default_size(GTK_WINDOW(win), 100, 100);
  gtk_widget_show(win);

  GtkWidget *box = gtk_vbox_new (false, 5);
  gtk_container_add(GTK_CONTAINER(win), box);

  v0->gtkNew(box);
  gtk_main();
  /////////////////////////////////////////////////////////////////////////////

  //cam.threadLoop();
  loopWithBeat(P,.01);

  actionProgressor.threadLoopWithBeat(0.01);
  
  
  cout <<"arrange your windows..." <<endl;
  MT::wait(1.);
  
  //pick-and-place loop
  for(uint k=0;k<2;k++){
    pickOrPlaceObject(Action::grasp, "box1", NULL);
    pickOrPlaceObject(Action::place, "box1", "cyl1");

    pickOrPlaceObject(Action::grasp, "box2", NULL);
    pickOrPlaceObject(Action::place, "box2", "cyl2");
    
    pickOrPlaceObject(Action::grasp, "box1", NULL);
    pickOrPlaceObject(Action::place, "box1", "table");

    pickOrPlaceObject(Action::grasp, "box2", NULL);
    pickOrPlaceObject(Action::place, "box2", "cyl1");
    
    pickOrPlaceObject(Action::grasp, "box1", NULL);
    pickOrPlaceObject(Action::place, "box1", "cyl2");

    pickOrPlaceObject(Action::grasp, "box2", NULL);
    pickOrPlaceObject(Action::place, "box2", "table");
  }
  
  cam.threadClose();
  close(P);

  birosInfo.dump();
  cout <<"bye bye" <<endl;
  return 0;
}
