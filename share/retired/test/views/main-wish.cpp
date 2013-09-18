#include <Gui/opengl.h>

int main(int argn,char** argv){
  MT::initCmdLine(argn, argv);
  
  arr X = randn(5,3);

#if 0
  //-- directly creating a view of a specific type, which the user knows:
  View *v = newView<MatrixView>(X);
  //this directly OPENs a window

  //allow to also give a container
  View *v = newView<MatrixView>(X, (GtkWidget*)box);

  //allow to give a RW-Lock?
  View *v = newView<MatrixView>(X, (GtkWidget*)box, (RWLock&)lock);


  //-- accessing a list of available views
  ViewRegistrationL views = getAvailableViews<arr>();
  //using this to instantiate a new view
  View *v = newView(views(2), X, (GtkWidget*)box, (RWLock&)lock);
#endif

  OpenGL gl;

  for(uint t=0;t<100;t++){
    //while looping, the view should autonomously update its content,
    //with the update frequency of the gtkProcess()
    X += .1*randn(5,3);
    gl.displayGrey(X, false, 10);
    MT::wait(.1);
  }

  return 0;
}

/*
Further points:

1) Can one get rid of the AppliesOnT template parameter for ViewRegistration_typed ?

2) remove all view related stuff from biros/control.*

3) rename 'ViewRegistration' to something more descriptive? E.g. ViewRegistration?

4) Is it ever necessary, given the ViewerType (e.g., MeshView), to access its global ViewRegistration_typed struct? If not, leave as is. If yes, there should be some static points within ViewRegistration_typed
*/
