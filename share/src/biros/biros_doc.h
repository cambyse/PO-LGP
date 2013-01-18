/** @mainpage Biros

Biros is a communication framework for application in robotics.


@section biros_intro Introduction

The basic components of Biros are:
- A Variable is a dump object which holds some data from a calculation.
- The data of a Variable is normally defined as a #FIELD. The #FIELD macro creates some convenient functions (getter/setter) and registers the member variable.
- A Process (not as in thread and process) does some calculations and exchanges information via Varibales.
- A Parameter is similar to the MT::getParameter functions.

To create your own Processes and/or Variables you simply have to inherit from Process and/or Variable.
Then you need to inject the Variables into the Processes.

Variables and Processes create a graph.
The actual class Biros allows control and access about the entire graph of variables and processes.

Most of the core structures are in defined in biros.h.

Events are used to control the execution of the Processes.


@section biros_debugging Debugging

There exist several debug views (via gtk) for Parameters, Variables (their FIELDS), and Processes:
see biros_views.h.

@subsection biros_debugging_event EventControl
Because biros uses events to trigger the steps of the Processes, it is easy to
control the execution. View EventControlView offers a way to do this graphically.


@subsection biros_debugging_visualization Visualizing Biros Objects
The View InsideOut allows visualize specific objcet in the biros graph via their specific views.
See share/src/birso_insideout.cpp and share/src/biros_views.h for more on that.


@section biros_examples Examples

More examples can be found under share/src/biros*

*/
