# THINGS in our mlr repos
Writtten <2013-12-05>

This document is a bullet point list of features/functionalities/methods we
have *somewhere* implemented in our repos. It is *not* meant as a documentation
of these features. Instead it is meant to get an overview and let you find the
right keywords to search for in the repos.

Please add features/things you have provides. But keep it a short and keywordy
as possible. No long documentations! At most, use `code formatting` to indicate
keywords to point to the code.

----------------------------------------------------------------------------------------------------------------------

## Optimization
### problem abstractions
* scalar function, vector function
* constrained (non-linear) problem
* $k$-order Markov function (including constraints)
* converter (reductions) between problem formulations `Convert`

### blackbox methods
* stochastic search, CMA

### gradient methods
* Rprop, plain adaptive gradient descent

### 2nd order
* Newton, Gauss-Newton

### constrained optimization
* penalty, log-barrier, augmented Lagrangian

----------------------------------------------------------------------------------------------------------------------

## Robot Kinematics & Dynamics
* standard forward kinematics
* Featherstone's articulated body dynamics, Newton-Euler
* Interfaces to PhysX, ODE, SWIFT

----------------------------------------------------------------------------------------------------------------------

##Control & Trajectories
### Trajectory optimization
* abstracted in terms of $k$-order Markov -> use any optimizer
* Task variables can impy hard constraints -> constrained optimizers

### Stochastic Optimal Control
* old code on AICO, iLQG
* Ricatti equation (e.g. for locally LQG approximation around an optimal trajectory)

### Feedback control
* operational space control

----------------------------------------------------------------------------------------------------------------------

## Perception
### OpenCV wrappers (as modules)
* OpenCV camera (USB or webcams)
* for color conversions
* Canny, keypoint and SURF extraction
* houghline fitter

### Image segmentation
* Felzenzwalb's patcher
* color segmentation/filter (based on HSV reference)
* a parameteric 2D shape fitter (using Rprop) on top of a color filter

### Camera calib
* optimization of camera projection matrix
* extraction of kalibration and pose matrix
* lens distortion optimizer missing
* stereo calibration and depth computation

### Point clouds
* Code by Johannes?
* Code by Oliver Erler
* point cloud viewers

### Meshes
* mesh manipulation (refining, rescaling, cleaning, etc)
* convex decomposition
* loaders of many mesh file formats
* computing implicit surfaces (Lewiner's code)

### Video encoding
* module to encode images into videos using libav (x264)
* and OpenCV video writer (is broken recently)

----------------------------------------------------------------------------------------------------------------------

## Basic Algorithms
* Runge-Kutta
* RRTs
* Approximate Nearest Neighbor

----------------------------------------------------------------------------------------------------------------------

## Systems
* Threads, Mutex, Condition Variables
* our Module system, event logging, event controlling

----------------------------------------------------------------------------------------------------------------------

## Reinforcement Learning
* POMDP solvers (optimizing FSCs, old UAI code)
* Tony's MDP lib
* basic MDP methods (VI, policy iteration)

----------------------------------------------------------------------------------------------------------------------

## Probabilistic Inference
* Factor Graphs, higher-order tensor operations
* Junction Tree, loopy belief propagation
* HMMs, standard forward-backward
* Kalman Filter

----------------------------------------------------------------------------------------------------------------------

## Gui
* OpenGL implementations for gtk, fltk, freeglut, Qt
* graphviz (dot) for displaying graphs (with gtk)
* basic plotting (with gnuplot or OpenGL)
* more advanced plotting using QtCustomPlot (in Philipp's code)

----------------------------------------------------------------------------------------------------------------------

## Geometry
* qhull wrappers
* For 3D: Vector, quaternions (conversion from Euler, matrices, etc), Transformations

### collision detection
* SWIFT, RAPID, SOLID
* GJK (Gilbert, Johnson, Keerthi) to compute closest points on convex hulls

----------------------------------------------------------------------------------------------------------------------

## Hardware drivers
* Schunk
* Huyuku (Laser scanner)
* Dynamixel
* Kinect
* uEye (IDS) cameras
* BumbleBee (point grey)
* usbreset

