

struct World{
  arr q; ///< true current DOFs: pose of effector
  arr x; ///< poses of all bodies: m times 3
  arrA shapes; ///< shapes per body: m times m'x5 dim

  //-- constraints
  boolA pushContact; //mxm
  boolA rigidContact; //mxm



  void glDraw(OpenGL&);

};
