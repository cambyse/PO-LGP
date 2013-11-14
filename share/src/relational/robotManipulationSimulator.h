/*  
    Copyright 2008-2012   Tobias Lang
    
    E-mail:    tobias.lang@fu-berlin.de
    
    This file is part of libARMANI.

    libARMANI is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    libARMANI is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with libARMANI.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef robotManipulationSimulator_h
#define robotManipulationSimulator_h

#include <Core/array.h>
#include <Ors/ors.h>
#include <Ors/ors_oldTaskVariables.h>



/************************************************
 * 
 *     ODE parameters
 * 
 ************************************************/

// Object bouncing when falling on table or other object.
#define ODE_COLL_BOUNCE 0.001
// Stiffness (time-scale of contact reaction) in [0.1, 0.5]; the larger, the more error correction
#define ODE_COLL_ERP 0.3
// Softness in [10e-10, 10e5]; the larger, the wider, the softer, the less error correction
#define ODE_COLL_CFM 10e0
// Friction (alternative: dInfinity)
#define ODE_FRICTION 0.02




namespace ors{
  struct Graph;
}
class OdeInterface;
struct SwiftInterface;
struct OpenGL;
struct RevelInterface;



class RobotManipulationSimulator {
public:
  
 /************************************************
  * 
  *     Administration
  * 
  ************************************************/
 
   RobotManipulationSimulator();
  ~RobotManipulationSimulator();
  
  // initialization
  void loadConfiguration(const char* ors_filename);
  void startOde(double ode_coll_bounce = ODE_COLL_BOUNCE, double ode_coll_erp = ODE_COLL_ERP, double ode_coll_cfm = ODE_COLL_CFM, double ode_friction = ODE_FRICTION);
  void startSwift();
  void startRevel(const char* filename = "z.avi");
  
  // shutdown
  void shutdownAll();
  
  
  
 /************************************************
  * 
  *     Standard simulation
  * 
  ************************************************/
   
  void simulate(uint t, const char* message = "");
  void watch();
  void indicateFailure();
 
  
  
 /************************************************
  * 
  *     General object information (state-independent)
  * 
  ************************************************/
 
  void calcObjectNumber();  // do only once
  void getObjects(uintA& objects); ///< return list all objects
  uint getTableID();
  void getBlocks(uintA& blocks);
  void getBalls(uintA& balls);
  void getBoxes(uintA& boxes);
  void getCylinders(uintA& cylinders);
  bool isBox(uint id);
  uint convertObjectName2ID(const char* name); ///< returns the graph-index of the object named "name"
  const char* convertObjectID2name(uint ID); ///< returns the name of the object with index "ID" in graph
#define OBJECT_TYPE__BOX 6
  int getOrsType(uint id); // this is the ORS type
  double* getSize(uint id);     ///< when using symmetric objects, take getSize(id)[0]
  double* getColor(uint id);    ///< returns 3-dim array with RGB values
  MT::String getColorString(uint obj); 
  
  
  
 /************************************************
  * 
  *     State information
  * 
  ************************************************/

  // object specific
  double* getPosition(uint id);   ///< returns 3-dim array with x/y/z-coordinates
  void getTablePosition(double& x1, double& x2, double& y1, double& y2);
  void getObjectPositions(arr& positions);
  // orientation is 2d:  orientation(0) = angle to z axis,  orientation(1) = angle of projection to x/y plane
  void getOrientation(arr& orientation, uint id);
  void getObjectAngles(arr& angles);
  bool isUpright(uint id);        ///< check if id is upright
  double getHeight(uint id);
  double getOverallHeight(uintA& objects);
  uint getInhand(uint man_id); ///< get id of the object catched by man_id
  uint getInhand(); ///< get id of object catched with finger of right hand
  uint getHandID();
  // pile info
  void getObjectsOn(uintA& list,const char *obj_id); ///< return list of objects above and in contact with id
  void getObjectsOn(uintA& list,const uint obj_id);
  bool isClear(uint id);        ///< check if id is clear
  bool onGround(uint id);
  void getObjectsClose(uintA& objects_close, uint obj); ///< return list of close by objects (with vertical level)
  // table
  bool freePosition(double x, double y, double radius);
  double highestPosition(double x, double y, double radius, uint id_ignored);
  // box
  uint getContainedObject(uint box_id);
  bool isClosed(uint box_id);
  bool containedInBox(uint id);        ///< check if id is contained in some box
  bool containedInClosedBox(uint id);        ///< check if id is contained in some closed box
  // low-level
  bool inContact(uint a,uint b);  ///< check if a and b are in contact
  void writeAllContacts(uint id);
  
  void printObjectInfo();
  
  
  
 /************************************************
  * 
  *     Actions
  * 
  *     Action noise can be set in 
  *     robotManipulationSimulator.cpp.
  * 
  ************************************************/
  
  // grabbing
  void grab_final(const char *manipulator, const char *obj_grabbed, const char* message = "");
  void grab(uint id, const char* message = "");
  void grab(const char* obj_grabbed, const char* message = "");
  void grabHere(const char* message = "");
  // dropping (putting)
  void dropObjectAbove_final(const char *obj_dropped, const char *obj_below, const char* message = "");    ///< drop the body part obj_id above the object rel_id
  void dropObjectAbove(uint obj_dropped, uint obj_below, const char* message = "");
  void dropObjectAbove(uint obj_below, const char* message = ""); ///< drop obj hold in finger of right hand above the object rel_id
  void dropObject(uint manipulator_id); ///<drop object held by "manipulator_id"
  void dropInhandObjectOnTable(const char* message = "");
  void calcTargetPositionForDrop(double& x, double& y, uint obj_dropped, uint obj_below);
  // posture control
  void relaxPosition(const char* message = "");   ///< move into a relaxed position
	void moveToPosition(const arr& pos, const char* message = "");
  // boxes
  void openBox(uint id, const char* message = "");
  void closeBox(uint id, const char* message = "");
  
  
  
 /************************************************
  * 
  *     OpenGL Displaying
  * 
  ************************************************/
  
  void displayText(const char* text, uint waiting_time);
  
  
  
 /************************************************
  * 
  *     Member objects
  * 
  ************************************************/
  
  OpenGL *gl;
  ors::Graph *C;
  OdeInterface *ode;
  SwiftInterface *swift;
  RevelInterface *revel;
    
  uint numObjects;
  double neutralHeight;
  uint Tabort; //abortion time when attractors fail
  
  
 
};




/************************************************
  * 
  *     Creating ORS-objects
  * 
  ************************************************/

namespace relational {
void generateOrsBlocksSample(ors::Graph& ors, const uint numOfBlocks);
void generateOrsFromSample(ors::Graph& ors, const MT::Array<arr>& sample);
//void generateOrsFromTraySample(ors::Graph& ors, const MT::Array<arr>& sample);
void generateBlocksSample(MT::Array<arr>& sample, uint numOfBlocks);
void createCylinder(ors::Body& cyl, const ors::Vector& pos, const arr& color, const arr& size);
void createCylinder(ors::Body& cyl, const ors::Vector& pos, const arr& color);
}




#endif  // robotManipulationSimulator_h
