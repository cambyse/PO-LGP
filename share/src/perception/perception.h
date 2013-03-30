#ifndef MT_perception_h
#define MT_perception_h

#ifdef MT_OPENCV
#  undef COUNT
#  include <opencv2/opencv.hpp>
#  undef MIN
#  undef MAX
#endif

#include <system/module.h>
#include <views/views.h>
#include <MT/ors.h>
#include <MT/array_t.cxx>

//===========================================================================
//
// fwd declarations
//

extern void loadPerception();

//-- Variables
struct Image;
struct FloatImage;
struct Colors;
struct HoughLines;
struct Patching;
struct SURFfeatures;
struct PerceptionOutput;

//-- Process creators
Process *newOpencvCamera();
Process* newCvtGray(Image& rgb, Image& gray);
Process* newCvtHsv(Image& rgb, Image& hsv);
Process* newHsvFilter(Image& hsv, FloatImage& evi);
Process* newMotionFilter(Image& rgb, Image& motion);
Process* newDifferenceFilter(Image& i1,Image& i2, Image& diff);
Process* newCannyFilter(Image& grayImage, Image& cannyImage, float cannyThreshold=50.f);
Process* newPatcher(Image& rgbImage, Patching& patchImage);
Process* newSURFer(Image& grayImage, SURFfeatures& features);
Process* newHoughLineFilter(Image& grayImage, HoughLines& houghLines);
Process* newShapeFitter(FloatImage& eviL, FloatImage& eviR, PerceptionOutput& perc);


//===========================================================================
//
// Types
//

struct RigidObjectRepresentation {
  uint found;
  
  //-- 2d shape
  uint shapeType;
  arr shapeParamsL, shapeParamsR, shapePointsL, shapePointsR;
  
  //-- 3d information
  arr shapePoints3d;
  arr center3d, orsShapeParams;
  arr diagDiff;

  RigidObjectRepresentation(){ found=0; }
};

typedef MT::Array<RigidObjectRepresentation*> RigidObjectRepresentationL;


//===========================================================================
//
// Variables
//

struct Image:Variable {
  FIELD(byteA, img);
  Image(const char* name):Variable(name) {}
};

struct FloatImage:Variable {
  FIELD(floatA, img);
  FloatImage(const char* name):Variable(name) {}
};

struct ColorChoice{
  FIELD(byteA, rgb);
  FIELD(byteA, hsv);
};

struct HoughLines {
#ifdef MT_OPENCV
  std::vector<cv::Vec4i> lines;
#endif
  FIELD(byteA, display);
};
inline void operator>>(istream& is,HoughLines& hl){}
inline void operator<<(ostream& os,const HoughLines& hl){}

struct Patching {
  uintA patching;  //for each pixel an integer
  arr pch_cen;     //patch centers
  uintA pch_edges; //patch Delauney edges
  floatA pch_rgb;  //patch mean colors
  FIELD(byteA, display);
};
inline void operator>>(istream& is,Patching& hl){}
inline void operator<<(ostream& os,const Patching& hl){}

struct SURFfeatures {
#ifdef MT_OPENCV
  std::vector<cv::KeyPoint> keypoints;
  std::vector<float> descriptors;
#endif
  FIELD(byteA, display);
};
inline void operator>>(istream& is,SURFfeatures& hl){}
inline void operator<<(ostream& os,const SURFfeatures& hl){}

/*! The RigidObjectRepresentation List output of perception */
struct PerceptionOutput {
  MT::Array<RigidObjectRepresentation> objects;
  FIELD(byteA, display);
};


//===========================================================================
//
// Modules
//



//===========================================================================
//
// Views
//

#define DECLARE_VIEW(VAR) \
struct VAR##_View:View{ \
  byteA copy; \
  VAR##_View():View(){} \
  VAR##_View(VAR& var, GtkWidget *container=NULL):View(){ object=&var; gtkNew(container); } \
  void glInit(); \
  void glDraw(); \
  void gtkNew(GtkWidget *container){ gtkNewGl(container); } \
};


DECLARE_VIEW(Image)
//DECLARE_VIEW(FloatImage)
DECLARE_VIEW(HoughLines)
DECLARE_VIEW(Patching)
DECLARE_VIEW(SURFfeatures)
DECLARE_VIEW(PerceptionOutput)


//===========================================================================
//
// PRELIMINARY
//

//BEGIN_MODULE(ColorPicker)
//  ACCESS(ColorChoice, colorChoice);
//END_MODULE()


//TODO Johannes ProcessL newPointcloudProcesses();
//TODO Johannes VariableL newPointcloudVariables();

//TODO: where should this go? maybe ors?
//TODO Johannes const int RADIUS = 2;
//TODO Johannes const int HEIGHT = 3;


/*TODO Johannes
struct ObjectBelief {

  ObjectBelief() {
    shapeParams.resize(4);  
  }
  //pose
  // TODO: make pointers
  ors::Vector position;
  ors::Quaternion rotation;

  arr poseCov;

  // primitive shapes
  ors::ShapeType shapeType;
  arr shapeParams;

  // TODO: make pointer, such that the using app does not need to implicitly
  // include half of the PCL?
  //pcl::ModelCoefficients::Ptr pcl_object;

  //pcl::PointCloud<PointT>* pointCloud;
  arr vertices;
  uintA triangles;
};

struct ObjectBeliefSet : Variable {
  FIELD(MT::Array<ObjectBelief*>, objects);
  ObjectBeliefSet(const char *name) : Variable(name) { reg_objects(); }
};
*/

#endif //MT_perception_h





/*

MEMO on what should/could be elements (Variables) of a perception system

** image level

cam image

grey image
hsv image

hsv-filter-values
hsv-filter-images

background
diff

depth

self-projection-mask
& cropped images

object-projections

Patch


** point cloud level

point cloud



** features level

SURFfeatures

HoughLines

Canny


** primitives level

(RANSAC type methods)

spheres

planes


** rigid body level

list of moving (or non-moving) rigid body hypothesis
(workspace for a tracker -- where does he get the likelihood function from?)


** shape level

list of shapes

*/
