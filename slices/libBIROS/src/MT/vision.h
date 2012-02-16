#ifndef MT_vision_h
#define MT_vision_h

#include "array.h"
//#include "threads.h"

extern arr camera_calibration;

//----- opencv wrappers



//macro tricks to easily convert arrays to CvMat*
struct CvMat;
struct CvMatDonor {
  CvMat *mat;
  int i;
  CvMatDonor();
  CvMat* get();
};
#define CVMAT(x) arr2cvmat(cvMatDonor.get(), x)
#define ENABLE_CVMAT CvMatDonor cvMatDonor;

CvMat* arr2cvmat(CvMat *mat, const byteA& img);
CvMat* arr2cvmat(CvMat *mat, const floatA& img);
//CvMat* arr2cvmat(CvMat *mat, const intA& img);
CvMat* arr2cvmat(CvMat *mat, const doubleA& img);
char cvShow(const byteA& img, const char *window="opencv", bool wait=false);
char cvShow(const floatA& img, const char *window="opencv", bool wait=false);
void smooth(floatA& theta, uint size);

//----- old MRF code (use BinaryBP instead!)
struct BP_data { arr phi, b, u, d, l, r; };
void mrf_BP(BP_data& msg, void (*conv)(arr&, const arr&), uint iter, byteA *max=NULL);

//----- convolutions
void boxConvolution(arr& out, const arr& in, uint width);
void gaussConvolution(arr& out, const arr& in, uint width, double eps=0.01);

//----- patch analysis
uint incremental_patch_ids(uintA& pch);
void get_patch_colors(floatA& pch_col, byteA& img, uintA& pch, uint np);
void get_patch_centroids(doubleA& pch_cen, byteA& img, uintA& pch, uint np);

//----- belief analysis
void getCenter(floatA& b);

//----- conversions (works for images AND color vectors/maps)
void byte2float(floatA& f, const byteA& b);
void rgb2hsv(floatA& hsv, floatA& rgb);
void getHsvEvidences(floatA &phi, floatA &hsv, const floatA& hsvTarget, const floatA& hsvTol);
float hsv_diff(const floatA& a, const floatA& b, const floatA& tol);
byteA evi2rgb(const floatA& theta);
floatA rgb2evi(const byteA& theta);

//----- high-level analysis
void getHsvCenter(arr& cen, byteA &img, uint iter, const floatA& hsvTarget, const floatA& hsvTol);
void localizeHsv(arr& worldPoint, byteA& left, byteA& right, const floatA& hsvTarget, const floatA& hsvTol, int smaller);
void findMaxRegionInEvidence(uintA& box, floatA *center, floatA *axis,
                             const floatA& theta, double threshold);
void getDiffProb(floatA& diff, const byteA& img0, const byteA& img1, float pixSdv, uint range);


//----- transformations
void imagePointPair2WorldPoint(floatA& world, const floatA& left, const floatA& right);

// draw helper routinges
void pch2img(byteA &img, const uintA &pch, floatA &pch_colormap);
void random_colorMap(floatA& pch_colormap, uint np);
void cvDrawGraph(byteA& img, doubleA& V, uintA& E);
void cvDrawBox(byteA& img, const floatA& box);
void cvDrawPoints(byteA& img, const arr& points);
void gnuplotHistogram(floatA &data, float min=0., float max=0., uint bins=100);

//----- flow estimation!

void evidence_diff(floatA& diff, const byteA& img0, const byteA& img1, float pixSdv=10.);

// messages
void compute_mu_IV(floatA& mu_IV, const byteA& It, const byteA& Itt, float pixSdv=10.);
void compute_mu_VV(floatA& mu_VV, const floatA& alphaV);






#ifdef  MT_IMPLEMENTATION
#  include "vision.cpp"
#endif


#endif
