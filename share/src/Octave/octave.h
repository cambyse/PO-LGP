#include <Core/array.h>

struct OctaveSpace{
  int verbose;
  OctaveSpace();

  ScalarFunction scalarFunction;
};

extern Singleton<OctaveSpace> OCTAVE;

class Matrix conv_arr2octave(const arr& x);
arr conv_octave2arr(const class Matrix& x);

