#include "al_util.h"

void flatten(arr& out, const MT::Array<arr>& in) {
	if (1 == in.nd) {
		for (uint i = 0; i < in.d0; ++i) {
      out.append(in(i));  
		}
		out.reshape(1, out.N);
	} 
	else if (2 == in.nd) {
    uint length = 0;
    for (uint i = 0; i < in.d0; ++i) {
      for (uint j = 0; j < in.d1; ++j) {
        out.append(in(i, j));  
        if (!i) {
           length += in(i,j).N;
        }
      }
    }
    out.reshape(in.d0, length);
	}
}
