#include <stdint.h>
#include <time.h>
#include <iostream>

using namespace std;

namespace {
#define PREVCOL_SHIFT   10
#define PREVCOL_HALF    (1 << (PREVCOL_SHIFT - 1))
#define PREVCOL_FIX(x)  ((int) ((x) * (1<<PREVCOL_SHIFT) + 0.5))

/*
 * rgb to yuv conversion taken from icewing: http://icewing.sf.net/
 *
 * Copyright (C) 1999-2009
 * Frank Loemker, Applied Computer Science, Faculty of Technology,
 * Bielefeld University, Germany
 *
 * This file is part of iceWing, a graphical plugin shell.
 *
 * iceWing is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * iceWing is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 */


/*********************************************************************
  Perform RGB to YUV conversion with intervall reduction.
*********************************************************************/
static inline void rgbToYuvVis (const uint8_t rc, const uint8_t gc, const uint8_t bc,
                                            uint8_t *yc, uint8_t *uc, uint8_t *vc)

{
    const int y_shift = PREVCOL_FIX(0.299*219.0/255.0) * rc +
            PREVCOL_FIX(0.587*219.0/255.0) * gc +
            PREVCOL_FIX(0.114*219.0/255.0) * bc + PREVCOL_HALF;
    const int y = y_shift >> PREVCOL_SHIFT;

    *uc = ((PREVCOL_FIX(0.564*224.0/255.0) * bc -
            PREVCOL_FIX(0.564*224.0/219.0) * y + PREVCOL_HALF) >> PREVCOL_SHIFT) + 128;
    *vc = ((PREVCOL_FIX(0.713*224.0/255.0) * rc -
            PREVCOL_FIX(0.713*224.0/219.0) * y + PREVCOL_HALF) >> PREVCOL_SHIFT) + 128;
    *yc = (y_shift + (16 << PREVCOL_SHIFT)) >> PREVCOL_SHIFT;
}

}

uint8_t *in_plane, *out_plane[3];

void bench_rgb2yuv(const unsigned int width, const unsigned int height) {
    uint8_t *rc, *gc, *bc;
    // BGR
    bc = in_plane;
    gc = in_plane + 1;
    rc = in_plane + 2;
    uint8_t * const yc = out_plane[0];
    uint8_t * const uc = out_plane[1];
    uint8_t * const vc = out_plane[2];
    const unsigned int num_pixel = width * height;

#pragma omp parallel for num_threads(2)
    for(int i = 0; i < num_pixel; ++i) {
        const int pixel_index = i*3;
        rgbToYuvVis(in_plane[pixel_index+2], in_plane[pixel_index+1], in_plane[pixel_index], yc + i, uc + i, vc + i);
    }
}

int main(int argc,char **argv){
  int width = 1280, height =1024;
  in_plane = new uint8_t[width*height*3];
  out_plane[0] = new uint8_t[width*height];
  out_plane[1] = new uint8_t[width*height];
  out_plane[2] = new uint8_t[width*height];

  // ramp up cpu speed
  for(int i = 0; i < 100; ++i) {
      bench_rgb2yuv(width, height);
      //std::clog << i << endl;
  }


  timespec start_ts, end_ts;
  clock_gettime(CLOCK_REALTIME, &start_ts);
  for(int i = 0; i < 1000; ++i) {
      bench_rgb2yuv(width, height);
  }

  clock_gettime(CLOCK_REALTIME, &end_ts);
  double start = start_ts.tv_sec + (start_ts.tv_nsec / 1e9), end = end_ts.tv_sec + (end_ts.tv_nsec / 1e9);
  std::clog << "Elapsed: " << (end - start) << ", per frame: " << (end-start) << endl;

  return 0;
};

