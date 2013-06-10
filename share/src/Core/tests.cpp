/*  ---------------------------------------------------------------------
    Copyright 2013 Marc Toussaint
    email: mtoussai@cs.tu-berlin.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
    You should have received a COPYING file of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>
    -----------------------------------------------------------------  */

#include "array.h"
#include "gtest/gtest.h"

TEST(ArrayTest, testOnes) {
  arr a = ones(5, 5);
  for (int i = 0; i < 5; ++i) {
    for (int j = 0; j < 5; ++j) {
      EXPECT_EQ(a(i, j), 1);
    }
  }
}

TEST(ArrayTest, testZeros) {
  arr a = zeros(5, 5);
  for (int i = 0; i < 5; ++i) {
    for (int j = 0; j < 5; ++j) {
      EXPECT_EQ(a(i, j), 0);
    }
  }
}

TEST(ArrayTest, testEye) {
  arr a = eye(5, 5);
  for (int i = 0; i < 5; ++i) {
    for (int j = 0; j < 5; ++j) {
      if (i == j) {
        EXPECT_EQ(a(i, j), 1) << "diagonal should be 1";
      } else {
        EXPECT_EQ(a(i, j), 0) << "non-diagonal should be 0";
      }
    }
  }
}

