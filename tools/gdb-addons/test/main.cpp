#include <Core/array.h>

int main(int argc, const char** argv) {
  arr testarray;
  int i = 0;
  testarray.resize(0);
  i++;
  testarray = rand(6);
  i++;
  testarray.reshape(3,3,4);
  i++;
}
