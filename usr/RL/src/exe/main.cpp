#include <util.h>

int main(int argn, char ** args) {
    int i;
    if(util::arg_int("some int 5", 2, i)) {
        return i;
    } else {
        return 0;
    }
}
