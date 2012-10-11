#include <pthread.h>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <typeinfo>
#include <math.h>
#include <string.h>
#include <cmath>
#include <stdint.h>

//    A thread that just exits
void*    mythread(void *arg) { return 0; }

int main (int argc, char *argv[]) {
    int       err,
              count=0;
    pthread_t thread;
    while (1) {
        err = pthread_create(&thread, 0, mythread, 0);
        if (err != 0) {
            printf("Count: %d Error: %d '%s'\n", count, err, strerror(err));
            sleep(5);
	    return 0;
        }
        count++;
        if (count % 1000 == 0)
            printf("Count: %d\n", count);
    }
    return 0;
}
