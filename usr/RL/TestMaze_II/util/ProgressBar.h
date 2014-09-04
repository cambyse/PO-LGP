#ifndef PROGRESS_BAR_H_
#define PROGRESS_BAR_H_

#include <string>
#include <iostream>
#include <sstream>

#define DEFAULT_PROGRESS_BAR_WIDTH 50

/** \brief This class provides a simple command line progress bar. */
class ProgressBar {

    //----members----/
private:
    static uint bar_width;
    static uint current_progress;
    static std::string start_msg;
    static std::stringstream end_msg;

    //----methods----/
public:
    static void init(const char* message = "Progress",
                     const uint& width = DEFAULT_PROGRESS_BAR_WIDTH,
                     const double& progress = 0
        );
    static void print(const double& progress);
    static void print(const int& progress, const int& max_progress);
    static std::stringstream& msg();
    static std::stringstream& msg_append();
    static void terminate(const char* message = "");
    static void set_bar_width(const uint& width) { bar_width=width; }
private:
    static void print();
};

#endif /* PROGRESS_BAR_H_ */
