#include "ProgressBar.h"
#include "../util.h"

#include <iostream>

#include <algorithm>

using std::string;
using std::cout;
using std::endl;
using std::flush;
using std::min;

uint ProgressBar::bar_width = DEFAULT_PROGRESS_BAR_WIDTH;
uint ProgressBar::current_progress = 0;
string ProgressBar::progress_label = "Progress";

void ProgressBar::init(const char* label, const uint& width, const double& progress) {
    // set label if provided
    if(label!=nullptr) {
        progress_label = std::string(label);
    }

    // set width
    bar_width = width;

    // set current progress
    current_progress = progress*bar_width; // drop decimal places

    // open new line for writing progress bar
    cout << endl;

    // print bar
    print();
}

void ProgressBar::print(const double& progress) {
    uint new_current_progress = progress*bar_width; // drop decimal places
    if(new_current_progress!=current_progress) {
        current_progress=new_current_progress;
        print();
    }
}

void ProgressBar::print(const int& progress, const int& max_progress) {
    uint new_current_progress = progress*bar_width/max_progress; // drop decimal places
    if(new_current_progress!=current_progress) {
        current_progress=new_current_progress;
        print();
    }
}

void ProgressBar::terminate() {
    // print a full bar
    // current_progress = bar_width;
    // print();
    // open new line for further output
    cout << endl;
}

void ProgressBar::print() {
    // go to beginning of line and print start marker
    cout << '\r' << progress_label << "|";

    // print a spinning wheel
    static unsigned short wheel = 0;
    wheel = (wheel+1)%4;
    switch(wheel) {
    case 0:
        cout << '|';
        break;
    case 1:
        cout << '/';
        break;
    case 2:
        cout << '-';
        break;
    case 3:
        cout << '\\';
        break;
    default:
        cout << '*';
    }
    cout << "|";

    // print processed part
    for(uint idx=0; idx < min(current_progress,bar_width); ++idx) {
        cout << "-";
    }

    if(current_progress<bar_width) {

        // print current position
        cout << "o";

        // print unprocessed part
        for(uint idx=current_progress+1; idx<bar_width; ++idx) {
            cout << " ";
        }

    }

    // print end marker
    cout << "|" << flush;
}
