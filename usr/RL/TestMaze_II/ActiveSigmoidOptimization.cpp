#include "ActiveSigmoidOptimization.h"

#include <algorithm>
//#include <iostream>
#include <fstream>

#define DEBUG_LEVEL 1
#include "debug.h"

using std::sort;
using std::endl;
using std::ofstream;

struct ActiveSigmoidOptimization::Point {
    double x, y;
    Point(double _x, double _y): x(_x), y(_y) {}
    bool operator<(const Point& other) const {
        if(x<other.x) {
            return true;
        } else if(x>other.x) {
            return false;
        } else if(y<other.y) {
            return true;
        } else {
            return false;
        }
    }
};

ActiveSigmoidOptimization::ActiveSigmoidOptimization(): data_sorted(false) {}

ActiveSigmoidOptimization::~ActiveSigmoidOptimization() {}

void ActiveSigmoidOptimization::add_new_point(const double& x, const double& y) {
    data.push_back(Point(x,y));
    if(DEBUG_LEVEL>1) {
        DEBUG_OUT(0,"Added " << data.back().x << "	" << data.back().y);
        sort_data();
        DEBUG_OUT(0,"All data points:");
        for(Point p : data) {
            DEBUG_OUT(0,"    " << p.x << "	" << p.y);
        }
    }
}

void ActiveSigmoidOptimization::print_curve_to_file(const char* file_name, const double& x_min, const double& x_max, const unsigned int& points) {

    // open file
    ofstream plot_file;
    plot_file.open(file_name);
    if(!plot_file.is_open()) {
        DEBUG_OUT(0,"Error: Could not open file");
        return;
    }

    // print data points
    plot_file << "## Data Points" << endl;
    plot_file << "# x	y" << endl;
    for(Point p : data) {
        plot_file << p.x << "	" << p.y << endl;
    }

    // separate blocks
    plot_file << endl << endl;

    // print interpolation
    plot_file << "## Interpolation" << endl;
    plot_file << "# x	y" << endl;

    double x_increment = (x_max-x_min)/points;
    for(double x = x_min; x<=x_max; x+=x_increment) {
        plot_file << x << "	" << f(x) << endl;
    }

    // close file
    plot_file.close();
}

void ActiveSigmoidOptimization::print_curve_to_file(const char* file_name, const unsigned int& points) {
    // sort data
    sort_data();
    double x_min = data.front().x;
    double x_max = data.back().x;
    print_curve_to_file(file_name,x_min,x_max,points);
}

double ActiveSigmoidOptimization::f(const double& x) {

    // number of data points
    int data_n = data.size();

    // special cases
    switch(data_n) {
    case 0:
        DEBUG_OUT(0,"Error: No data");
        return 0;
    case 1:
        DEBUG_OUT(1,"Warning: Only one data point");
        return data.back().y;
    case 2:
    {
        // interpolate linearly: find a and b so that f(x) = a + b x goes
        // through both data points
        Point p1 = data.front();
        Point p2 = data.back();
        double b = (p2.y-p1.y)/(p2.x-p1.x);
        double a = p1.y - p1.x * b;
        return a + b*x;
    }
    default:
        // interpolate with cubic splines
        break;
    }

    // find enclosing points
    int idx_1, idx_2;
    find_enclosing_points(x, idx_1, idx_2);

    // handle special cases
    if(idx_1==idx_2) {
        // exact match
        return data[idx_1].y;
    } else if(idx_1<0){
        // need to extrapolate to the left
        DEBUG_OUT(1,"Warning: Extrapolating to the left");
        idx_1 = idx_2;
        idx_2 = idx_2 + 1;
    } else if(idx_2<0){
        // need to extrapolate to the right
        DEBUG_OUT(1,"Warning: Extrapolating to the right");
        idx_2 = idx_1;
        idx_1 = idx_1 - 1;
    }

    // get the points
    Point p1 = data[idx_1];
    Point p2 = data[idx_2];

    // do cubic interpolation (use quadratic interpolation at the border)

    if(idx_1==0) { // quadratic at left border

        // get tangent at p2
        Point p3 = data[idx_2+1];
        double d2 = (p3.y-p1.y)/(p3.x-p1.x);
        double x1=p1.x, x2=p2.x, y1=p1.y, y2=p2.y;

        // compute coefficents y = a + b x + c x^2
        double a = (-d2*pow(x1,2)*x2+d2*x1*pow(x2,2)+pow(x1,2)*y2-2*x1*x2*y2+pow(x2,2)*y1)/pow(x1-x2,2);
        double b = (d2*pow(x1,2)-d2*pow(x2,2)-2*x2*y1+2*x2*y2)/pow(x2-x1,2);
        double c = (-d2*x1+d2*x2+y1-y2)/pow(x2-x1,2);
        return a + b*x + c*pow(x,2);

    } else if(idx_1==data_n-1) { // quadratic at right border

        // get tangent at p1
        Point p0 = data[idx_1-1];
        double d1 = (p0.y-p1.y)/(p0.x-p1.x);
        double x1=p1.x, x2=p2.x, y1=p1.y, y2=p2.y;

        // compute coefficents y = a + b x + c x^2
        double a = (d1*pow(x1,2)*x2-d1*x1*pow(x2,2)+pow(x1,2)*y2-2*x1*x2*y1+pow(x2,2)*y1)/pow(x1-x2,2);
        double b = (-d1*pow(x1,2)+d1*pow(x2,2)+2*x1*y1-2*x1*y2)/pow(x1-x2,2);
        double c = (d1*x1-d1*x2-y1+y2)/pow(x1-x2,2);
        return a + b*x + c*pow(x,2);

    } else { // cubic interpolation

        // get tangent at p1 and p2
        Point p0 = data[idx_1-1];
        double d1 = (p0.y-p1.y)/(p0.x-p1.x);
        Point p3 = data[idx_2+1];
        double d2 = (p3.y-p1.y)/(p3.x-p1.x);
        double x1=p1.x, x2=p2.x, y1=p1.y, y2=p2.y;

        // third order polynomial in symmetrical form:
        // y = (1-t)y1 + t y2 + t(1-t)(a(1-t)+b t)
        double t = (x-x1)/(x2-x1);
        double a = d1*(x2-x1)-(y2-y1);
        double b = -d2*(x2-x1)+(y2-y1);
        return (1-t)*y1 + t*y2 + t*(1-t)*(a*(1-t)+b*t);
    }
}

void ActiveSigmoidOptimization::sort_data() {
    sort(data.begin(),data.end());
}

void ActiveSigmoidOptimization::find_enclosing_points(const double& x, int& idx_1, int& idx_2) {

    // set some values
    int data_n = data.size();
    idx_1 = -1;
    idx_2 = -1;

    // check
    if(data_n<2) {
        DEBUG_OUT(0,"Error: Not enough data to find enclosing points");
        return;
    }

    // make sure data points are sorted
    sort_data();

    // simple linear search
    for(int idx = 0; idx<data_n; ++idx) {
        Point p = data[idx];
        if(p.x<x) {
            idx_1 = idx;
        } else if(p.x==x) {
            idx_1 = idx;
            idx_2 = idx;
            return;
        } else if(p.x>x) {
            idx_2 = idx;
            return;
        }
    }
}
