#include "SplineInterpolation.h"

#include "util.h"

#include <float.h> // for DBL_MAX
#include <fstream>

#define DEBUG_LEVEL 1
#include "debug.h"

using std::ofstream;
using std::endl;

double SplineInterpolation::f(const std::vector<double>& x_data,
                              const std::vector<double>& y_data,
                              const double& x_querry
    ) {

    // number of data points
    int x_data_n = x_data.size();
    int y_data_n = y_data.size();
    if(x_data_n!=y_data_n) {
        DEBUG_OUT(0,"Error: x- and y-data have unequal size:");
    }

    // use the smaler of both if equal
    int data_n = util::min<int>(x_data_n,y_data_n);

    // special cases
    switch(data_n) {
    case 0:
        DEBUG_OUT(0,"Error: No data");
        return 0;
    case 1:
        DEBUG_OUT(1,"Warning: Only one data point");
        return y_data.back();
    case 2:
    {
        // interpolate linearly: find a and b so that f(x) = a + b x goes
        // through both data points
        double x1 = x_data.front();
        double x2 = x_data.back();
        double y1 = y_data.front();
        double y2 = y_data.back();
        double b = (y2-y1)/(x2-x1);
        double a = y1 - x1 * b;
        return a + b*x_querry;
    }
    default:
        // interpolate with cubic splines
        break;
    }

    // find enclosing points
    int idx_1, idx_2;
    find_enclosing_points(x_data, x_querry, idx_1, idx_2);

    // handle special cases
    if(idx_1==idx_2) {
        // exact match
        return y_data[idx_1];
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

    // index to point before and after enclosing points
    int idx_0 = idx_1 - 1;
    int idx_3 = idx_2 + 1;

    // do cubic interpolation (use quadratic interpolation at the border)

    if(idx_1==0) { // quadratic at left border

        // get points
        double x1 = x_data[idx_1];
        double y1 = y_data[idx_1];
        double x2 = x_data[idx_2];
        double y2 = y_data[idx_2];
        double x3 = x_data[idx_3];
        double y3 = y_data[idx_3];

        // get tangent at point 2
        double d2 = (y3-y1)/(x3-x1);

        // compute coefficents y = a + b x + c x^2
        double a = (-d2*pow(x1,2)*x2+d2*x1*pow(x2,2)+pow(x1,2)*y2-2*x1*x2*y2+pow(x2,2)*y1)/pow(x1-x2,2);
        double b = (d2*pow(x1,2)-d2*pow(x2,2)-2*x2*y1+2*x2*y2)/pow(x2-x1,2);
        double c = (-d2*x1+d2*x2+y1-y2)/pow(x2-x1,2);
        return a + b*x_querry + c*pow(x_querry,2);

    } else if(idx_1==data_n-1) { // quadratic at right border

        // get points
        double x0 = x_data[idx_0];
        double y0 = y_data[idx_0];
        double x1 = x_data[idx_1];
        double y1 = y_data[idx_1];
        double x2 = x_data[idx_2];
        double y2 = y_data[idx_2];

        // get tangent at point 1
        double d1 = (y0-y1)/(x0-x1);

        // compute coefficents y = a + b x + c x^2
        double a = (d1*pow(x1,2)*x2-d1*x1*pow(x2,2)+pow(x1,2)*y2-2*x1*x2*y1+pow(x2,2)*y1)/pow(x1-x2,2);
        double b = (-d1*pow(x1,2)+d1*pow(x2,2)+2*x1*y1-2*x1*y2)/pow(x1-x2,2);
        double c = (d1*x1-d1*x2-y1+y2)/pow(x1-x2,2);
        return a + b*x_querry + c*pow(x_querry,2);

    } else { // cubic interpolation

        // get points
        double x0 = x_data[idx_0];
        double y0 = y_data[idx_0];
        double x1 = x_data[idx_1];
        double y1 = y_data[idx_1];
        double x2 = x_data[idx_2];
        double y2 = y_data[idx_2];
        double x3 = x_data[idx_3];
        double y3 = y_data[idx_3];

        // get tangent at point 1 and point 2
        double d1 = (y0-y1)/(x0-x1);
        double d2 = (y3-y1)/(x3-x1);

        // third order polynomial in symmetrical form:
        // y = (1-t)y1 + t y2 + t(1-t)(a(1-t)+b t)
        double t = (x_querry-x1)/(x2-x1);
        double a = d1*(x2-x1)-(y2-y1);
        double b = -d2*(x2-x1)+(y2-y1);
        return (1-t)*y1 + t*y2 + t*(1-t)*(a*(1-t)+b*t);
    }
}

void SplineInterpolation::print_curve_to_file(const std::vector<double>& x_data,
                                              const std::vector<double>& y_data,
                                              const char* file_name,
                                              const double& x_min,
                                              const double& x_max,
                                              const unsigned int& points) {
    // number of data points
    int x_data_n = x_data.size();
    int y_data_n = y_data.size();
    if(x_data_n!=y_data_n) {
        DEBUG_OUT(0,"Error: x- and y-data have unequal size:");
    }

    // use the smaler of both if equal
    int data_n = util::min<int>(x_data_n,y_data_n);

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
    for(int idx = 0; idx<data_n; ++idx) {
        plot_file << x_data[idx] << "	" << y_data[idx] << endl;
    }

    // separate blocks
    plot_file << endl << endl;

    // print interpolation
    plot_file << "## Interpolation" << endl;
    plot_file << "# x	y" << endl;

    double x_increment = (x_max-x_min)/points;
    for(double x = x_min; x<=x_max; x+=x_increment) {
        plot_file << x << "	" << f(x_data,y_data,x) << endl;
    }

    // close file
    plot_file.close();
}

void SplineInterpolation::print_curve_to_file(const std::vector<double>& x_data,
                                              const std::vector<double>& y_data,
                                              const char* file_name,
                                              const unsigned int& points) {
    double x_min = x_data.front();
    double x_max = x_data.back();
    print_curve_to_file(x_data,y_data,file_name,x_min,x_max,points);
}

void SplineInterpolation::find_enclosing_points(const std::vector<double>& x_data,
                                                const double& x_querry,
                                                int& idx_1,
                                                int& idx_2) {
    // number of data points
    int data_n = x_data.size();

    // check
    if(data_n<2) {
        DEBUG_OUT(0,"Error: Not enough data to find enclosing points");
        return;
    }

    // default
    idx_1 = -1;
    idx_2 = -1;

    // simple linear search
    double old_x = -DBL_MAX;
    for(int idx = 0; idx<data_n; ++idx) {
        double x_value = x_data[idx];
        if(x_value<old_x) {
            DEBUG_OUT(0,"Error: Data not sorted");
        } else {
            old_x = x_value;
        }
        if(x_value<x_querry) {
            idx_1 = idx;
        } else if(x_value==x_querry) {
            idx_1 = idx;
            idx_2 = idx;
            return;
        } else if(x_value>x_querry) {
            idx_2 = idx;
            return;
        }
    }
}
