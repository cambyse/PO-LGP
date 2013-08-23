#include "ActiveSigmoidOptimization.h"

#include <math.h> // for exp
#include <float.h> // for DBL_MAX
#include <algorithm> // for sort

#include "qcustomplot.h"
#include "util.h"

#define DEBUG_LEVEL 1
#include "debug.h"

using util::min;
using util::max;
using util::clamp;
using std::vector;
using std::pair;
using std::make_pair;
using std::sort;

ActiveSigmoidOptimization::ActiveSigmoidOptimization(const double& min_x,
                                                     const double& max_x,
                                                     const double& y_init,
                                                     const int& control_point_n_
    ):
    control_point_n(control_point_n_),
    data_initialized(false),
    bias_factor(0),
    x_sig(control_point_n),
    y_sig(control_point_n,y_init),
    y_sig_upper(y_sig),
    y_sig_lower(y_sig)
{
    // initialize x-values
    double x = min_x;
    double x_step = (max_x-min_x)/(control_point_n-1);
    for(int idx=0; idx<control_point_n; ++idx, x+=x_step) {
        x_sig[idx] = x;
    }
}

ActiveSigmoidOptimization::~ActiveSigmoidOptimization() {}

void ActiveSigmoidOptimization::add_new_point(const double& x, const double& y) {
    // add point
    x_data.push_back(x);
    y_data.push_back(y);

    DEBUG_OUT(2,"Added " << x_data.back() << "	" << y_data.back());

    // data need to be (re)initialized
    data_initialized = false;
}

void ActiveSigmoidOptimization::initialize_data() {
    if(!data_initialized) {

        DEBUG_OUT(2,"Initializing data");

        // sort data
        int raw_data_n = min<int>(x_data.size(),y_data.size());
        vector<pair<double,double> > ordering(raw_data_n);
        for(int i=0; i<raw_data_n; ++i) {
            ordering[i] = make_pair(x_data[i],y_data[i]);
        }
        sort(ordering.begin(), ordering.end());
        for(int i=0; i<raw_data_n; ++i) {
            x_data[i] = ordering[i].first;
            y_data[i] = ordering[i].second;
        }

        // associate data points with line segments
        data_per_segment.resize(control_point_n+1,make_pair(-1,-2));
        int segment_idx = 0;
        int data_idx = 0;

        // iterate through data and (all but the last) segments
        while(data_idx<raw_data_n && segment_idx<control_point_n) {
            if(x_data[data_idx]<=x_sig[segment_idx]) {
                // point lies in current segment
                if(data_per_segment[segment_idx].first<0) {
                    data_per_segment[segment_idx].first = data_idx;
                }
                if(data_per_segment[segment_idx].second<data_idx) {
                    data_per_segment[segment_idx].second = data_idx;
                }
                // increment data idx keep segment idx
                ++data_idx;
            } else {
                // point does not lie in current segment
                // increment segment idx keep data idx
                ++segment_idx;
            }
        }

        // debug check
        if(data_idx<raw_data_n && segment_idx!=control_point_n) {
            DEBUG_OUT(0,"Error: Data initialization failed");
            return;
        }

        // fill last segment (if data left)
        while(data_idx<raw_data_n) {
            // point lies in last segment
            if(data_per_segment[segment_idx].first<0) {
                data_per_segment[segment_idx].first = data_idx;
            }
            if(data_per_segment[segment_idx].second<data_idx) {
                data_per_segment[segment_idx].second = data_idx;
            }
            // increment data idx keep segment idx (which is maximum)
            ++data_idx;
        }

        // data are now initialized
        data_initialized = true;
    }
}

void ActiveSigmoidOptimization::print_to_QCP(QCustomPlot * plotter,
                                             const bool& print_upper,
                                             const bool& print_lower
    ) const {
    // initialize data
    int raw_data_n = min<int>(x_data.size(),y_data.size());
    QVector<double> qx_raw_data(raw_data_n), qy_raw_data(raw_data_n);
    QVector<double> qx_sig(control_point_n), qy_sig(control_point_n);
    QVector<double> qx_sig_upper(control_point_n), qy_sig_upper(control_point_n);
    QVector<double> qx_sig_lower(control_point_n), qy_sig_lower(control_point_n);

    // transfer raw data to QVector
    for(int i=0; i<raw_data_n; ++i) {
        qx_raw_data[i] = x_data[i];
        qy_raw_data[i] = y_data[i];
    }

    // transfer points to QVector
    for(int i=0; i<control_point_n; ++i) {
        qx_sig[i]       = x_sig[i];
        qx_sig_upper[i] = x_sig[i];
        qx_sig_lower[i] = x_sig[i];
        qy_sig[i]       = y_sig[i];
        qy_sig_upper[i] = y_sig_upper[i];
        qy_sig_lower[i] = y_sig_lower[i];
    }

    // clear graphs
    plotter->clearGraphs();

    // create graph for upper bounds
    QCPGraph * upper_graph = plotter->addGraph();
    upper_graph->setPen(QColor(200, 0, 0, 200));
    upper_graph->setBrush(QColor(200, 0, 0, 50));
    upper_graph->setLineStyle(QCPGraph::lsLine);
    upper_graph->setScatterStyle(QCP::ScatterStyle::ssDisc);
    upper_graph->setScatterSize(2);
    upper_graph->setName("Upper Bound");
    upper_graph->setData(qx_sig, qy_sig_upper);

    // create graph for lower bounds
    QCPGraph * lower_graph = plotter->addGraph();
    lower_graph->setPen(QColor(200, 0, 0, 200));
    lower_graph->setBrush(QColor(200, 0, 0, 50));
    lower_graph->setLineStyle(QCPGraph::lsLine);
    lower_graph->setScatterStyle(QCP::ScatterStyle::ssDisc);
    lower_graph->setScatterSize(2);
    lower_graph->setName("Lower Bound");
    lower_graph->setData(qx_sig, qy_sig_lower);

    // create graph for sigmoid
    QCPGraph * sig_graph = plotter->addGraph();
    sig_graph->setPen(QColor(0, 200, 0, 255));
    sig_graph->setLineStyle(QCPGraph::lsLine);
    sig_graph->setScatterStyle(QCP::ScatterStyle::ssDiamond);
    lower_graph->setScatterSize(3);
    sig_graph->setName("Mean");
    sig_graph->setData(qx_sig, qy_sig);

    // create graph for raw data
    QCPGraph * raw_data_graph = plotter->addGraph();
    raw_data_graph->setPen(QColor(255, 155, 0, 255));
    raw_data_graph->setLineStyle(QCPGraph::lsNone);
    raw_data_graph->setScatterStyle(QCP::ScatterStyle::ssDisc);
    raw_data_graph->setName("Raw Data");
    raw_data_graph->setData(qx_raw_data, qy_raw_data);

    // fill bounds or remove (removing changes indices)
    if(print_upper) {
        upper_graph->setChannelFillGraph(sig_graph);
    } else {
        plotter->removeGraph(upper_graph);
    }
    if(print_lower) {
        lower_graph->setChannelFillGraph(sig_graph);
    } else {
        plotter->removeGraph(lower_graph);
    }

    // give the axes some labels:
    plotter->xAxis->setLabel("x");
    plotter->yAxis->setLabel("y");

    // add title and legend
    plotter->setTitle("Active Sigmoid Optimization");
    plotter->legend->setVisible(true);
    QFont legendFont;
    legendFont.setPointSize(7);
    plotter->legend->setFont(legendFont);
    plotter->legend->setBrush(QBrush(QColor(255,255,255,150)));
    plotter->legend->setBorderPen(QPen(QColor(0,0,0,150)));

    // rescale axes and replot
    plotter->rescaleAxes();
    plotter->replot();
}

void ActiveSigmoidOptimization::optimize_sigmoid(const int& iterations) {
    DEBUG_OUT(2,"Optimizing sigmoid");
    bias_factor = 0;
    optimize(iterations);
}

void ActiveSigmoidOptimization::optimize_upper_bound(const double& strength, const int& iterations) {
    DEBUG_OUT(2,"Optimizing upper bound with bias strength " << strength);
    bias_factor = -strength;
    vector<double> y_tmp = y_sig;
    optimize(iterations);
    y_sig_upper = y_sig;
    y_sig = y_tmp;
    bias_factor = 0;
}

void ActiveSigmoidOptimization::optimize_lower_bound(const double& strength, const int& iterations) {
    DEBUG_OUT(2,"Optimizing lower bound with bias strength " << strength);
    bias_factor = strength;
    vector<double> y_tmp = y_sig;
    optimize(iterations);
    y_sig_lower = y_sig;
    y_sig = y_tmp;
    bias_factor = 0;
}

void ActiveSigmoidOptimization::optimize(const int& iterations) {

    initialize_data();
    initialize_to_linear();

    // set some variables
    // double delta_step_threshold = 1e-3;
    // double max_delta_step = DBL_MAX;

    // optimize control points until convergence
    for(int max_iteration=iterations; max_iteration>0; --max_iteration) {
        // max_delta_step = 0;
        for(int i=0; i<control_point_n; ++i) {
            optimize_control_point(i);
            // double delta_step = optimize_control_point(i);
            // max_delta_step = max<double>(delta_step, max_delta_step);
        }
        // if(max_delta_step<delta_step_threshold) {
        //     // convergence
        //     break;
        // }
    }

    //========================================//

    // double step_size = 0.1;                     // start with this value
    // double last_rms_error = get_rms_error();    // initialize
    // for(int max_iteration=iterations==-1?1000:iterations; max_iteration>0; --max_iteration) {
    //     vector<double> old_y_sig = y_sig;
    //     for(double& y : y_sig) {
    //         // apply random step
    //         y += (2*drand48()-1)*step_size;
    //     }
    //     for(int i=0; i<control_point_n; ++i) {
    //         // make monoton
    //         if(i==0) {
    //             y_sig[i] = min<double>(y_sig[i+1],y_sig[i]);
    //         } else if(i==control_point_n-1) {
    //             y_sig[i] = max<double>(y_sig[i-1],y_sig[i]);
    //         } else {
    //             y_sig[i] = clamp<double>(y_sig[i-1],y_sig[i+1],y_sig[i]);
    //         }
    //     }
    //     // get new rms error
    //     double current_rms_error = get_rms_error();
    //     if(current_rms_error>last_rms_error) {
    //         // don't apply
    //         y_sig = old_y_sig;
    //         // reduce step size
    //         // step_size *= 0.99;
    //     } else {
    //         // update last rms error
    //         last_rms_error = current_rms_error;
    //         // increase step size
    //         // step_size *= 1.1;
    //     }
    // }
}

double ActiveSigmoidOptimization::optimize_control_point(const int& i) {

    DEBUG_OUT(3,"   Optimizing " << i << "th control point");

    // control point is optimized to minimize the squared error
    double b = 0;
    double c = 0;
    if(i==0) {
        // first control point
        double xi   = x_sig[i  ];
        double xip1 = x_sig[i+1];
        double yip1 = y_sig[i+1];
        for(int data_idx = data_per_segment[i].first; data_idx<=data_per_segment[i].second; ++data_idx) {
            double dx = x_data[data_idx];
            double dy = y_data[data_idx];
            DEBUG_OUT(4,"        Using data point " << data_idx <<
                      " (" << x_data[data_idx] << "," << y_data[data_idx] << ") before");
            double term1 = 1-(dx-xi)/(xip1-xi);
            double term2 = yip1*(dx-xi)/(xip1-xi)-dy;
            b += 2*term1*term2;
            c += term1*term1;
        }
        for(int data_idx = data_per_segment[i+1].first; data_idx<=data_per_segment[i+1].second; ++data_idx) {
            double dx = x_data[data_idx];
            double dy = y_data[data_idx];
            DEBUG_OUT(4,"        Using data point " << data_idx <<
                      " (" << x_data[data_idx] << "," << y_data[data_idx] << ") after");
            double term1 = 1-(dx-xi)/(xip1-xi);
            double term2 = yip1*(dx-xi)/(xip1-xi)-dy;
            b += 2*term1*term2;
            c += term1*term1;
        }
    } else if(i==control_point_n-1) {
        // last control point
        double xim1 = x_sig[i-1];
        double xi   = x_sig[i  ];
        double yim1 = y_sig[i-1];
        for(int data_idx = data_per_segment[i].first; data_idx<=data_per_segment[i].second; ++data_idx) {
            double dx = x_data[data_idx];
            double dy = y_data[data_idx];
            DEBUG_OUT(4,"        Using data point " << data_idx <<
                      " (" << x_data[data_idx] << "," << y_data[data_idx] << ") before");
            double term1 = yim1*(1-(dx-xim1)/(xi-xim1))-dy;
            double term2 = (dx-xim1)/(xi-xim1);
            b += 2*term1*term2;
            c += term2*term2;
        }
        for(int data_idx = data_per_segment[i+1].first; data_idx<=data_per_segment[i+1].second; ++data_idx) {
            double dx = x_data[data_idx];
            double dy = y_data[data_idx];
            DEBUG_OUT(4,"        Using data point " << data_idx <<
                      " (" << x_data[data_idx] << "," << y_data[data_idx] << ") after");
            double term1 = yim1*(1-(dx-xim1)/(xi-xim1))-dy;
            double term2 = (dx-xim1)/(xi-xim1);
            b += 2*term1*term2;
            c += term2*term2;
        }
    } else {
        // all other control points
        double xim1 = x_sig[i-1];
        double xi   = x_sig[i  ];
        double xip1 = x_sig[i+1];
        double yim1 = y_sig[i-1];
        double yip1 = y_sig[i+1];
        for(int data_idx = data_per_segment[i].first; data_idx<=data_per_segment[i].second; ++data_idx) {
            double dx = x_data[data_idx];
            double dy = y_data[data_idx];
            DEBUG_OUT(4,"        Using data point " << data_idx <<
                      " (" << x_data[data_idx] << "," << y_data[data_idx] << ") before");
            double term1 = yim1*(1-(dx-xim1)/(xi-xim1))-dy;
            double term2 = (dx-xim1)/(xi-xim1);
            b += 2*term1*term2;
            c += term2*term2;
        }
        for(int data_idx = data_per_segment[i+1].first; data_idx<=data_per_segment[i+1].second; ++data_idx) {
            double dx = x_data[data_idx];
            double dy = y_data[data_idx];
            DEBUG_OUT(4,"        Using data point " << data_idx <<
                      " (" << x_data[data_idx] << "," << y_data[data_idx] << ") after");
            double term1 = 1-(dx-xi)/(xip1-xi);
            double term2 = yip1*(dx-xi)/(xip1-xi)-dy;
            b += 2*term1*term2;
            c += term1*term1;
        }
        // virtual point (mean of neighbors)
        double dy = (y_sig[i-1]+y_sig[i+1])/2;
        double dx = x_sig[i];
        double term1 = 1-(dx-xi)/(xip1-xi);
        double term2 = yip1*(dx-xi)/(xip1-xi)-dy;
        b += 2*term1*term2 * 1e-0;
        c += term1*term1 * 1e-0;
    }

    // add a bias
    b += bias_factor;

    // calculate new value
    double new_sig_y = c!=0?-b/(2*c):y_sig[i];
    if(i==0) {
        new_sig_y = min(new_sig_y,y_sig[i+1]);
    } else if(i==control_point_n-1) {
        new_sig_y = max(y_sig[i-1],new_sig_y);
    } else {
        new_sig_y = clamp(y_sig[i-1],y_sig[i+1],new_sig_y);
    }

    // set new value and return difference
    double dy = fabs(y_sig[i] - new_sig_y);
    y_sig[i] = new_sig_y;
    return dy;

    //===================================================//

    // // remember old value to evaluate effective change
    // double old_y_value = y_sig[i];

    // double last_rms_error = get_rms_error();
    // double step_size = 1; // start with this step size

    // // go fixed number of steps
    // for(int step_idx=0; step_idx<1000; ++step_idx) {
    //     // remember old value to reset in case of failure
    //     double old_y = y_sig[i];
    //     // apply random step
    //     y_sig[i] += (2*drand48()-1)*step_size;
    //     // make monoton
    //     if(i==0) {
    //         y_sig[i] = min<double>(y_sig[i+1],y_sig[i]);
    //     } else if(i==control_point_n-1) {
    //         y_sig[i] = max<double>(y_sig[i-1],y_sig[i]);
    //     } else {
    //         y_sig[i] = clamp<double>(y_sig[i-1],y_sig[i+1],y_sig[i]);
    //     }
    //     // get new rms error
    //     double current_rms_error = get_rms_error();
    //     if(current_rms_error>last_rms_error) {
    //         // don't apply
    //         y_sig[i] = old_y;
    //         // reduce step size
    //         step_size *= 0.5;
    //     } else {
    //         // update last rms error
    //         last_rms_error = current_rms_error;
    //         // increase step size
    //         step_size *= 1.2;
    //     }
    // }

    // // return effective y-change
    // return fabs(old_y_value-y_sig[i]);
}

void ActiveSigmoidOptimization::initialize_to_linear() {
    // get linear approximation
    double c1 = 0;
    double c2 = 0;
    double c3 = 0;
    double c4 = 0;
    int raw_data_n = min<int>(x_data.size(),y_data.size());
    for(int data_idx=0; data_idx<raw_data_n; ++data_idx) {
        c1 += x_data[data_idx];
        c2 += y_data[data_idx];
        c3 += x_data[data_idx] * x_data[data_idx];
        c4 += x_data[data_idx] * y_data[data_idx];
    }
    int D = raw_data_n;
    double beta = (c1*c2 - D*c4)/(c1*c1 - D*c3);
    double alpha = (c2 - c1*beta)/D;


    // set control points
    for(int p_idx=0; p_idx<control_point_n; ++p_idx) {
        y_sig[p_idx] = alpha + beta * x_sig[p_idx];
    }
}

// double ActiveSigmoidOptimization::get_rms_error() const {
//     int raw_data_n = min<int>(x_data.size(),y_data.size());
//     double squared_error_sum = 0;
//     for(int i=0; i<raw_data_n; ++i) {
//         double sig_value = SplineInterpolation::f(x_sig,y_sig,x_data[i]);
//         squared_error_sum += pow(y_data[i] - sig_value, 2);
//     }
//     return sqrt(squared_error_sum/raw_data_n) + bias_term() + stiffness_term();
// }

// double ActiveSigmoidOptimization::bias_term() const {
//     double sum = 0;
//     for(double y : y_sig) {
//         sum += y;
//     }
//     sum /= control_point_n;
//     sum *= bias_factor;
//     return sum;
// }

// double ActiveSigmoidOptimization::stiffness_term() const {
//     double sum = 0;
//     for(int idx=1; idx<control_point_n-1; ++idx) {
//         sum += pow( y_sig[idx] - (y_sig[idx-1] + y_sig[idx+1])/2 , 2 );
//     }
//     sum /= control_point_n-2;                     // take the mean
//     sum *= min<int>(x_data.size(),y_data.size()); // scale with number of data points
//     sum *= 1e-10;                                 // make relatively small
//     return sum;
// }
