#include "SmoothingKernelSigmoid.h"

#include <float.h> // for DBL_MAX

#include "qcustomplot.h"
#include "util.h"

#define DEBUG_LEVEL 1
#include "debug.h"

using util::min;
using util::max;
using util::clamp;
using util::sgn;
using std::vector;
using std::pair;
using std::make_pair;
using std::sort;

SmoothingKernelSigmoid::SmoothingKernelSigmoid(const double& width,
                                               const double& g,
                                               const double& min,
                                               const double& max):
    raw_data_n(0),
    kernel_width(width),
    gamma(g),
    min_x(min),
    max_x(max)
{}

SmoothingKernelSigmoid::~SmoothingKernelSigmoid() {}

void SmoothingKernelSigmoid::add_new_point(const double& x, const double& y) {
    // add point
    x_data.push_back(x);
    y_data.push_back(y);
    ++raw_data_n;

    DEBUG_OUT(2,"Added " << x_data.back() << "	" << y_data.back());
}

double SmoothingKernelSigmoid::get_max_uncertain(const int& sample_n) const {
    vector<double> x_max_dev_vec;
    double max_dev = -DBL_MAX;
    for(int i=0; i<sample_n; ++i) {
        double x = (double)i/(sample_n-1);
        x *= max_x-min_x;
        x += min_x;
        double mean, dev;
        mean_dev(x, mean, dev);
        if(dev>max_dev) {
            max_dev = dev;
            x_max_dev_vec.assign(1,x);
        } else if(dev==max_dev) {
            x_max_dev_vec.push_back(x);
        }
    }
    return util::random_select(x_max_dev_vec);
}

double SmoothingKernelSigmoid::print_to_QCP(QCustomPlot * plotter,
                                            const bool& raw_data,
                                            const bool& smooth_data,
                                            const bool& bounds,
                                            const bool& deviation) const {
    // initialize data
    int smooth_data_n = 1000;
    QVector<double> qx_raw_data(raw_data_n), qy_raw_data(raw_data_n);
    QVector<double> qx_smooth_data(smooth_data_n), qy_smooth_data(smooth_data_n);
    QVector<double> qx_smooth_upper(smooth_data_n), qy_smooth_upper(smooth_data_n);
    QVector<double> qx_smooth_lower(smooth_data_n), qy_smooth_lower(smooth_data_n);
    QVector<double> qx_smooth_dev(smooth_data_n), qy_smooth_dev(smooth_data_n);

    // transfer raw data to QVector
    for(int i=0; i<raw_data_n; ++i) {
        qx_raw_data[i] = x_data[i];
        qy_raw_data[i] = y_data[i];
    }

    // create smooth data
    vector<double> x_max_dev_vec;
    double max_dev = -DBL_MAX;
    for(int i=0; i<smooth_data_n; ++i) {
        double x = (double)i/(smooth_data_n-1);
        x *= max_x-min_x;
        x += min_x;
        double mean, dev;
        mean_dev(x, mean, dev);
        qx_smooth_data[i] = x;
        qy_smooth_data[i] = mean;
        qx_smooth_upper[i] = x;
        qy_smooth_upper[i] = mean+dev;
        qx_smooth_lower[i] = x;
        qy_smooth_lower[i] = mean-dev;
        qx_smooth_dev[i] = x;
        qy_smooth_dev[i] = dev;
        if(dev>max_dev) {
            max_dev = dev;
            x_max_dev_vec.assign(1,x);
        } else if(dev==max_dev) {
            x_max_dev_vec.push_back(x);
        }
    }

    // clear graphs
    plotter->clearGraphs();

    // create graph for smoothed upper bounds
    QCPGraph * smooth_upper_graph = plotter->addGraph();
    smooth_upper_graph->setPen(QColor(200, 0, 0, 200));
    smooth_upper_graph->setBrush(QColor(200, 0, 0, 50));
    smooth_upper_graph->setLineStyle(QCPGraph::lsLine);
    smooth_upper_graph->setScatterStyle(QCP::ScatterStyle::ssNone);
    smooth_upper_graph->setName("Upper Bound");
    smooth_upper_graph->setData(qx_smooth_upper, qy_smooth_upper);

    // create graph for smoothed lower bounds
    QCPGraph * smooth_lower_graph = plotter->addGraph();
    smooth_lower_graph->setPen(QColor(200, 0, 0, 200));
    smooth_lower_graph->setBrush(QColor(200, 0, 0, 50));
    smooth_lower_graph->setLineStyle(QCPGraph::lsLine);
    smooth_lower_graph->setScatterStyle(QCP::ScatterStyle::ssNone);
    smooth_lower_graph->setName("Lower Bound");
    smooth_lower_graph->setData(qx_smooth_lower, qy_smooth_lower);

    // create graph for raw data
    QCPGraph * raw_data_graph = plotter->addGraph();
    raw_data_graph->setPen(QColor(255, 250, 0, 255));
    raw_data_graph->setLineStyle(QCPGraph::lsNone);
    raw_data_graph->setScatterStyle(QCP::ScatterStyle::ssDisc);
    raw_data_graph->setName("Raw Data");
    raw_data_graph->setData(qx_raw_data, qy_raw_data);

    // create graph for smoothed data
    QCPGraph * smooth_data_graph = plotter->addGraph();
    smooth_data_graph->setPen(QColor(255, 250, 0, 255));
    smooth_data_graph->setLineStyle(QCPGraph::lsLine);
    smooth_data_graph->setScatterStyle(QCP::ScatterStyle::ssNone);
    smooth_data_graph->setName("Smoothed Data");
    smooth_data_graph->setData(qx_smooth_data, qy_smooth_data);

    // create graph for dev
    QCPGraph * smooth_dev_graph = plotter->addGraph();
    smooth_dev_graph->setPen(QColor(0, 0, 255, 255));
    smooth_dev_graph->setLineStyle(QCPGraph::lsLine);
    smooth_dev_graph->setScatterStyle(QCP::ScatterStyle::ssNone);
    smooth_dev_graph->setName("Deviation");
    smooth_dev_graph->setData(qx_smooth_dev, qy_smooth_dev);

    // fill smoothed bounds
    smooth_upper_graph->setChannelFillGraph(smooth_data_graph);
    smooth_lower_graph->setChannelFillGraph(smooth_data_graph);

    // give the axes some labels:
    plotter->xAxis->setLabel("x");
    plotter->yAxis->setLabel("y");

    // hide plots
    if(!raw_data) {
        plotter->removeGraph(raw_data_graph);
    }
    if(!smooth_data && !bounds) {
        plotter->removeGraph(smooth_data_graph);
    }
    if(!bounds) {
        plotter->removeGraph(smooth_upper_graph);
        plotter->removeGraph(smooth_lower_graph);
    }
    if(!deviation) {
        plotter->removeGraph(smooth_dev_graph);
    }

    // add title and legend
    plotter->setTitle("Smoothig Kernel Sigmoid");
    plotter->legend->setVisible(true);
    plotter->legend->setPositionStyle(QCPLegend::psBottomRight);
    QFont legendFont;
    legendFont.setPointSize(7);
    plotter->legend->setFont(legendFont);
    plotter->legend->setBrush(QBrush(QColor(255,255,255,150)));
    plotter->legend->setBorderPen(QPen(QColor(0,0,0,150)));

    // rescale axes and replot
    plotter->rescaleAxes();
    plotter->replot();

    return util::random_select(x_max_dev_vec);
}

double SmoothingKernelSigmoid::kernel(const double& x1, const double& x2) const {
    double d = fabs(x1-x2);
    double e = pow(d/kernel_width,gamma);
    return exp(-e);
}

void SmoothingKernelSigmoid::mean_dev(const double& x,
                                      double& mean,
                                      double& dev) const {

    mean = 0;                                    // sample mean
    double sq_mean = 0;                          // mean of squared samples
    double w_sum = 0;                            // sum of weights
    double sq_w_sum = 0;                         // sum of squared weights
    for(int data_idx=0; data_idx<raw_data_n; ++data_idx) {
        double w = kernel(x_data[data_idx],x);
        mean += w*y_data[data_idx];
        sq_mean += w*pow(y_data[data_idx],2);
        w_sum += w;
        sq_w_sum += pow(w,2);
    }
    mean /= w_sum;                               // normalize
    sq_mean /= w_sum;                            // normalize
    double sample_var = sq_mean - pow(mean,2);   // sample variance
    double mean_var = sample_var * sq_w_sum;     // variance of the mean
    dev = sqrt(mean_var);                        // standard deviaion of the mean

    dev /= sq_w_sum;                             // force a large deviation for unsampled points
}
