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
    vector<double> x_max_uncertain_vec;
    double max_uncertain = -DBL_MAX;
    for(int i=0; i<sample_n; ++i) {
        double x = (double)i/(sample_n-1);
        x *= max_x-min_x;
        x += min_x;
        double mean, mean_dev, dev;
        mean_dev_weights(x, mean, mean_dev, dev);
        if(mean_dev>max_uncertain) {
            max_uncertain = mean_dev;
            x_max_uncertain_vec.assign(1,x);
        } else if(mean_dev==max_uncertain) {
            x_max_uncertain_vec.push_back(x);
        }
    }
    return util::random_select(x_max_uncertain_vec);
}

void SmoothingKernelSigmoid::print_to_QCP(QCustomPlot * plotter,
                                          const bool& raw_data,
                                          const bool& smooth_data,
                                          const bool& bounds,
                                          const bool& deviation,
                                          const bool& uncert) const {
    // initialize data
    int smoothing_n = 1000;
    QVector<double> qx_raw_data(raw_data_n), qy_raw_data(raw_data_n);
    QVector<double> qx(smoothing_n);
    QVector<double> qy_mean(smoothing_n),
        qy_upper_dev(smoothing_n),
        qy_lower_dev(smoothing_n),
        qy_upper_uncertain(smoothing_n),
        qy_lower_uncertain(smoothing_n),
        qy_uncertain(smoothing_n);

    // transfer raw data to QVector
    for(int i=0; i<raw_data_n; ++i) {
        qx_raw_data[i] = x_data[i];
        qy_raw_data[i] = y_data[i];
    }

    // create smooth data
    for(int i=0; i<smoothing_n; ++i) {
        double x = (double)i/(smoothing_n-1);
        x *= max_x-min_x;
        x += min_x;
        double mean, mean_dev, dev;
        mean_dev_weights(x, mean, mean_dev, dev);
        qx[i] = x;
        qy_mean[i] = mean;
        qy_upper_dev[i] = mean+dev;
        qy_lower_dev[i] = mean-dev;
        qy_upper_uncertain[i] = mean+mean_dev;
        qy_lower_uncertain[i] = mean-mean_dev;
        qy_uncertain[i] = mean_dev;
    }

    // // Note: QCustomPlot sometimes hangs on drawing the plot inside
    // // QPainter::drawPolyline(). I could not observe this when using 100 instead
    // // of 1000 point for showing smooth plots. Also I seems to me, that for many
    // // data points is occurs less frequently. Smoothing the data to prevent
    // // large steps (see below) does not help.
    // QVector<double> tmp_qy_mean = qy_mean;
    // QVector<double> tmp_qy_upper_dev = qy_upper_dev;
    // QVector<double> tmp_qy_lower_dev = qy_lower_dev;
    // QVector<double> tmp_qy_upper_uncertain = qy_upper_uncertain;
    // QVector<double> tmp_qy_lower_uncertain = qy_lower_uncertain;
    // QVector<double> tmp_qy_uncertain = qy_uncertain;
    // for(int i=0; i<smoothing_n; ++i) {
    //     int sum = 10;
    //     qy_mean[i] = 10*tmp_qy_mean[i];
    //     qy_upper_dev[i] = 10*tmp_qy_upper_dev[i];
    //     qy_lower_dev[i] = 10*tmp_qy_lower_dev[i];
    //     qy_upper_uncertain[i] = 10*tmp_qy_upper_uncertain[i];
    //     qy_lower_uncertain[i] = 10*tmp_qy_lower_uncertain[i];
    //     qy_uncertain[i] = 10*tmp_qy_uncertain[i];
    //     if(i>0) {
    //         qy_mean[i] += tmp_qy_mean[i-1];
    //         qy_upper_dev[i] += tmp_qy_upper_dev[i-1];
    //         qy_lower_dev[i] += tmp_qy_lower_dev[i-1];
    //         qy_upper_uncertain[i] += tmp_qy_upper_uncertain[i-1];
    //         qy_lower_uncertain[i] += tmp_qy_lower_uncertain[i-1];
    //         qy_uncertain[i] += tmp_qy_uncertain[i-1];
    //         ++sum;
    //     }
    //     if(i<smoothing_n-1) {
    //         qy_mean[i] += tmp_qy_mean[i+1];
    //         qy_upper_dev[i] += tmp_qy_upper_dev[i+1];
    //         qy_lower_dev[i] += tmp_qy_lower_dev[i+1];
    //         qy_upper_uncertain[i] += tmp_qy_upper_uncertain[i+1];
    //         qy_lower_uncertain[i] += tmp_qy_lower_uncertain[i+1];
    //         qy_uncertain[i] += tmp_qy_uncertain[i+1];
    //         ++sum;
    //     }
    //     qy_mean[i] /= sum;
    //     qy_upper_dev[i] /= sum;
    //     qy_lower_dev[i] /= sum;
    //     qy_upper_uncertain[i] /= sum;
    //     qy_lower_uncertain[i] /= sum;
    //     qy_uncertain[i] /= sum;
    // }

    // clear graphs
    plotter->clearGraphs();

    // define some colors etc
    QColor data_col             =        QColor(255, 250,   0, 255);
    QPen   dev_pen              = QPen(  QColor(200,   0,   0, 255));
    QBrush dev_brush            = QBrush(QColor(200,   0,   0,  50));
    QPen   uncertain_pen        = QPen(  QColor(100,   0,   0, 255));
    QBrush uncertain_brush      = QBrush(QColor(  0,   0,   0,  10));
    QPen   uncertain_base_pen   = QPen(  QColor(  0,   0,   0, 255));
    uncertain_pen.setStyle(Qt::DotLine);

    // create graph for smoothed upper bounds
    QCPGraph * upper_dev_graph = plotter->addGraph();
    upper_dev_graph->setPen(dev_pen);
    upper_dev_graph->setBrush(dev_brush);
    upper_dev_graph->setLineStyle(QCPGraph::lsLine);
    upper_dev_graph->setScatterStyle(QCP::ScatterStyle::ssNone);
    upper_dev_graph->setName("Standard Error");
    upper_dev_graph->setData(qx, qy_upper_dev);

    // create graph for smoothed lower bounds
    QCPGraph * lower_dev_graph = plotter->addGraph();
    lower_dev_graph->setPen(dev_pen);
    lower_dev_graph->setBrush(dev_brush);
    lower_dev_graph->setLineStyle(QCPGraph::lsLine);
    lower_dev_graph->setScatterStyle(QCP::ScatterStyle::ssNone);
    lower_dev_graph->setName("");
    lower_dev_graph->setData(qx, qy_lower_dev);

    // create graph for uncertainty
    QCPGraph * upper_uncertain_graph = plotter->addGraph();
    upper_uncertain_graph->setPen(uncertain_pen);
    upper_uncertain_graph->setBrush(uncertain_brush);
    upper_uncertain_graph->setLineStyle(QCPGraph::lsLine);
    upper_uncertain_graph->setScatterStyle(QCP::ScatterStyle::ssNone);
    upper_uncertain_graph->setName("Standar Error\nof the Mean");
    upper_uncertain_graph->setData(qx, qy_upper_uncertain);

    // create graph for uncertainty
    QCPGraph * lower_uncertain_graph = plotter->addGraph();
    lower_uncertain_graph->setPen(uncertain_pen);
    lower_uncertain_graph->setBrush(uncertain_brush);
    lower_uncertain_graph->setLineStyle(QCPGraph::lsLine);
    lower_uncertain_graph->setScatterStyle(QCP::ScatterStyle::ssNone);
    lower_uncertain_graph->setName("");
    lower_uncertain_graph->setData(qx, qy_lower_uncertain);

    // create graph for uncertainty
    QCPGraph * uncertain_graph = plotter->addGraph();
    uncertain_graph->setPen(uncertain_base_pen);
    uncertain_graph->setLineStyle(QCPGraph::lsLine);
    uncertain_graph->setScatterStyle(QCP::ScatterStyle::ssNone);
    uncertain_graph->setName("");
    uncertain_graph->setData(qx, qy_uncertain);

    // create graph for raw data
    QCPGraph * raw_data_graph = plotter->addGraph();
    raw_data_graph->setPen(data_col);
    raw_data_graph->setLineStyle(QCPGraph::lsNone);
    raw_data_graph->setScatterStyle(QCP::ScatterStyle::ssDisc);
    raw_data_graph->setName("Raw Data");
    raw_data_graph->setData(qx_raw_data, qy_raw_data);

    // create graph for smoothed data
    QCPGraph * mean_graph = plotter->addGraph();
    mean_graph->setPen(data_col);
    mean_graph->setLineStyle(QCPGraph::lsLine);
    mean_graph->setScatterStyle(QCP::ScatterStyle::ssNone);
    mean_graph->setName("Mean");
    mean_graph->setData(qx, qy_mean);

    // fill bounds
    upper_dev_graph->setChannelFillGraph(mean_graph);
    lower_dev_graph->setChannelFillGraph(mean_graph);
    upper_uncertain_graph->setChannelFillGraph(mean_graph);
    lower_uncertain_graph->setChannelFillGraph(mean_graph);

    // give the axes some labels:
    plotter->xAxis->setLabel("x");
    plotter->yAxis->setLabel("y");

    // hide plots
    if(!raw_data) {
        plotter->removeGraph(raw_data_graph);
    }
    if(!smooth_data && !bounds && !uncert) {
        plotter->removeGraph(mean_graph);
    }
    if(!bounds) {
        plotter->removeGraph(upper_dev_graph);
        plotter->removeGraph(lower_dev_graph);
    }
    if(!deviation) {
        plotter->removeGraph(upper_dev_graph);
        plotter->removeGraph(lower_dev_graph);
    }
    if(!uncert) {
        plotter->removeGraph(upper_uncertain_graph);
        plotter->removeGraph(lower_uncertain_graph);
        plotter->removeGraph(uncertain_graph);
    }

    // add title and legend
    plotter->setTitle("Smoothig Kernel Sigmoid");
    QCPLegend * legend = plotter->legend;
    legend->setVisible(true);
    legend->setPositionStyle(QCPLegend::psBottomRight);
    QFont legendFont;
    legendFont.setPointSize(7);
    legend->setFont(legendFont);
    legend->setBrush(QBrush(QColor(255,255,255,150)));
    legend->setBorderPen(QPen(QColor(0,0,0,150)));
    legend->removeItem(legend->itemWithPlottable(lower_dev_graph));
    legend->removeItem(legend->itemWithPlottable(lower_uncertain_graph));
    legend->removeItem(legend->itemWithPlottable(uncertain_graph));

    // rescale axes and replot
    plotter->rescaleAxes();
    plotter->replot();
}

void SmoothingKernelSigmoid::mean_dev_weights(const double& x,
                                              double& mean,
                                              double& mean_dev,
                                              double& dev) const {

    // when no data are available
    if(raw_data_n==0) {
        mean = 0;
        mean_dev = 1;
        dev = 1;
        return;
    }

    double mean_sq = 0;                          // mean of squared samples
    mean = 0;                                    // sample mean
    double w_sum = 0;                            // sum of weights
    double w_sq_sum = 0;                         // sum of squared weights
    for(int data_idx=0; data_idx<raw_data_n; ++data_idx) {
        double px = x_data[data_idx];
        double py = y_data[data_idx];
        double w = kernel(px,x);
        mean += w*py;
        mean_sq += w*py*py;
        w_sum += w;
        w_sq_sum += w*w;
    }

    mean /= w_sum;                               // normalize
    mean_sq /= w_sum;                            // normalize
    double sample_var = mean_sq - pow(mean,2);   // sample variance
    double mean_var = sample_var / w_sq_sum;     // variance of the mean
    mean_dev = sqrt(mean_var);                   // standard deviaion of the mean
    dev = sqrt(sample_var);                      // standard deviaion of the sample
}

double SmoothingKernelSigmoid::kernel(const double& x1, const double& x2) const {
    double d = fabs(x1-x2);
    double e = pow(d/kernel_width,gamma);
    return exp(-e);
}
