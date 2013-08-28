#include "SmoothingKernelSigmoid.h"

#include <math.h> // for exp
#include <float.h> // for DBL_MAX

#include "qcustomplot.h"
#include "util.h"
#include "util/ProgressBar.h"
#include "lbfgs_codes.h"

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

SmoothingKernelSigmoid::SmoothingKernelSigmoid(const double& min_x,
                                               const double& max_x,
                                               const int& control_point_n_,
                                               const double& width,
                                               const double& g
    ):
    control_point_n(control_point_n_),
    raw_data_n(0),
    initialized(false),
    bias_factor(0),
    kernel_width(width),
    gamma(g),
    infinity(DBL_MAX/1e10),
    bound_scaling(1e-100),
    x_sig(control_point_n),
    y_sig(control_point_n),
    y_sig_upper(y_sig),
    y_sig_lower(y_sig),
    y_values(nullptr)
{
    // initialize x-values
    double x = min_x;
    double x_step = (max_x-min_x)/(control_point_n-1);
    for(int idx=0; idx<control_point_n; ++idx, x+=x_step) {
        x_sig[idx] = x;
    }
}

SmoothingKernelSigmoid::~SmoothingKernelSigmoid() {
    lbfgs_free(y_values);
}

void SmoothingKernelSigmoid::add_new_point(const double& x, const double& y) {
    // add point
    x_data.push_back(x);
    y_data.push_back(y);
    ++raw_data_n;

    DEBUG_OUT(2,"Added " << x_data.back() << "	" << y_data.back());

    // data need to be (re)initialized
    initialized = false;
}

double SmoothingKernelSigmoid::print_to_QCP(QCustomPlot * plotter,
                                          const bool& print_upper,
                                          const bool& print_lower
    ) const {
    // initialize data
    int smooth_data_n = 1000;
    QVector<double> qx_raw_data(raw_data_n), qy_raw_data(raw_data_n);
    // QVector<double> qx_sig(control_point_n), qy_sig(control_point_n);
    // QVector<double> qx_sig_upper(control_point_n), qy_sig_upper(control_point_n);
    // QVector<double> qx_sig_lower(control_point_n), qy_sig_lower(control_point_n);
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
        x *= x_sig.back()-x_sig.front();
        x += x_sig.front();
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

    // // transfer points to QVector
    // for(int i=0; i<control_point_n; ++i) {
    //     qx_sig[i]       = x_sig[i];
    //     qx_sig_upper[i] = x_sig[i];
    //     qx_sig_lower[i] = x_sig[i];
    //     qy_sig[i]       = y_sig[i];
    //     qy_sig_upper[i] = y_sig_upper[i];
    //     qy_sig_lower[i] = y_sig_lower[i];
    // }

    // clear graphs
    plotter->clearGraphs();

    // // create graph for upper bounds
    // QCPGraph * upper_graph = plotter->addGraph();
    // upper_graph->setPen(QColor(200, 0, 0, 200));
    // upper_graph->setBrush(QColor(200, 0, 0, 50));
    // upper_graph->setLineStyle(QCPGraph::lsLine);
    // upper_graph->setScatterStyle(QCP::ScatterStyle::ssDisc);
    // upper_graph->setScatterSize(2);
    // upper_graph->setName("Upper Bound");
    // upper_graph->setData(qx_sig, qy_sig_upper);

    // // create graph for lower bounds
    // QCPGraph * lower_graph = plotter->addGraph();
    // lower_graph->setPen(QColor(200, 0, 0, 200));
    // lower_graph->setBrush(QColor(200, 0, 0, 50));
    // lower_graph->setLineStyle(QCPGraph::lsLine);
    // lower_graph->setScatterStyle(QCP::ScatterStyle::ssDisc);
    // lower_graph->setScatterSize(2);
    // lower_graph->setName("Lower Bound");
    // lower_graph->setData(qx_sig, qy_sig_lower);

    // // create graph for sigmoid
    // QCPGraph * sig_graph = plotter->addGraph();
    // sig_graph->setPen(QColor(0, 200, 0, 255));
    // sig_graph->setLineStyle(QCPGraph::lsLine);
    // sig_graph->setScatterStyle(QCP::ScatterStyle::ssDiamond);
    // lower_graph->setScatterSize(3);
    // sig_graph->setName("Mean");
    // sig_graph->setData(qx_sig, qy_sig);

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

    // // fill bounds or remove (removing changes indices)
    // if(print_upper) {
    //     upper_graph->setChannelFillGraph(sig_graph);
    // } else {
    //     plotter->removeGraph(upper_graph);
    // }
    // if(print_lower) {
    //     lower_graph->setChannelFillGraph(sig_graph);
    // } else {
    //     plotter->removeGraph(lower_graph);
    // }

    // give the axes some labels:
    plotter->xAxis->setLabel("x");
    plotter->yAxis->setLabel("y");

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

void SmoothingKernelSigmoid::optimize_sigmoid(const int& iterations) {
    DEBUG_OUT(2,"Optimizing sigmoid");
    initialize();
    bias_factor = 0;
    optimize_model(iterations);
}

void SmoothingKernelSigmoid::optimize_upper_bound(const double& strength, const int& iterations) {
    DEBUG_OUT(2,"Optimizing upper bound with bias strength " << strength);
    vector<double> y_tmp = y_sig;
    bias_factor = strength;
    optimize_model(iterations);
    y_sig_upper = y_sig;
    y_sig = y_tmp;
    bias_factor = 0;
}

void SmoothingKernelSigmoid::optimize_lower_bound(const double& strength, const int& iterations) {
    DEBUG_OUT(2,"Optimizing lower bound with bias strength " << strength);
    vector<double> y_tmp = y_sig;
    bias_factor = -strength;
    optimize_model(iterations);
    y_sig_lower = y_sig;
    y_sig = y_tmp;
    bias_factor = 0;
}

lbfgsfloatval_t SmoothingKernelSigmoid::static_evaluate_model(
        void *instance,
        const lbfgsfloatval_t *y,
        lbfgsfloatval_t *g,
        const int n,
        const lbfgsfloatval_t /*step*/
) {
    return ((SmoothingKernelSigmoid*)instance)->evaluate_model(y,g,n);
}

lbfgsfloatval_t SmoothingKernelSigmoid::evaluate_model(
        const lbfgsfloatval_t *y,
        lbfgsfloatval_t *g,
        const int n
) {

    lbfgsfloatval_t fx = 0;
    for(int i=0; i<n; i++) {
        g[i] = 0;
    }

    // // iterate through data
    // if(DEBUG_LEVEL>0) {ProgressBar::init("Evaluating: ");}
    // // update progress bar
    // if(DEBUG_LEVEL>0) {ProgressBar::print(idx, control_point_n-1);}
    // // terminate progress bar
    // if(DEBUG_LEVEL>0) {ProgressBar::terminate();}

    // precomputing n_eff and nu_eff
    DEBUG_OUT(3,"Precomputing n_eff and nu_eff:");
    vector<double> n_eff_vec(control_point_n,0);
    vector<double> nu_eff_vec(control_point_n,0);
    for(int point_idx=0; point_idx<control_point_n; ++point_idx) {
        for(int data_idx=0; data_idx<raw_data_n; ++data_idx) {
            n_eff_vec[point_idx] += kernel(x_sig[point_idx],x_data[data_idx]);
        }
        for(int point_2_idx=0; point_2_idx<raw_data_n; ++point_2_idx) {
            if(point_2_idx==point_idx) {
                continue;
            } else {
                nu_eff_vec[point_idx] += kernel(x_sig[point_idx],x_sig[point_2_idx]);
            }
        }
        DEBUG_OUT(3,"    " << point_idx << "	n_eff:	" << n_eff_vec[point_idx] <<
                  "	nu_eff:	" << nu_eff_vec[point_idx]);
    }

    // compute cost function and gradient
    for(int point_1_idx=0; point_1_idx<control_point_n; ++point_1_idx) {
        double cx = x_sig[point_1_idx];
        double cy = y[point_1_idx];
        double n_eff = n_eff_vec[point_1_idx];

        // data loss
        double y_smooth = kernel_smoothed_data(cx);
        fx += pow((y_smooth-cy)*n_eff,2);
        g[point_1_idx] += -2*n_eff*n_eff*(y_smooth-cy);

        // monotony bounds
        for(int point_2_idx=0; point_2_idx<control_point_n; ++point_2_idx) {
            double cost_1, cost_2, grad_1, grad_2;
            // one way
            bounds(y[point_1_idx] - y[point_2_idx], cost_1, grad_1);
            fx += cost_1;
            g[point_1_idx] += grad_1;
            g[point_2_idx] += -grad_1;
            // other way
            bounds(y[point_2_idx] - y[point_1_idx], cost_2, grad_2);
            fx += cost_2;
            g[point_1_idx] += -grad_2;
            g[point_2_idx] += grad_2;
        }

        // bias
        fx += cy*bias_factor;
        g[point_1_idx] += bias_factor;
    }

    return fx;
}

int SmoothingKernelSigmoid::static_progress_model(
        void *instance,
        const lbfgsfloatval_t *x,
        const lbfgsfloatval_t *g,
        const lbfgsfloatval_t fx,
        const lbfgsfloatval_t xnorm,
        const lbfgsfloatval_t gnorm,
        const lbfgsfloatval_t step,
        int n,
        int k,
        int ls
) {
    return ((SmoothingKernelSigmoid*)instance)->progress_model(x,g,fx,xnorm,gnorm,step,n,k,ls);
}

int SmoothingKernelSigmoid::progress_model(
        const lbfgsfloatval_t *x,
        const lbfgsfloatval_t * /*g*/,
        const lbfgsfloatval_t fx,
        const lbfgsfloatval_t /*xnorm*/,
        const lbfgsfloatval_t /*gnorm*/,
        const lbfgsfloatval_t /*step*/,
        int /*n*/,
        int k,
        int /*ls*/
) {

    DEBUG_OUT(1,"Iteration " << k << " (fx = " << fx << ")");
    if(DEBUG_LEVEL>=3) {
        for(int p_idx=0; p_idx<control_point_n; ++p_idx) {
            DEBUG_OUT(0, "    control point " << p_idx << ": " << x[p_idx]);
        }
        DEBUG_OUT(0,"Iteration " << k << " (fx = " << fx << ")");
        DEBUG_OUT(1,"");
    }

    return 0;
}

int SmoothingKernelSigmoid::optimize_model(unsigned int max_iter) {

    // Initialize the parameters for the L-BFGS optimization.
    lbfgs_parameter_t param;
    lbfgs_parameter_init(&param);
    // param.orthantwise_c = l1;
    // param.linesearch = LBFGS_LINESEARCH_BACKTRACKING;
    if(max_iter>0) {
        param.max_iterations = max_iter;
    }

    // todo what values
    param.delta = 1e-3;   // change of objective (f-f')/f (default 0)
    param.epsilon = 1e-5; // change of parameters ||g||/max(1,||x||) (default 1e-5)

    // Start the L-BFGS optimization
    lbfgsfloatval_t fx;
    int ret = lbfgs(control_point_n, y_values, &fx, static_evaluate_model, static_progress_model, this, &param);

    // transfer
    lbfgs_to_vec();

    // Report the result.
    DEBUG_OUT(1, "L-BFGS optimization terminated with status code = " << ret << " ( " << lbfgs_code(ret) << " )");
    DEBUG_OUT(1,"final cost = " << fx );
    DEBUG_OUT(1,"");

    return ret;
}

void SmoothingKernelSigmoid::check_derivatives(const int& number_of_samples, const double& range, const double& max_variation, const double& max_relative_deviation) {

    // initialize arrays
    lbfgsfloatval_t * x = lbfgs_malloc(control_point_n);
    lbfgsfloatval_t * dx = lbfgs_malloc(control_point_n);
    lbfgsfloatval_t * grad = lbfgs_malloc(control_point_n);
    lbfgsfloatval_t * grad_dummy = lbfgs_malloc(control_point_n);

    // remember deviations
    lbfgsfloatval_t relative_deviation = 0;

    DEBUG_OUT(0,"Checking first derivative (#samples="<<number_of_samples<<", range=+/-"<<range<<", max_var="<<max_variation<<", max_rel_dev="<<max_relative_deviation);
    for(int count=0; count<number_of_samples; ++count) {

        // set test point
        for(int x_idx=0; x_idx<control_point_n; ++x_idx) {
            x[x_idx] = range * (2*drand48()-1);
            dx[x_idx] = (2*drand48()-1)*max_variation;
        }

        lbfgsfloatval_t fx = evaluate_model(x,grad,control_point_n);
        DEBUG_OUT(1, "fx = " << fx );
        for(int x_idx=0; x_idx<control_point_n; ++x_idx) {

            // go in positive direction
            x[x_idx] += dx[x_idx]/2.;
            lbfgsfloatval_t fx_plus = evaluate_model(x,grad_dummy,control_point_n);
            // go in negative direction
            x[x_idx] -= dx[x_idx];
            lbfgsfloatval_t fx_minus = evaluate_model(x,grad_dummy,control_point_n);
            // reset x
            x[x_idx] += dx[x_idx]/2.;

            // numerical gradient
            lbfgsfloatval_t ngrad = (fx_plus-fx_minus)/dx[x_idx];

            // check for deviations
            lbfgsfloatval_t current_relative_deviation = fabs(ngrad-grad[x_idx])/fabs(grad[x_idx]);
            if(current_relative_deviation>relative_deviation) {
                relative_deviation=current_relative_deviation;
            }

            // print result
            DEBUG_OUT(1,
                      "    diff[" << x_idx << "] = " << grad[x_idx]-ngrad <<
                      ", grad["   << x_idx << "] = " << grad[x_idx] <<
                      ", ngrad["  << x_idx << "] = " << ngrad <<
                      ", x["      << x_idx << "] = " << x[x_idx] <<
                      ", dx["     << x_idx << "] = " << dx[x_idx] <<
                      ", rel_dev["     << x_idx << "] = " << current_relative_deviation
                );


        }
    }
    if(relative_deviation>max_relative_deviation) {
        DEBUG_OUT(0, "ERRORS in first derivative found: max relative deviation = " << relative_deviation << " (tolerance = " << max_relative_deviation << ")" );
        DEBUG_OUT(0, "");
    } else {
        DEBUG_OUT(0, "No error in first derivative found (no relative deviations larger that " << max_relative_deviation << ").");
        DEBUG_OUT(0, "");
    }
    lbfgs_free(x);
    lbfgs_free(dx);
    lbfgs_free(grad);
    lbfgs_free(grad_dummy);
}

void SmoothingKernelSigmoid::initialize() {
    // get linear approximation
    double c1 = 0;
    double c2 = 0;
    double c3 = 0;
    double c4 = 0;
    for(int data_idx=0; data_idx<raw_data_n; ++data_idx) {
        c1 += x_data[data_idx];
        c2 += y_data[data_idx];
        c3 += x_data[data_idx] * x_data[data_idx];
        c4 += x_data[data_idx] * y_data[data_idx];
    }
    int D = raw_data_n;
    double beta = (c1*c2 - D*c4)/(c1*c1 - D*c3);
    double alpha = (c2 - c1*beta)/D;

    // set vector for lbfgs
    y_values = lbfgs_malloc(control_point_n);

    // set control points
    for(int p_idx=0; p_idx<control_point_n; ++p_idx) {
        y_sig[p_idx] = alpha + beta * x_sig[p_idx];
    }
    vec_to_lbfgs();
}

void SmoothingKernelSigmoid::lbfgs_to_vec() {
    for(int p_idx=0; p_idx<control_point_n; ++p_idx) {
        y_sig[p_idx] = y_values[p_idx];
    }
}

void SmoothingKernelSigmoid::vec_to_lbfgs() {
    for(int p_idx=0; p_idx<control_point_n; ++p_idx) {
        y_values[p_idx] = y_sig[p_idx];
    }
}

double SmoothingKernelSigmoid::kernel(const double& x1, const double& x2) const {
    double d = fabs(x1-x2);
    double e = pow(d/kernel_width,gamma);
    return exp(-e);
}

double SmoothingKernelSigmoid::kernel_smoothed_data(const double& x) const {
    double n_eff = 0;
    double y_sum = 0;
    for(int data_idx=0; data_idx<raw_data_n; ++data_idx) {
        double k = kernel(x_data[data_idx],x);
        y_sum += k*y_data[data_idx];
        n_eff += k;
    }
    return y_sum/n_eff;
}

void SmoothingKernelSigmoid::bounds(const double& d,
                                    double& cost,
                                    double& grad) const {
    //=========================================================//
    // 1/(1+r^2) sigmoid
    //=========================================================//
    // double d_scale = 1e-2;
    // double sig_scale = 1e1;
    // cost = d/(d_scale*sqrt(1+pow(d/d_scale,2)));
    // grad = d_scale/(sqrt(pow(d,2)/pow(d_scale,2)+1)*(pow(d,2)+pow(d_scale,2)));
    // cost *= sig_scale;
    // grad *= sig_scale;
    //=========================================================//
    // linear
    //=========================================================//
    // double factor = 1e1;
    // if(d>0) {
    //     cost = factor*d*d;
    //     grad = 2*factor*d;
    // } else {
    //     cost = 0;
    //     grad = 0;
    // }
    //=========================================================//
    // linear
    //=========================================================//
    double l = 1e1;
    double f = 1e-1;
    if(d<-l) {
        cost = 1-2*((d+l)/l);
        grad = -2/l;
    } else if(d>-l && d<0) {
        cost = pow(d/l,2);
        grad = 2*d/(l*l);
    } else {
        cost = 0;
        grad = 0;
    }
    cost *= f;
    grad *= f;
}

void SmoothingKernelSigmoid::mean_dev(const double& x,
                                      double& mean,
                                      double& dev) const {
    mean = 0;                // sample mean
    double sq_mean = 0;      // mean of squared samples
    double w_sum = 0;        // sum of weights
    double sq_w_sum = 0;     // sum of squared weights
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

    dev /= sq_w_sum;
}
