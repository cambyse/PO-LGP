#include <iostream>
#include <vector>
#include <math.h>

using std::vector;
using std::cout;
using std::endl;

typedef vector<double> vec_double_1D;
typedef vector<vec_double_1D> vec_double_2D;
typedef vector<vec_double_2D> vec_double_3D;

vector<int> Range(int n) {
    vector<int> ret;
    for(int i=0; i<n; ++i) {
        ret.push_back(i);
    }
    return ret;
}

template<class C>
C clamp(C lower, C upper, C value) {
    if(value<lower) return lower;
    if(value>upper) return upper;
    return value;
}

// likelihood
vec_double_1D p_y_I_x0_t0(double p) { return vec_double_1D({p,1-p}); }
vec_double_1D p_y_I_x1_t0(double p) { return vec_double_1D({p,1-p}); }
vec_double_1D p_y_I_x0_t1(double p) { return vec_double_1D({p,1-p}); }
vec_double_1D p_y_I_x1_t1(double p) { return vec_double_1D({p,1-p}); }

vec_double_2D p_y_I_x_t0(double p1, double p2) { return vec_double_2D({p_y_I_x0_t0(p1), p_y_I_x1_t0(p2)}); }
vec_double_2D p_y_I_x_t1(double p1, double p2) { return vec_double_2D({p_y_I_x0_t1(p1), p_y_I_x1_t1(p2)}); }

//vec_double_3D p_y_I_x_t(double p1 = 0.75, double p2 = 0.01, double p3 = 0.1, double p4 = 0.9) {
vec_double_3D p_y_I_x_t(double p_y0_I_x0_t0,
                        double p_y0_I_x1_t0,
                        double p_y0_I_x0_t1,
                        double p_y0_I_x1_t1) {
    return vec_double_3D({p_y_I_x_t0( p_y0_I_x0_t0, p_y0_I_x1_t0), p_y_I_x_t1( p_y0_I_x0_t1, p_y0_I_x1_t1)});
}

// prior
vec_double_1D p_t(double p_t0) { return vec_double_1D({p_t0,1-p_t0}); }

// observation marginal
double p_y_I_x(int x,
               int y,
               double p_y0_I_x0_t0,
               double p_y0_I_x1_t0,
               double p_y0_I_x0_t1,
               double p_y0_I_x1_t1,
               double p_t0) {
    double p_y_I_x_ = 0;
    for(int t : {0,1}) {
        p_y_I_x_ += p_y_I_x_t(p_y0_I_x0_t0, p_y0_I_x1_t0, p_y0_I_x0_t1, p_y0_I_x1_t1)[t][x][y]*p_t(p_t0)[t];
    }
    return p_y_I_x_;
}

// posterior
vec_double_1D p_t_I_x_y(int x,
                        int y,
                        double p_y0_I_x0_t0,
                        double p_y0_I_x1_t0,
                        double p_y0_I_x0_t1,
                        double p_y0_I_x1_t1,
                        double p_t0) {
    vec_double_1D p_t_I_x_y_({0,0});
    for(int t : {0,1}) {
        p_t_I_x_y_[t] = p_y_I_x_t(p_y0_I_x0_t0, p_y0_I_x1_t0, p_y0_I_x0_t1, p_y0_I_x1_t1)[t][x][y]*p_t(p_t0)[t]/p_y_I_x(x, y, p_y0_I_x0_t0, p_y0_I_x1_t0, p_y0_I_x0_t1, p_y0_I_x1_t1, p_t0);
    }
    return p_t_I_x_y_;
}

// entropy
double H(vec_double_1D p) {
    double H_ = 0;
    for(int idx : Range(p.size())) {
        H_ += -p[idx]*log(p[idx]);
    }
    return H_;
}

// cross entropy
double Hx(vec_double_1D p1, vec_double_1D p2) {
    double Hx_ = 0;
    for(int idx : Range(std::min(p1.size(),p2.size()))) {
        Hx_ += -p1[idx]*log(p2[idx]);
    }
    return Hx_;
}

// expected entropy
double EH_p_t_I_x_y(int x,
                    double p_y0_I_x0_t0,
                    double p_y0_I_x1_t0,
                    double p_y0_I_x0_t1,
                    double p_y0_I_x1_t1,
                    double p_t0) {
    double EH_p_t_I_x_y_ = 0;
    for(int y : {0,1}) {
        EH_p_t_I_x_y_ += p_y_I_x(x,y,p_y0_I_x0_t0, p_y0_I_x1_t0, p_y0_I_x0_t1, p_y0_I_x1_t1, p_t0)*H(p_t_I_x_y(x,y,p_y0_I_x0_t0, p_y0_I_x1_t0, p_y0_I_x0_t1, p_y0_I_x1_t1, p_t0));
    }
    return EH_p_t_I_x_y_;
}

// expected cross entropy
double EHx_p_t_I_x_y(int x,
                     double p_y0_I_x0_t0,
                     double p_y0_I_x1_t0,
                     double p_y0_I_x0_t1,
                     double p_y0_I_x1_t1,
                     double p_t0) {
    double EHx_p_t_I_x_y_ = 0;
    for(int y : {0,1}) {
        EHx_p_t_I_x_y_ += p_y_I_x(x,y,p_y0_I_x0_t0, p_y0_I_x1_t0, p_y0_I_x0_t1, p_y0_I_x1_t1, p_t0)*Hx(p_t(p_t0),p_t_I_x_y(x,y,p_y0_I_x0_t0, p_y0_I_x1_t0, p_y0_I_x0_t1, p_y0_I_x1_t1, p_t0));
    }
    return EHx_p_t_I_x_y_;
}

int main(int, char**) {
    // double p_y0_I_x0_t0 = 0.5;
    // double p_y0_I_x1_t0 = 0.5;
    // double p_y0_I_x0_t1 = 0.5;
    // double p_y0_I_x1_t1 = 0.5;
    // double p_t0 = 0.5;

    // double p_y0_I_x0_t0 = 0.142;
    // double p_y0_I_x1_t0 = 0.01;
    // double p_y0_I_x0_t1 = 0.943;
    // double p_y0_I_x1_t1 = 0.728;
    // double p_t0 = 0.281;

    double p_y0_I_x0_t0 = 0.078;
    double p_y0_I_x1_t0 = 0.001;
    double p_y0_I_x0_t1 = 0.953;
    double p_y0_I_x1_t1 = 0.727;
    double p_t0 = 0.361;

    bool first_run = true;
    double step = 1e-7;
    double objective = 0; // will be initialized in first iteration
    double tolerance = 1e-3;

    for(int counter : Range(1000000)) {

        double p_y0_I_x0_t0_old = p_y0_I_x0_t0;
        double p_y0_I_x1_t0_old = p_y0_I_x1_t0;
        double p_y0_I_x0_t1_old = p_y0_I_x0_t1;
        double p_y0_I_x1_t1_old = p_y0_I_x1_t1;
        double p_t0_old         = p_t0;

        if(!first_run) {
            p_y0_I_x0_t0 = clamp(tolerance,1-tolerance,p_y0_I_x0_t0+2*step*(drand48()-0.5));
            p_y0_I_x1_t0 = clamp(tolerance,1-tolerance,p_y0_I_x1_t0+2*step*(drand48()-0.5));
            p_y0_I_x0_t1 = clamp(tolerance,1-tolerance,p_y0_I_x0_t1+2*step*(drand48()-0.5));
            p_y0_I_x1_t1 = clamp(tolerance,1-tolerance,p_y0_I_x1_t1+2*step*(drand48()-0.5));
            p_t0         = clamp(tolerance,1-tolerance,        p_t0+2*step*(drand48()-0.5));
        }

        vec_double_1D EHx({EHx_p_t_I_x_y(0, p_y0_I_x0_t0, p_y0_I_x1_t0, p_y0_I_x0_t1, p_y0_I_x1_t1, p_t0),
                    EHx_p_t_I_x_y(1, p_y0_I_x0_t0, p_y0_I_x1_t0, p_y0_I_x0_t1, p_y0_I_x1_t1, p_t0)});
        vec_double_1D negEH({-EH_p_t_I_x_y(0, p_y0_I_x0_t0, p_y0_I_x1_t0, p_y0_I_x0_t1, p_y0_I_x1_t1, p_t0),
                    -EH_p_t_I_x_y(1, p_y0_I_x0_t0, p_y0_I_x1_t0, p_y0_I_x0_t1, p_y0_I_x1_t1, p_t0)});

        double new_objective = (EHx[0]-EHx[1])*(negEH[0]-negEH[1]);

        bool accept = first_run || new_objective<objective;

        if(first_run) {
            first_run = false;
            cout << "FIRST RUN" << endl;
        }

        if(accept) {
            cout << p_y0_I_x0_t0 << " / " <<
                p_y0_I_x1_t0 << " / " <<
                p_y0_I_x0_t1 << " / " <<
                p_y0_I_x1_t1 << " / " <<
                p_t0 << endl;
            cout << "p(y|x=0,t=0): " << p_y_I_x0_t0(p_y0_I_x0_t0)[0] << "	" << p_y_I_x0_t0(p_y0_I_x0_t0)[1] << endl;
            cout << "p(y|x=1,t=0): " << p_y_I_x0_t0(p_y0_I_x1_t0)[0] << "	" << p_y_I_x0_t0(p_y0_I_x1_t0)[1] << endl;
            cout << "p(y|x=0,t=1): " << p_y_I_x0_t0(p_y0_I_x0_t1)[0] << "	" << p_y_I_x0_t0(p_y0_I_x0_t1)[1] << endl;
            cout << "p(y|x=1,t=1): " << p_y_I_x0_t0(p_y0_I_x1_t1)[0] << "	" << p_y_I_x0_t0(p_y0_I_x1_t1)[1] << endl;
            cout << "        p(t): " << p_t(p_t0)[0] << "	" << p_t(p_t0)[1] << endl;
            cout << "Expected Cross Entropy: " << EHx[0] << "	" << EHx[1] << " (" << EHx[0]-EHx[1] << ")" << endl;
            cout << "Expected  Neg. Entropy: " << negEH[0] << "	" << negEH[1] << " (" << negEH[0]-negEH[1] << ")" << endl;
            cout << "New Objective: " << new_objective << endl;
        }

        if(accept) {
            objective = new_objective;
            cout << "ACCEPT" << endl;
        } else {
            p_y0_I_x0_t0 = p_y0_I_x0_t0_old;
            p_y0_I_x1_t0 = p_y0_I_x1_t0_old;
            p_y0_I_x0_t1 = p_y0_I_x0_t1_old;
            p_y0_I_x1_t1 = p_y0_I_x1_t1_old;
            p_t0         = p_t0_old;
            //cout << "REJECT" << endl;
        }
    }

    return 0;
}
