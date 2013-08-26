#ifndef NELDER_MEAD_SIMPLEX_H
#define NELDER_MEAD_SIMPLEX_H

#include "Matrix.h"

#include <fstream>

#define GAMMA 1.0 //expansion parameter
#define BETA 0.5 //contraction factor
#define RHO 0.5 //scaling parameter
#define ALPHA 1.0 //reflection factor

class Nelder_Mead_Simplex {

public:
    Nelder_Mead_Simplex(Matrix <double> start, double initLength, double (*_function_double)(Matrix <double> vars));
    Matrix <double> minimizeFunction();
    Matrix <double> minimizeFunction(std::ofstream & outfile);
    void evaluatePoints();
    void reflect();
    Matrix <double> findXa();
    void reflection(Matrix <double> Xa);
    Matrix <double> expansion(Matrix <double> Xa);
    Matrix <double> in_contraction(Matrix <double> Xa);
    Matrix <double> out_contraction(Matrix <double> Xa);
    void shrink();
    void addPoint(Matrix <double> newPoint);
    Matrix <double> getPoints();

private:
    Matrix <double> points;
    Matrix <double> Xb, Xw, Xl, Xr;
    double f_b, f_w, f_l;
    double a;
    double b;
    double c;
    double n;
    double (*function_double)(Matrix <double> vars);
};

#endif // NELDER_MEAD_SIMPLEX_H
