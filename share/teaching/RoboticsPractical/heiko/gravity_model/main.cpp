#include <Algo/MLcourse.h>
#include <string>
#include <iostream>
#include "../../interface/myBaxter.h"

void data2features(arr& X, arr Q, arr F){

    for(uint i = 0; i < Q.d0; i++){
        arr x = {};
        arr q = Q[i];

        x.append(1);
        x.append(q);
        //x.append(~q*q);
        x.append(sin(q));
        x.append(cos(q));
        x.append(F[i]);
        X.append(x);
    }

    X.reshape({Q.d0, X.d0/Q.d0});
}


arr getVarQuotient(arr X, arr Y, arr Coeffs){
        arr Mean = ridgeRegression(ones(Y.d0, 1), Y);

        arr MSE_const = zeros(17);
        arr MSE_model = zeros(17);

        for(uint i = 0; i < Y.d0; i++){
                arr Model_i = ~Coeffs * X[i];

                MSE_const += ((Y[i] - Mean) % (Y[i] - Mean));
                MSE_model += ((Y[i] - Model_i) % (Y[i] - Model_i));
        }

        MSE_const /= (double) Y.d0;
        MSE_model /= (double) Y.d0;

        return sqrt(MSE_model) / sqrt(MSE_const);
}

int main(int argc, char** argv){
    mlr::initCmdLine(argc, argv);
    mlr::String datafile = mlr::getParameter<mlr::String>("datafile", mlr::String("data.csv"));
    {
        arr data;
        data << FILE(datafile);

        arr X, Y, Q, F, M, X_est, Q_est, F_est, Coeffs;

        Q = data.sub(0,-1, 0,16);
        F = data.sub(0,-1, 17,33);
        Y = data.sub(0,-1, 34,-1);


        data2features(X, Q, F);

        Coeffs = ridgeRegression(X, Y);


        data2features(X_est, Q.row(0), F.row(0));

        std::cout << getVarQuotient(X, Y, Coeffs);


        //std::cout << Coeffs << std::endl << std::endl;
        //std::cout << Mean << std::endl;
    }

}
