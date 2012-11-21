/*
 * lbfgs_codes.h
 *
* Created on: Nov 21, 2012
* Author: robert
*/

#ifndef LBFGS_CODES_H_
#define LBFGS_CODES_H_

#include <lbfgs.h>

#include <string>

std::string lbfgs_code(const int& code) {

    std::string code_str = "";

    if(code==LBFGS_SUCCESS) {
        if(code_str!="") { code_str += " / "; }
        code_str += "LBFGS_SUCCESS";
    }
    if(code==LBFGS_CONVERGENCE) {
        if(code_str!="") { code_str += " / "; }
        code_str += "LBFGS_CONVERGENCE";
    }
    if(code==LBFGS_STOP) {
        if(code_str!="") { code_str += " / "; }
        code_str += "LBFGS_STOP";
    }
    if(code==LBFGS_ALREADY_MINIMIZED) {
        if(code_str!="") { code_str += " / "; }
        code_str += "LBFGS_ALREADY_MINIMIZED";
    }
    if(code==LBFGSERR_UNKNOWNERROR) {
        if(code_str!="") { code_str += " / "; }
        code_str += "LBFGSERR_UNKNOWNERROR";
    }
    if(code==LBFGSERR_LOGICERROR) {
        if(code_str!="") { code_str += " / "; }
        code_str += "LBFGSERR_LOGICERROR";
    }
    if(code==LBFGSERR_OUTOFMEMORY) {
        if(code_str!="") { code_str += " / "; }
        code_str += "LBFGSERR_OUTOFMEMORY";
    }
    if(code==LBFGSERR_CANCELED) {
        if(code_str!="") { code_str += " / "; }
        code_str += "LBFGSERR_CANCELED";
    }
    if(code==LBFGSERR_INVALID_N) {
        if(code_str!="") { code_str += " / "; }
        code_str += "LBFGSERR_INVALID_N";
    }
    if(code==LBFGSERR_INVALID_N_SSE) {
        if(code_str!="") { code_str += " / "; }
        code_str += "LBFGSERR_INVALID_N_SSE";
    }
    if(code==LBFGSERR_INVALID_X_SSE) {
        if(code_str!="") { code_str += " / "; }
        code_str += "LBFGSERR_INVALID_X_SSE";
    }
    if(code==LBFGSERR_INVALID_EPSILON) {
        if(code_str!="") { code_str += " / "; }
        code_str += "LBFGSERR_INVALID_EPSILON";
    }
    if(code==LBFGSERR_INVALID_TESTPERIOD) {
        if(code_str!="") { code_str += " / "; }
        code_str += "LBFGSERR_INVALID_TESTPERIOD";
    }
    if(code==LBFGSERR_INVALID_DELTA) {
        if(code_str!="") { code_str += " / "; }
        code_str += "LBFGSERR_INVALID_DELTA";
    }
    if(code==LBFGSERR_INVALID_LINESEARCH) {
        if(code_str!="") { code_str += " / "; }
        code_str += "LBFGSERR_INVALID_LINESEARCH";
    }
    if(code==LBFGSERR_INVALID_MINSTEP) {
        if(code_str!="") { code_str += " / "; }
        code_str += "LBFGSERR_INVALID_MINSTEP";
    }
    if(code==LBFGSERR_INVALID_MAXSTEP) {
        if(code_str!="") { code_str += " / "; }
        code_str += "LBFGSERR_INVALID_MAXSTEP";
    }
    if(code==LBFGSERR_INVALID_FTOL) {
        if(code_str!="") { code_str += " / "; }
        code_str += "LBFGSERR_INVALID_FTOL";
    }
    if(code==LBFGSERR_INVALID_WOLFE) {
        if(code_str!="") { code_str += " / "; }
        code_str += "LBFGSERR_INVALID_WOLFE";
    }
    if(code==LBFGSERR_INVALID_GTOL) {
        if(code_str!="") { code_str += " / "; }
        code_str += "LBFGSERR_INVALID_GTOL";
    }
    if(code==LBFGSERR_INVALID_XTOL) {
        if(code_str!="") { code_str += " / "; }
        code_str += "LBFGSERR_INVALID_XTOL";
    }
    if(code==LBFGSERR_INVALID_MAXLINESEARCH) {
        if(code_str!="") { code_str += " / "; }
        code_str += "LBFGSERR_INVALID_MAXLINESEARCH";
    }
    if(code==LBFGSERR_INVALID_ORTHANTWISE) {
        if(code_str!="") { code_str += " / "; }
        code_str += "LBFGSERR_INVALID_ORTHANTWISE";
    }
    if(code==LBFGSERR_INVALID_ORTHANTWISE_START) {
        if(code_str!="") { code_str += " / "; }
        code_str += "LBFGSERR_INVALID_ORTHANTWISE_START";
    }
    if(code==LBFGSERR_INVALID_ORTHANTWISE_END) {
        if(code_str!="") { code_str += " / "; }
        code_str += "LBFGSERR_INVALID_ORTHANTWISE_END";
    }
    if(code==LBFGSERR_OUTOFINTERVAL) {
        if(code_str!="") { code_str += " / "; }
        code_str += "LBFGSERR_OUTOFINTERVAL";
    }
    if(code==LBFGSERR_INCORRECT_TMINMAX) {
        if(code_str!="") { code_str += " / "; }
        code_str += "LBFGSERR_INCORRECT_TMINMAX";
    }
    if(code==LBFGSERR_ROUNDING_ERROR) {
        if(code_str!="") { code_str += " / "; }
        code_str += "LBFGSERR_ROUNDING_ERROR";
    }
    if(code==LBFGSERR_MINIMUMSTEP) {
        if(code_str!="") { code_str += " / "; }
        code_str += "LBFGSERR_MINIMUMSTEP";
    }
    if(code==LBFGSERR_MAXIMUMSTEP) {
        if(code_str!="") { code_str += " / "; }
        code_str += "LBFGSERR_MAXIMUMSTEP";
    }
    if(code==LBFGSERR_MAXIMUMLINESEARCH) {
        if(code_str!="") { code_str += " / "; }
        code_str += "LBFGSERR_MAXIMUMLINESEARCH";
    }
    if(code==LBFGSERR_MAXIMUMITERATION) {
        if(code_str!="") { code_str += " / "; }
        code_str += "LBFGSERR_MAXIMUMITERATION";
    }
    if(code==LBFGSERR_WIDTHTOOSMALL) {
        if(code_str!="") { code_str += " / "; }
        code_str += "LBFGSERR_WIDTHTOOSMALL";
    }
    if(code==LBFGSERR_INVALIDPARAMETERS) {
        if(code_str!="") { code_str += " / "; }
        code_str += "LBFGSERR_INVALIDPARAMETERS";
    }
    if(code==LBFGSERR_INCREASEGRADIENT) {
        if(code_str!="") { code_str += " / "; }
        code_str += "LBFGSERR_INCREASEGRADIENT";
    }

    return code_str;
}

//---------------------------//
//  List of all error codes  //
//---------------------------//
//LBFGS_SUCCESS
//LBFGS_CONVERGENCE
//LBFGS_STOP
//LBFGS_ALREADY_MINIMIZED
//LBFGSERR_UNKNOWNERROR
//LBFGSERR_LOGICERROR
//LBFGSERR_OUTOFMEMORY
//LBFGSERR_CANCELED
//LBFGSERR_INVALID_N
//LBFGSERR_INVALID_N_SSE
//LBFGSERR_INVALID_X_SSE
//LBFGSERR_INVALID_EPSILON
//LBFGSERR_INVALID_TESTPERIOD
//LBFGSERR_INVALID_DELTA
//LBFGSERR_INVALID_LINESEARCH
//LBFGSERR_INVALID_MINSTEP
//LBFGSERR_INVALID_MAXSTEP
//LBFGSERR_INVALID_FTOL
//LBFGSERR_INVALID_WOLFE
//LBFGSERR_INVALID_GTOL
//LBFGSERR_INVALID_XTOL
//LBFGSERR_INVALID_MAXLINESEARCH
//LBFGSERR_INVALID_ORTHANTWISE
//LBFGSERR_INVALID_ORTHANTWISE_START
//LBFGSERR_INVALID_ORTHANTWISE_END
//LBFGSERR_OUTOFINTERVAL
//LBFGSERR_INCORRECT_TMINMAX
//LBFGSERR_ROUNDING_ERROR
//LBFGSERR_MINIMUMSTEP
//LBFGSERR_MAXIMUMSTEP
//LBFGSERR_MAXIMUMLINESEARCH
//LBFGSERR_MAXIMUMITERATION
//LBFGSERR_WIDTHTOOSMALL
//LBFGSERR_INVALIDPARAMETERS
//LBFGSERR_INCREASEGRADIENT

//--------------------------------//
//  List of all linesearch codes  //
//--------------------------------//
//LBFGS_LINESEARCH_DEFAULT
//LBFGS_LINESEARCH_MORETHUENTE
//LBFGS_LINESEARCH_BACKTRACKING_ARMIJO
//LBFGS_LINESEARCH_BACKTRACKING
//LBFGS_LINESEARCH_BACKTRACKING_WOLFE
//LBFGS_LINESEARCH_BACKTRACKING_STRONG_WOLFE

#endif /* LBFGS_CODES_H_ */
