#include "Nelder_Mead_Simplex.h"
#include <cmath>

/*
 * Constructor
 */
Nelder_Mead_Simplex::Nelder_Mead_Simplex(Matrix <double> start, double initLength, double (*_function_double)(Matrix <double> vars))
{
	function_double = _function_double;
	n = start.rows();
	Matrix <double> p (n+1,n, 0.0);
	points = p;
	c = initLength;
	b = (c/(n*sqrt(2.0))) * (sqrt(n+1.0) - 1.0);
	a = b + c/sqrt(2.0);
	for(unsigned int i = 0; i < n; i++)
	{
		points(0,i) = start(0,i);
	}
	for(unsigned int i = 1; i <= n; i++)
	{
		for(unsigned int j = 0; j < n; j++)
		{
			points(i,j) = b + start(0,j);
		}
		points(i,i-1) = a + start(0,i-1);
	}
}

/*
 * Functions and Procedures
 */

Matrix <double> Nelder_Mead_Simplex::minimizeFunction()
{
	bool converged = false;
	evaluatePoints();
	while(converged)
	{
		bool mustShrink = true;
		Matrix <double> Xa = findXa();
		reflection(Xa);
		double fXr = function_double(Xr);
		if(fXr < f_b)
		{
			mustShrink = false;
			Matrix <double> Xe = expansion(Xa);
			if(function_double(Xe) < f_b) //expansion better than best
			{
				//add Xe
				addPoint(Xe);
			} else
			{
				//add Xr
				addPoint(Xr);
			}
		} else if(fXr > f_w) //reflection worst than worse
		{
			Matrix <double> Xc = in_contraction(Xa);
			if(function_double(Xc) <= f_w)
			{
				//add Xc
				addPoint(Xc);
				mustShrink = false;
			}
		} else if(fXr < f_w && fXr > f_l)
		{
			Matrix <double> Xo = out_contraction(Xa);
			if(function_double(Xo) <= fXr) //better or equal to reflected
			{
				addPoint(Xo);
				mustShrink = false;
			}
		} else
		{
			addPoint(Xr);
			mustShrink = false;
		}
		if(mustShrink) shrink();
		evaluatePoints();
		if(f_w - f_b < pow(10, -10.0) + pow(10,-4)*std::abs(f_b)) converged = true;
	}
	return Xb;
}

Matrix <double> Nelder_Mead_Simplex::minimizeFunction(std::ofstream & outfile)
{
	bool converged = false;
	unsigned int iteration = 0;
	evaluatePoints();
	outfile << "Nelder-Mead method.\n";
	for(unsigned int i = 0; i < points.rows(); i++)
	{
		outfile << "X" << i+1 << ",";
	}
	outfile << "F(X)\n";
	outfile << "Iteration: " << iteration << "\n";
	outfile << "Points:\n";
	for(unsigned int i = 0; i < points.cols(); i++)
	{
		for(unsigned int j = 0; j < points.rows(); j++)
		{
			outfile << points(i,j) <<",";
		}
		outfile << function_double(points.getCol(i)) << "\n";
	}
	while(!converged)
	{
		bool mustShrink = true;
		Matrix <double> Xa = findXa();
		reflection(Xa);
		double fXr = function_double(Xr);
		if(fXr < f_b)
		{
			mustShrink = false;
			Matrix <double> Xe = expansion(Xa);
			if(function_double(Xe) < f_b) //expansion better than best
			{
				//add Xe
				addPoint(Xe);
				outfile << "Expansion\n";
			} else
			{
				//add Xr
				addPoint(Xr);
				outfile << "Reflection\n";
			}
		} else if(fXr > f_w) //reflection worst than worse
		{
			Matrix <double> Xc = in_contraction(Xa);
			if(function_double(Xc) <= f_w)
			{
				//add Xc
				addPoint(Xc);
				mustShrink = false;
				outfile << "Inner Contraction\n";
			}
		} else if(fXr < f_w && fXr > f_l)
		{
			Matrix <double> Xo = out_contraction(Xa);
			if(function_double(Xo) <= fXr) //better or equal to reflected
			{
				addPoint(Xo);
				mustShrink = false;
				outfile << "Outer Contraction\n";
			}
		} else
		{
			addPoint(Xr);
			mustShrink = false;
			outfile << "Reflection between best and second worst points.\n";
		}
		if(mustShrink)
		{
			shrink();
			outfile << "Shrink\n";
		}
		Matrix <double> lastXb = Xb;
		double lastf_b = f_b;
		evaluatePoints();
		iteration++;
		outfile << "Iteration: " << iteration << "\n";
		outfile << "Points:\n";
		for(unsigned int i = 0; i < points.cols(); i++)
		{
			for(unsigned int j = 0; j < points.rows(); j++)
			{
				outfile << points(i,j) <<",";
			}
			outfile << function_double(points.getCol(i)) << "\n";
		}
		double convergence = (Xb - lastXb).Magnitude()/(1+Xb.Magnitude()) + std::abs(f_b - lastf_b)/(1+ std::abs(f_b));
		outfile << "Convergence:,,," << convergence << "\n";
		if(f_w - f_b < pow(10, -6.0) + pow(10,-4)*std::abs(f_b)) converged = true;
	}
	return Xb;
}

void Nelder_Mead_Simplex::evaluatePoints()
{
	Xb = points.getCol(0); //best
	Xw = points.getCol(0); //worst
	Xl = points.getCol(0); //lousy/second worst
	f_b = function_double(Xb);
	f_w = function_double(Xw);
	f_l = function_double(Xl);
	for(unsigned int i = 1; i < points.cols(); i++)
	{
		Matrix <double> tmp = points.getCol(i);
		double f_tmp = function_double(tmp);
		if(f_tmp < function_double(Xb)) //point better than best
		{
			Xb = tmp;
			f_b = function_double(Xb);
		}
		if(f_tmp > function_double(Xw)) //point worse than worst
		{
			Xl = Xw;
			f_l = f_w;
			Xw = tmp;
			f_w = function_double(Xw);
		}
		if(f_tmp > function_double(Xl) && f_tmp < function_double(Xw)) //between worst and second worst
		{
			Xl = tmp;
			f_l = function_double(Xl);
		}
	}
}

Matrix <double> Nelder_Mead_Simplex::findXa()
{
	Matrix <double> Xa (1,n, 0.0); //average
	for(unsigned int i = 0; i < points.cols(); i++)
	{
		Matrix <double> tmp = points.getCol(i);
		if(!(tmp == Xw)) Xa = Xa + tmp;
	}
	Xa = Xa/n;
	return Xa;
}

void Nelder_Mead_Simplex::reflection(Matrix <double> Xa)
{
	Xr = Xa + (Xa - Xw)*ALPHA;
}

Matrix <double> Nelder_Mead_Simplex::expansion(Matrix <double> Xa)
{
	Matrix <double> Xe = Xr + (Xr-Xa)*GAMMA;
	return Xe;
}

Matrix <double> Nelder_Mead_Simplex::in_contraction(Matrix <double> Xa)
{
	Matrix <double> Xc = Xa - (Xa - Xw)*BETA;
	return Xc;
}

Matrix <double> Nelder_Mead_Simplex::out_contraction(Matrix <double> Xa)
{
	Matrix <double> Xo = Xa + (Xa - Xw)*BETA;
	return Xo;
}

void Nelder_Mead_Simplex::shrink()
{
	Matrix <double> p(0, 0, 0);
	for(unsigned int i = 0; i < points.cols(); i++)
	{
		Matrix <double> Xi = Xb + (points.getCol(i) - Xb)*RHO;
		p = p.addCol(Xi);
	}
	points = p;
	return;
}

void Nelder_Mead_Simplex::addPoint(Matrix <double> newPoint)
{
	Matrix <double> p(0, 0, 0);
	for(unsigned int i = 0; i < points.cols(); i++)
	{
		if(!(points.getCol(i)==Xw))
		{
			p = p.addCol(points.getCol(i));
		}
	}
	p = p.addCol(newPoint);
	points = p;
	return;
}

Matrix <double> Nelder_Mead_Simplex::getPoints()
{
	return points;
}
