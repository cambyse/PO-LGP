/*
 * Matrix Class Version 1.02
 * Right now Matrix could theoretically exist that is not rectangular. This should be optimized if possible.
*/


#ifndef _Matrix
#define _Matrix
#include <iostream>
#include <vector>
#include <cmath>

template <class T>
class Matrix;

template <class T>
std::ostream & operator << (std::ostream & output, Matrix <T> & mat);


template <class T>
class Matrix
{
	private:
	std::vector <std::vector <T> > mat;

	public:
	Matrix(unsigned int, unsigned int, T);
	Matrix(unsigned int); //sets up identity matrix
	Matrix(unsigned int, unsigned int, std::vector <Matrix <T> > mats); //fills up each row before moving vertically
	//Matrix(unsigned int, unsigned int, vector <T> values)
	Matrix();
	Matrix operator + (Matrix mat2);
	Matrix operator - (Matrix mat2);
	Matrix operator * (Matrix mat2);
	Matrix operator * (double n);
	Matrix operator / (double n);
	bool operator == (const Matrix mat2);
	Matrix CrossProd(Matrix <T> mat2); //only for 3D at the moment
	double DotProd(Matrix <T> mat2);
	T Magnitude();
	Matrix getRow(unsigned int);
	Matrix getCol(unsigned int);
	Matrix addRow(Matrix <T> mat2);
	Matrix addCol(Matrix <T> mat2);
	Matrix LU();
	Matrix Inverse();
	Matrix Transpose();
	Matrix Cofactor();
	Matrix SubMatrix(unsigned int, unsigned int);
	T Determinant();
	bool set(unsigned int col, unsigned int row, T value);
	unsigned int rows();
	unsigned int cols();
	int definite(); //2 positive, 1 semi-positive, -1 semi-negative, -2 negative, 0 zero definite, -3 indefinite
	bool isZeroMatrix();
	T & operator () (unsigned int col, unsigned int row);
	friend std::ostream & operator << <T> (std::ostream & output, Matrix <T> & mat);
};

template <class T>
Matrix<T>::Matrix()
{
	mat.clear();
}

template <class T>
Matrix<T>::Matrix(unsigned int cols, unsigned int rows, T values)
{
	T x=values;
	mat.clear();
	for(unsigned int i=0; i<cols; i++)
	{
		mat.push_back(std::vector <T> ());
		for(unsigned int j=0; j<rows; j++)
		{
			mat[i].push_back(x);
			mat[i][j]=values;
		}
	}
}

template <class T>
Matrix<T>::Matrix(unsigned int cols)
{
	mat.clear();
	for(unsigned int i=0; i<cols; i++)
	{
		mat.push_back(std::vector <T> ());
		for(unsigned int j=0; j<cols; j++)
		{
			mat[i].push_back(0);
			if(i==j) mat[i][j] = 1;
		}
	}
}

template <class T>
Matrix<T>::Matrix(unsigned int cols, unsigned int rows, std::vector <Matrix <T> > mats)
{
	mat.clear();
	for(unsigned int i=0; i < cols; i++)
	{
		mat.push_back(std::vector <T> ());
		for(unsigned int j = 0; j < rows; j++)
		{
			mat[i].push_back(0);
		}
	}
	unsigned int x = 0;
	unsigned int y = 0;
	for(unsigned int k = 0; k<mats.size(); k++)
	{
		for(unsigned int i = 0; i < mats[k].cols(); i++)
		{
			for(unsigned int j = 0; j < mats[k].rows(); j++)
			{
				mat[i+x][j+y] = mats[k](i,j);
			}
		}
		x+=mats[k].cols();
		if(x == mat.size())
		{
			x = 0;
			y+= mats[k].rows();
		}
	}
}

template <class T>
Matrix <T> Matrix<T>::operator + (Matrix mat2)
{
	if(mat2.rows()==rows() && mat2.cols()==cols())
	{
		Matrix ret(cols(), rows(), 0);
		for(unsigned int i=0; i<cols(); i++)
		{
			for(unsigned int j=0; j<rows(); j++)
			{
				ret(i, j)=mat2(i, j)+mat[i][j];
			}
		}
		return ret;
	} else
	{
		return Matrix <T> ();
	}
}


template <class T>
Matrix <T> Matrix<T>::operator - (Matrix mat2)
{
	if(mat2.rows()==rows() && mat2.cols()==cols())
	{
		Matrix ret(cols(), rows(), 0);
		for(unsigned int i=0; i<cols(); i++)
		{
			for(unsigned int j=0; j<rows(); j++)
			{
				ret(i, j)=mat[i][j]-mat2(i, j);
			}
		}
		return ret;
	} else
	{
		return Matrix <T> ();
	}
}


template <class T>
Matrix <T> Matrix<T>::operator * (Matrix mat2)
{
	if(cols()==mat2.rows())
	{
		Matrix ret(mat2.cols(), rows(), 0);
		for(unsigned int i=0; i<ret.cols(); i++)
		{
			for(unsigned int j = 0; j < ret.rows(); j++)
			{
				for(unsigned int k = 0; k < cols(); k++)
				{
					ret(i,j)=ret(i,j)+mat[k][j]*mat2(i, k);
				}
			}
		}
		return ret;
	} else
	{
		return Matrix <T> ();
	}
}

template <class T>
Matrix <T> Matrix<T>::operator * (double n)
{
	Matrix ret(cols(), rows(), 0);
	for(unsigned int i = 0; i < cols(); i++)
	{
		for(unsigned int j = 0; j < rows(); j++)
		{
			ret.set(i, j, mat[i][j]*n);
		}
	}
	return ret;
}

template <class T>
Matrix <T> Matrix<T>::operator / (double n)
{
	Matrix ret(cols(), rows(), 0);
	for(unsigned int i = 0; i < cols(); i++)
	{
		for(unsigned int j = 0; j < rows(); j++)
		{
			ret.set(i, j, mat[i][j]/n);
		}
	}
	return ret;
}

template <class T>
T & Matrix<T>::operator () (unsigned int col, unsigned int row)
{
	return mat[col][row];
}

template <class T>
bool Matrix<T>::operator == (Matrix mat2)
{
	if(cols() != mat2.cols()) return false;
	if(rows() != mat2.rows()) return false;
	for(unsigned int i = 0; i < cols(); i++)
	{
		for(unsigned int j = 0; j < rows(); j++)
		{
			if(mat[i][j] != mat2(i,j)) return false;
		}
	}
	return true;
}

template <class T>
Matrix <T> Matrix <T>::CrossProd(Matrix <T> mat2)
{
	Matrix <T> result (1,this->rows(),0);
	if(result.rows() != mat2.rows()) return result;
	result(0,0) = mat[0][1]*mat2(0,2) - mat[0][2]*mat2(0,1);
	result(0,1) = mat[0][2]*mat2(0,0) - mat[0][0]*mat2(0,2);
	result(0,2) = mat[0][0]*mat2(0,1) - mat[0][1]*mat2(0,0);
	return result;
}

template <class T>
double Matrix<T>::DotProd(Matrix <T> mat2)
{
	double result = 0.0;
	for(unsigned int i = 0; i < rows; i++)
	{
		result += mat[0][i]*mat2(0,i);
	}
	return result;
}


template <class T>
T Matrix<T>::Magnitude()
{
	T result = 0;
	for(unsigned int i = 0; i < this->rows(); i++)
	{
		result += pow(mat[0][i],2.0);
	}
	result = sqrt(result);
	return result;
}

template <class T>
bool Matrix<T>::set(unsigned int col, unsigned int row, T value)
{
	if (row > rows() || col > cols()) return false;
	mat[col][row]=value;
	return true;
}

template <class T>
Matrix <T> Matrix <T>::getRow(unsigned int r)
{
	Matrix <T> result(cols(),1, 0);
	for(unsigned int i = 0; i < cols(); i++)
	{
		result(i,0) = mat[i][r];
	}
	return result;
}

template <class T>
Matrix <T> Matrix <T>::getCol(unsigned int c)
{
	Matrix <T> result(1,rows(), 0);
	for(unsigned int i = 0; i < rows(); i++)
	{
		result(0,i) = mat[c][i];
	}
	return result;
}

template <class T>
Matrix <T> Matrix <T>::addRow(Matrix <T> newRow)
{
	Matrix <T> result(newRow.cols(), rows()+1, 0);
	for(unsigned int i = 0; i < newRow.cols(); i++)
	{
		for(unsigned int j = 0; j < rows(); j++)
		{
			result(i,j) = mat[i][j];
		}
		result(i, rows()) = newRow(i,0);
	}
	return result;
}

template <class T>
Matrix <T> Matrix <T>::addCol(Matrix <T> newCol)
{
	Matrix <T> result(cols()+1, newCol.rows(), 0);
	for(unsigned int j = 0; j < newCol.rows(); j++)
	{
		for(unsigned int i = 0; i < cols(); i++)
		{
			result(i,j) = mat[i][j];
		}
		result(cols(), j) = newCol(0,j);
	}
	return result;
}

template <class T>
Matrix <T> Matrix<T>::SubMatrix(unsigned int i, unsigned int j)
{
	Matrix <T> sub(cols()-1, rows()-1, 0);
	unsigned int Ai=0;
	for(unsigned int x=0; x<sub.cols(); x++)
	{
		if(Ai==i) Ai++;
		unsigned int Aj=0;
		for(unsigned int y=0; y<sub.rows(); y++)
		{
			if(Aj==j) Aj++;
			sub(x, y)=mat[Ai][Aj];
			Aj++;
		}
		Ai++;
	}
	return sub;
}

template <class T>
Matrix <T> Matrix<T>::Transpose()
{
	Matrix <T> tmat(rows(), cols(), 0);
	for(unsigned int i=0; i<cols(); i++)
	{
		for(unsigned int j=0; j<rows(); j++)
		{
			tmat(j, i)=mat[i][j];
		}
	}
	return tmat;
}

template <class T>
Matrix <T> Matrix<T>::Cofactor()
{
	Matrix <T> cof(cols(), rows(), 0);
	for(unsigned int i=0; i<cols(); i++)
	{
		for(unsigned int j=0; j<rows(); j++)
		{
			#ifdef VERBOSE
			std::cout << "On submatrix "<<i<<", "<<j<<".\n";
			#endif
			Matrix <T> sub=SubMatrix(i, j);
			#ifdef VERBOSE
			std::cout << "Taking Determinant.\n";
			#endif
			T d=sub.Determinant();
			if((i+1+j+1)%2==0)
			{
				cof(i, j)=d;
			} else
			{
				cof(i, j)=d*-1;
			}
			#ifdef VERBOSE
			std::cout << "Moving to next.\n";
			#endif
		}
	}
	return cof;
}

template <class T>
Matrix<T> Matrix<T>::Inverse()
{
	#ifdef VERBOSE
	std::cout <<"Taking Determinant of Initial Matrix\n";
	#endif
	T d=this->Determinant();
	Matrix <T> inv=Cofactor().Transpose();
	for(unsigned int i=0; i<cols(); i++)
	{
		for(unsigned int j=0; j<rows(); j++)
		{
			inv(i, j)=inv(i, j)/d;
		}
	}
	return inv;
}

template <class T>
Matrix <T> Matrix<T>::LU()
{
	Matrix <T> LU=*this;
	for(unsigned int k = 0; k<cols()-1; k++)
	{
		for(unsigned int x = k+1; x<cols(); x++)
		{
			LU(x, k) = LU(x,k) / LU(k,k);
		}
        for(unsigned int i=k+1; i<cols(); i++)
        {
			for(unsigned int j=k+1; j<rows(); j++)
			{
				LU(i, j) = LU(i,j) - LU(i,k)*LU(k,j);
			}
		}
	}
	return LU;
}

template <class T>
T Matrix<T>::Determinant()
{
	T det=1;
	Matrix <T> LU=this->LU();
	for(unsigned int i=0; i<LU.cols(); i++)
	{
		det=det*LU(i, i);
	}
	return det;
}


template <class T>
std::ostream & operator << (std::ostream & output, Matrix <T> & mat)
{
	output<<"[\n";
	for(unsigned int j=0; j<mat.rows(); j++)
	{
		for(unsigned int i=0; i<mat.cols(); i++)
		{
			output<<mat.mat[i][j]<<" ";
		}
		output<<"\n";
	}
	output<<"]\n";
	return output;
}

template <class T>
unsigned int Matrix<T>::cols()
{
	return mat.size();
}

template <class T>
unsigned int Matrix<T>::rows()
{
	if(mat.size()==0) return 0;
	return mat[0].size();
}

template <class T>
int Matrix<T>::definite()
{
	unsigned int positive = 0;
	unsigned int negative = 0;
	unsigned int zeros = 0;
	int det;
	for(unsigned int i = 0; i < cols(); i++)
	{
		Matrix <T> m1(i+1, i+1, 0);
		for(unsigned int j = 0; j < i+1; j++)
		{
			for(unsigned int k = 0; k < i+1; k++)
			{
				m1.set(j, k, mat[j][k]);
			}
		}
		det = m1.Determinant();
		if(det>0) positive++;
		if(det<0) negative++;
		if(det==0) zeros++;
	}
	if(positive == cols()) return 2;
	if(negative == 0 && positive + zeros == cols()) return 1;
	if(negative == cols()) return -2;
	if(positive == 0 && negative + zeros == cols()) return -1;
	if(zeros == cols()) return 0;
	return -3;
}

template <class T>
bool Matrix<T>::isZeroMatrix()
{
	for(unsigned int i = 0; i <  cols(); i++)
	{
		for(unsigned j = 0; j < rows(); j++)
		{
			//if(mat[i][j] != 0) allZeros = false;
			if(std::abs(mat[i][j]) > pow(10,-4.0)) return false;
		}
	}
	return true;
}

#endif
