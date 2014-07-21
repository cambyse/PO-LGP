#include <Core/array.h>

void TEST(CheatSheet) {
  using namespace std;
  // CHEAT SHEET for MT::Array
  
  cout << "##### CREATING MATRICES" << endl;
  // Create MT::Array<double> with the 'arr' macro
  arr A;
  // or MT::Array<int> with 'intA'
  intA B; 
  // or MT::Array<uint> with 'intA'
  uintA C; 

  // Use matlab-like function to create matrices
  cout << "\n##### MATLAB FUNCTIONS" << endl;
  A = randn(2);
  A = zeros(3, 5);
  A = ones(3,3);
  arr I = eye(4);
  // or fill the matrix manually with an initialization list
  A = {1, 3, 7, 8};

  cout << "\n##### MISC" << endl;
  // you can reshape matrices
  A.reshape(2, 2);
  // print matrices
  std::cout << A << std::endl;

  // query information about the array dimentions
  std::cout << "number of elements total:    " << A.N << std::endl;
  std::cout << "number of elements in dim 0: " << A.d0 << std::endl;
  std::cout << "number of elements in dim 1: " << A.d1 << std::endl;

  cout << "\n##### ACCESS" << endl;
  // access single elements
  std::cout << A(0, 0) << std::endl;
  // overwriting single elements
  A(0, 0) += 1;
  std::cout << "A(0, 0) = " << A(0, 0) << std::endl;
  // access rows
  cout <<"I.row(0): " <<I.row(0) <<endl;
  cout <<"I.rows(0, 2):\n" <<I.rows(0, 2) <<endl;
  // access columns
  cout <<"I.col(3): " <<I.col(3) <<endl;
  cout <<"I.rows(1, 4):\n" <<I.rows(1, 4) <<endl;
  // iterate through all elements
  for (const auto& elem : A) std::cout << elem << std::endl;
  for (auto& elem : A) elem += 4.;
  cout <<"A+4: " <<A <<endl;

  cout << "\n##### MATRIX OPERATIONS" << std::endl;
  // Work as you'd expect
  A = randn(2, 2);
  I = eye(2);
  CHECK(A * I == A, "");
  CHECK(A + A == A * 2., "");
  // transpose
  CHECK(A == ~(~A), "A = A^T^T");
  // inplace transpose
  cout <<"before inplace transpose:\n" <<A <<endl;
  transpose(A);
  cout <<"after inplace transpose:\n" <<A <<endl;
  transpose(A);
  cout <<"back to normal/after second inplace transpose:\n" <<A <<endl;
  // inverse
  cout <<"\nA * A.I:\n" <<A * inverse(A) <<endl;
  cout <<"\nA.I * A:\n" <<inverse(A) * A <<endl;
  CHECK_ZERO(maxDiff(A * inverse(A),I),1e-6, "");
}
