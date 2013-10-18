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
  A = ones(3);
  arr I = eye(4);
  // or fill the matrix manually with an initialization list
  A = {1, 3, 7, 8};

  cout << "\n##### MISC" << endl;
  // you can reshape matrices
  A.reshape(2, 2);
  // print matrices different ways
  std::cout << A << std::endl;
  A.print();
  A.print("A:");

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
  I.row(0).print("I.row(0):");
  I.rows(0, 2).print("I.rows(0, 2):");
  // access columns
  I.col(3).print("I.col(3):");
  I.rows(1, 4).print("I.rows(1, 4):");
  // iterate through all elements
  for (const auto& elem : A) std::cout << elem << std::endl;
  for (auto& elem : A) elem += 4.;
  A.print("A+4:");

  cout << "\n##### MATRIX OPERATIONS" << std::endl;
  // Work as you'd expect
  A = randn(2, 2);
  I = eye(2);
  CHECK(A * I == A, "");
  CHECK(A + A == A * 2., "");
  // transpose
  CHECK(A == ~(~A), "A = A^T^T");
  // inplace transpose
  A.print("before inplace transpose");
  transpose(A);
  A.print("after inplace transpose");
  transpose(A);
  A.print("back to normal/after second inplace transpose");
  // inverse
  (A * inverse(A)).print("\nA * A.I");
  (inverse(A) * A).print("\nA.I * A");
  CHECK(A * inverse(A) == I, "");
}
