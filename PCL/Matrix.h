#include <iostream>
#include <Eigen/Dense>
using namespace Eigen;
using namespace std;

class SparseMatrix {
private:
	MatrixXd H;
	int currentPlane;
public:

	SparseMatrix(int nPlanes) {
		H.resize(4, 16 + nPlanes * 4);
		H.fill(0);
	}


	SparseMatrix(MatrixXd left, int nPlanes) {
		H = left;
		H.resize(left.rows(), left.cols() + nPlanes * 4);
	}  

	SparseMatrix(MatrixXd left, int nPlanes, Matrix4d &plane, int position) {
		SparseMatrix(left, nPlanes);
		currentPlane = position;
		H.block(0, 16 + currentPlane * 4, 4, 4) << plane;
		
	}

	void ChangePlane(Matrix4Xd newPlane, int position) {
		
		//Initialise previous plane values back to 0
		H.block(0, 16 + currentPlane * 4, 4, 4).fill(0); //<< 1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16;
		//Input new plane
		H.block(0, 16 + position * 4, 4, 4) << newPlane;
		cout << endl << "H=" << endl << H;

	}

};

