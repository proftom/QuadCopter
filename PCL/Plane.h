#ifndef PCL_planeCloud_H_
#define PCL_planeCloud_H_

#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace std;

class Plane  {
	
public:

	float A, B, C, D;
	vector<int> indicies;
	vector< vector<float> > covariance;

	Plane(float a, float b, float c, float d) : A(a), B(b), C(c), D(d){}
	
	Plane(float x, float y, float z, pcl::Normal &norm) 
	{
		Plane(norm.normal_x, norm.normal_y, norm.normal_z, -(norm.normal_x * x + norm.normal_y * y + norm.normal_z * z));
	}

	Plane(float x, float y, float z, float a, float b, float c) {
		A = a;
		B = b;
		C = c;
		Plane(a, b, c, -(a * x + b * y + c * z));
	}

	Plane(float a, float b, float c, float d, vector<int> ind)  {
		A = a;
		B = b;
		C = c;
		D = d;
		indicies = ind;
	}

	const Plane operator-(const Plane &plane) const {

		return Plane(this->A - plane.A, this->B - plane.B, this->C - plane.C, this->D - plane.D);

	}

	void calculateCovarianceMatrix(vector<Plane> &planes) {
		vector<vector<float> > covarianceMatrix(4);
		covarianceMatrix[0].assign(4,0); covarianceMatrix[1].assign(4,0); covarianceMatrix[2].assign(4,0); covarianceMatrix[3].assign(4,0); 
		
		float 
			xbar  = 0.0, 
			ybar  = 0.0, 
			zbar  = 0.0, 
			dbar  = 0.0,
			xxbar = 0.0,
			xybar = 0.0, 
			xzbar = 0.0, 
			xdbar = 0.0,
			yybar = 0.0,
			yzbar = 0.0, 
			ydbar = 0.0,
			zzbar = 0.0,
			zdbar = 0.0,
			ddbar = 0.0;

		
		for (int i = 0; i < indicies.size(); i++) {

			xbar += planes[indicies[i]].A;
			ybar += planes[indicies[i]].B;
			zbar += planes[indicies[i]].C;
			dbar += planes[indicies[i]].D;
			xxbar += planes[indicies[i]].A * planes[indicies[i]].A;
			xybar += planes[indicies[i]].A * planes[indicies[i]].B;
			xzbar += planes[indicies[i]].A * planes[indicies[i]].C;
			xdbar += planes[indicies[i]].A * planes[indicies[i]].D;
			yybar += planes[indicies[i]].B * planes[indicies[i]].B;
			yzbar += planes[indicies[i]].B * planes[indicies[i]].C;
			ydbar += planes[indicies[i]].B * planes[indicies[i]].D;
			zzbar += planes[indicies[i]].C * planes[indicies[i]].C;
			zdbar += planes[indicies[i]].C * planes[indicies[i]].D;
			ddbar += planes[indicies[i]].D * planes[indicies[i]].D;

		}

		xbar /= indicies.size(); ybar /= indicies.size(); zbar /= indicies.size(); dbar /= indicies.size(); xxbar /= indicies.size(); xybar /= indicies.size();
		xzbar /= indicies.size(); xdbar /= indicies.size(); yybar /= indicies.size(); yzbar /= indicies.size(); ydbar /= indicies.size();
		zzbar /= indicies.size(); zdbar /= indicies.size(); ddbar /= indicies.size();

		covarianceMatrix[0][0] = (xxbar - xbar*xbar) / indicies.size();
		covarianceMatrix[0][1] = (xybar - xbar*ybar) / indicies.size();
		covarianceMatrix[0][2] = (xzbar - xbar*zbar) / indicies.size(); 
		covarianceMatrix[0][3] = (xdbar - xbar*dbar) / indicies.size();
		covarianceMatrix[1][1] = (yybar - ybar*ybar) / indicies.size(); 
		covarianceMatrix[1][2] = (yzbar - ybar*zbar) / indicies.size(); 
		covarianceMatrix[1][3] = (ydbar - ybar*dbar) / indicies.size();
		covarianceMatrix[2][2] = (zzbar - zbar*zbar) / indicies.size();
		covarianceMatrix[2][3] = (zdbar - zbar*dbar) / indicies.size();
		covarianceMatrix[3][3] = (ddbar - dbar*dbar) / indicies.size();

		//Lower triangle
		covarianceMatrix[1][0] = covarianceMatrix[0][1];
		covarianceMatrix[2][0] = covarianceMatrix[0][2];
		covarianceMatrix[2][1] = covarianceMatrix[1][2];
		covarianceMatrix[3][0] = covarianceMatrix[0][3];
		covarianceMatrix[3][1] = covarianceMatrix[1][3];
		covarianceMatrix[3][2] = covarianceMatrix[2][3];

		covariance = covarianceMatrix;

	}

};

#endif
