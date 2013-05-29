/*
 * landmarks.h
 *
 *  Created on: 28 May 2013
 *      Author: ChinemeluEzeh
 */

#ifndef LANDMARKS_H_
#define LANDMARKS_H_

/*
 * Landmark.h
 *
 *  Created on: 27 May 2013
 *      Author: ChinemeluEzeh
 */
#include <Eigen/Dense>
#include <iostream>
#include <math.h>
#include <vector>


using namespace std;
using namespace Eigen;


class Landmarks {
private:
	class Landmark {
	private:
		Vector4f plane;
		Matrix4f cov;
		int count;
	public:
		Landmark (Vector4f nPlane, Matrix4f nCov)
		{
			setPlane(nPlane);
			setCov(nCov);
			count = 0;
		}
		void increaseCount() {
			count++;
		}
		Vector4f getPlane() {
			return plane;
		}
		Matrix4f getCov() {
			return cov;
		}
		void setPlane(Vector4f nPlane) {
			plane = nPlane;
		}
		void setCov(Matrix4f nCov) {
			cov = nCov;
		}
	};

	vector<Landmark> landmarks;

public:
	void addToLandmarks(Vector4f plane, MatrixXf cov) {
		Landmark *new_landmark = new Landmark(plane, cov);
		landmarks.push_back(*new_landmark);
	}
	int size() {
		return landmarks.size();
	}
	Vector4f getPlane(int i) {
		return landmarks[i].getPlane();
	}
	Matrix4f getCov(int i) {
		return landmarks[i].getCov();
	}
	float getMahaDist(Vector4f A, Vector4f B, Matrix4f S) {
		Matrix4f dist;
		Vector4f diff = A-B;
		dist<< diff.transpose() *  S.inverse()*   diff;

		return dist(0,0);
	}
	void increaseCount(int planeId) {
		landmarks[planeId].increaseCount();
	}
	void updatePlane(int planeId, Vector4f delta) {
		Vector4f temp = landmarks[planeId].getPlane() + delta;
			landmarks[planeId].setPlane(temp);
		}
	void updateCov(int planeId, Matrix4f delta) {
		landmarks[planeId].setCov(landmarks[planeId].getCov() - delta);
	}
};





#endif /* LANDMARKS_H_ */
