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
		int count;
	public:
		Landmark (Vector4f nPlane)
		{
			setPlane(nPlane);
			count = 0;
		}
		void increaseCount() {
			count++;
		}
		Vector4f getPlane() {
			return plane;
		}
		void setPlane(Vector4f nPlane) {
			plane = nPlane;
		}
	};

	vector<Landmark> landmarks;

public:
	void addToLandmarks(Vector4f plane) {
		Landmark *new_landmark = new Landmark(plane);
		landmarks.push_back(*new_landmark);
	}
	int size() {
		return landmarks.size();
	}
	Vector4f getPlane(int i) {
		return landmarks[i].getPlane();
	}
	float getMahaDist(Vector4f A, Vector4f B, Matrix4f S) {
			Matrix<float, 1, 1> dist;
			Vector4f diff = A-B;
			return diff.transpose() *  S.inverse()*   diff;
		}
	void increaseCount(int planeId) {
		landmarks[planeId].increaseCount();
	}
	void updatePlane(int planeId, Vector4f delta) {
		Vector4f temp = landmarks[planeId].getPlane() + delta;
			landmarks[planeId].setPlane(temp);
		}
};





#endif /* LANDMARKS_H_ */
