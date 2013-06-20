//============================================================================
// Name        : Test.cpp
// Author      : Chinemelu Ezeh
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <Eigen/Dense>
#include <iostream>
#include <math.h>
#include <vector>
#include <time.h>
#include <Eigen/StdVector>
#include <boost/thread/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include "Vision.h"
//#include "SerialClass.h"

using namespace std;
using namespace Eigen;

typedef Matrix< float , 16 , 16> Matrix16f;
typedef Matrix< float , 16 , 1> Vector16f;
#define STATENUM 16
#define Sampling_Time 0.01
#define covFactor 1000
#define distThreshold 800

struct planes_struct {
	Vector4f plane;
	Matrix4f cov;
	bool isMatched;
};

struct imuRawS
{
	int accX;
	int accY;
	int accZ;
	int gyroX;
	int gyroY;
	int gyroZ;

};

// Function definitions.
Matrix3f DCM_fn();
void initialisation ();
Matrix4f omega_fn();
void state_prediction ();
MatrixXf Xi_fn();
MatrixXf noiseMatrix();
Matrix16f F_fn();
MatrixXf G_fn();
void covariance_prediction();
MatrixXf Xip_fn();
MatrixXf H_fn(int planeId);
void update();
void getNewMeasurement();
bool getNewMeasurementThalamus();
void getNewObservation();
void getNewObservationLive(QCVision& vision);
//void run();


//Variables
vector<Vector4f, Eigen::aligned_allocator<Vector4f> > landmarks;
int timeSteps = 0;
vector<Vector16f, Eigen::aligned_allocator<Vector16f> > statehistory;
vector<planes_struct, Eigen::aligned_allocator<planes_struct> > newPlanes;

Vector3f acc;
Vector3f gyro;
Vector16f state;
MatrixXf P(28,28);
MatrixXf bigH;
Matrix4f omega;
Matrix3f DCM;
MatrixXf Xi(4,3);
MatrixXf Xip(4,3);
MatrixXf Q(12,12);

int kalman(QCVision& vision) {
	clock_t tStart = clock();
	vision.DCM = &DCM;
	vision.stateVector = &state;
	
	initialisation ();
	while(true) {

		while(!getNewMeasurementThalamus()){
			//cout <<"wait"<<endl;
			boost::this_thread::sleep(boost::posix_time::millisec(2));
		}

		DCM = DCM_fn();
		omega = omega_fn();
		Xi = Xi_fn();

		state_prediction();
		covariance_prediction();
		vision.m_mutexLockPlanes.lock();
		if (vision.bNewSetOfPlanes){
			getNewObservationLive(vision);
			vision.bNewSetOfPlanes = false;
			vision.m_mutexLockPlanes.unlock();
			update();
			cout << "state at time t = " << timeSteps << endl<< state.segment(0,3) << endl<<endl;

		} else {
		vision.m_mutexLockPlanes.unlock();
		}
		//cout << "state at time t = " << timeSteps << endl<< state << endl<<endl;

		//uncomment for epic memory depletion
		//statehistory.push_back(state);
	}

	//this is useless in continous operation
	printf("Time taken: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
	for (int i = 0; i < (int)statehistory.size(); i++)
		cout << "state at time t = " << i << endl<< statehistory[i] << endl<<endl;
	return 0;
}

void initialisation () { //Incomplete.

	//Initialise landmarks.
	Vector4f planeCloud_t;
	planeCloud_t << -1,0,0,0;
	landmarks.push_back(planeCloud_t);
	planeCloud_t << 0,1,0,0;
	landmarks.push_back(planeCloud_t);
	planeCloud_t << 0,0,1,0;
	landmarks.push_back(planeCloud_t);

	//initialise landmarks and P
	Matrix3f block1 = Matrix3f::Identity();
	Matrix4f block2 = Matrix4f::Identity();
	P << 	block1*0.09, MatrixXf::Zero(3,13 + 12),
		MatrixXf::Zero(3,16+12),
		MatrixXf::Zero(4,6), block2*0.01, MatrixXf::Zero(4,6+12),
		MatrixXf::Zero(3,10), block1*0.0025, MatrixXf::Zero(3,3+12),
		MatrixXf::Zero(3,13), block1,  MatrixXf::Zero(3,12),
		MatrixXf::Zero(12,28);
	//	cout << "P initial" << endl << P << endl << endl;
	Q = noiseMatrix();
	//initialise state vector.
	state <<-2.2, 2.2, 2.2, 0, 0, 0, 0.853553, 0.146447, 0.353553, -0.353553,
		-0.0456 ,   0.0069,   -0.0048 ,  -0.0331  ,  0.1024 ,   0.1473;
	//	cout << "state initial" << endl << state << endl << endl;
}
Matrix3f DCM_fn() { //There is a round off error in dcm(2,3). May cause an issue.
	Vector4f q;
	q << state.segment(6,4);
	Matrix3f dcm;
	dcm << 	2*(pow(q(0),2) + pow(q(1),2)) - 1,		2*(q(1)*q(2) + q(0)*q(3)), 		2*(q(1)*q(3) - q(0)*q(2)),
		2*(q(1)*q(2) - q(0)*q(3)), 				2*(pow(q(0),2) + pow(q(2),2)) - 1, 2*(q(2)*q(3) + q(0)*q(1)),
		2*(q(1)*q(3) + q(0)*q(2)), 2*(q(2)*q(3) - q(0)*q(1)), 2*(pow(q(0),2) + pow(q(3),2)) - 1;
	//	cout << "dcm" << endl << dcm << endl << endl;
	return dcm;
}
Matrix4f omega_fn() {
	Vector3f w(gyro);

	Matrix4f omega_t;
	omega_t << 0,		-w(0),	-w(1),	-w(2),
		w(0),	0,		w(2),	-w(1),
		w(1), 	-w(2), 	0, 		w(0),
		w(2),	w(1), 	-w(0), 	0;
	//	cout << "omega_t" << endl << omega_t << endl << endl;
	return omega_t;
}
// Calculates Predicted Next State.
void state_prediction () {

	/*
	* state-> 	0:2 position
	* 			3:5 velocity
	* 			6:9 quaternions
	* 			10:12 gyro bias
	* 			13:15 acc bias
	*/
	//Get Xdelta.

	Vector16f xdelta; //dynamic allocation maybe.
	//Position
	xdelta(0) = state(3);
	xdelta(1) = state(4);
	xdelta(2) = state(5);

	//Velocity
	//initialize the DCM using quaternions from state.
	xdelta.segment(3,3) << DCM.transpose() *  acc;
	xdelta(5) += 9.816;

	//Quaternions
	/*
	* To compute quaternion transition, first compute angular velocity vector
	*/

	xdelta.segment(6,4) << 0.5 * Xi * gyro;
	xdelta.segment(10,6) << 0,0,0,0,0,0;
	//Biases
	//xdelta.block<6,1>(10,1) = 0;
	//	cout << "xdelta" << endl << xdelta << endl<<endl;
	state += Sampling_Time*xdelta;
	//normalise quaternions.
	state.segment(6,4)/=state.segment(6,4).norm();
	//	cout << "state predicted" << endl << state << endl<<endl;
}
MatrixXf Xi_fn() {
	MatrixXf Xi_t(4,3);
	Vector4f q(state.segment(6,4));
	Xi_t << -q(1), -q(2), -q(3),
		q(0), -q(3), q(2),
		q(3), q(0), -q(1),
		-q(2), q(1), q(0);
	//	cout << "Xi_t" << endl << Xi_t << endl << endl;
	return Xi_t;
}
MatrixXf noiseMatrix() {
	Matrix<float, 12, 12> Q_t;
	Matrix3f zeros3(Matrix<float, 3, 3>::Zero());
	Matrix3f Qgyro;
	Matrix3f Qacc;
	Matrix3f Qgyrobias;
	Matrix3f Qaccbias;

	Qgyro << 	0.1679,   -0.0615,   -0.1051,
		-0.0615,    0.1320,   -0.0654,
		-0.1051,   -0.0654,    0.2536;
	Qgyro *= 1.0e-03;


	Qacc << 	0.6670,    0.0377,   -0.0648,
		0.0377,    0.8222,   -0.1422,
		-0.0648,   -0.1422,    2.7314;
	Qacc *= 1.0e-03;


	Qgyrobias << 	0.3552,   -0.0323,   -0.2509,
		-0.0323,    0.2822,   -0.2381,
		-0.2509,   -0.2381,    0.5686;
	Qgyrobias *= 1.0e-03;


	Qaccbias << 	0.3501,   -0.1312,    0.2149,
		-0.1312,    0.4719,   -0.2273,
		0.2149,   -0.2273,    1.3670;
	Qaccbias *= 1.0e-02;

	Q_t << 	Qgyro, zeros3, zeros3, zeros3,
		zeros3, Qacc, zeros3, zeros3,
		zeros3, zeros3, Qgyrobias, zeros3,
		zeros3, zeros3, zeros3, Qaccbias;
	//	cout << "Q_t" << endl << Q_t << endl << endl;
	return Q_t;
}
Matrix16f F_fn() {
	// Porting the equations direct from Matlab...
	Vector4f Fvqparts(Xi * acc);

	MatrixXf Fvq(3,4);
	Fvq <<	Fvqparts(1), -Fvqparts(0), Fvqparts(3), -Fvqparts(2),
		Fvqparts(2), -Fvqparts(3), -Fvqparts(0), Fvqparts(1),
		Fvqparts(3), Fvqparts(2), -Fvqparts(1), -Fvqparts(0);
	Fvq*=2;
	/*********************/
	Matrix3f Fvba(-DCM.transpose());
	Matrix4f Fqq(0.5 * omega);
	MatrixXf Fqbw(-0.5 * Xi);
	/********/
	//Build F matrix
	Matrix16f F;
	Matrix3f eye3(Matrix3f::Identity());
	Matrix3f zeros3(Matrix3f::Zero());

	F << 	zeros3, eye3, Matrix<float, 3, 10>::Zero(),
		zeros3, zeros3, Fvq, zeros3, Fvba,
		Matrix<float, 4, 6>::Zero(), Fqq, Fqbw, Matrix<float, 4, 3>::Zero(),
		Matrix<float, 6, 16>::Zero();
	F *= Sampling_Time;
	F+= Matrix<float, 16, 16>::Identity();
	//	cout << "Xi" << endl << Xi << endl << endl;
	//	cout << "Fvq" << endl << Fvq << endl << endl;
	//	cout << "Fqbw" << endl << Fqbw << endl << endl;
	//	cout << "F" << endl << F << endl << endl;
	return F;
}
MatrixXf G_fn() {
	MatrixXf Gqww(0.5 * Xi_fn());
	Matrix3f Gvwa(DCM.transpose());
	Matrix <float, 16, 12>G;
	G << Matrix<float, 3, 12>::Zero(),
		Matrix<float, 3, 3>::Zero(), Gvwa, Matrix<float, 3, 6>::Zero(),
		Gqww, Matrix<float, 4, 9>::Zero(),
		Matrix<float, 3, 6>::Zero(), Matrix<float, 3, 3>::Identity(), Matrix<float, 3, 3>::Zero(),
		Matrix<float, 3, 9>::Zero(), Matrix<float, 3, 3>::Identity();
	G *= Sampling_Time;
	//	cout << "G" << endl << G << endl << endl;
	return G;
}
void covariance_prediction() { //Create a function to build P up in step3.
	Matrix16f P_old(P.block<16,16>(0,0));
	MatrixXf G(G_fn());
	MatrixXf F(F_fn());
	P.block<16,16>(0,0) << F*P.block<16,16>(0,0)*F.transpose() + G*Q*G.transpose();
	// Should we propagate any change to the other blocks of P?
	MatrixXf P_temp(F*P.block(0,16, 16, P.cols() - 16));
	P.block(0,16, 16, P.cols() - 16) = P_temp;
	P.block(16,0, P.rows() - 16,16)  = P_temp.transpose();

	//	cout << "P" << endl << P << endl << endl;
}
MatrixXf Xip_fn() {
	MatrixXf Xip_t(4,3);
	Vector4f q(state.segment(6,4));
	//	cout << "q" << endl << q << endl << endl;
	Xip_t << 	-q(1), -q(2), -q(3),
		-q(0), -q(3),  q(2),
		q(3),  -q(0), -q(1),
		-q(2),  q(1),  -q(0);
	//	cout << "Xip" << endl << Xip_t << endl << endl;
	return Xip_t;
}
MatrixXf H_fn(int planeId) {
	MatrixXf H(4, 20);
	// Build H for each plane.


	Vector4f plane(landmarks[planeId]);
	Vector4f Hqparts;
	Hqparts << Xip * plane.segment(0,3); //Nx,Ny Nz
	//	cout << "Hqparts" << endl << Hqparts << endl << endl;
	//	cout << "currtP" << endl << plane << endl << endl;
	MatrixXf Hq(3,4);
	Hq << -Hqparts(1), -Hqparts(0), Hqparts(3), -Hqparts(2),
		-Hqparts(2), -Hqparts(3), -Hqparts(0), Hqparts(1),
		-Hqparts(3), Hqparts(2), -Hqparts(1), -Hqparts(0);
	Hq*=2;
	//	cout << "Hq" << endl << Hq << endl << endl;
	MatrixXf block(4,3);
	Matrix4f block2;
	Matrix4f block3(4,4);
	block << MatrixXf::Zero(3,3), plane.segment(0,3).transpose();
	block2 << Hq, Matrix<float, 1, 4>::Zero();

	block3 << DCM,  Matrix<float, 3, 1>::Zero(), state.segment(0,3).transpose(), 1;

	H << 	block,  	MatrixXf::Zero(4,3), 	block2,		MatrixXf::Zero(4,6),
		block3;
	//	cout << "H" << endl << H << endl << endl;
	return H;
}
void update() {
	//Data Association.

	for (int i = 0; i < (signed)newPlanes.size(); i++) {
		DCM = DCM_fn();
		Xip = Xip_fn();
		omega = omega_fn();
		Matrix4f hypermute;
		hypermute << 	0, 0, 1,0,
			1, 0,0,0,
			0,1,0,0,
			0,0,0,1;
		Matrix4f transPlane;
		transPlane << DCM, Vector3f::Zero(), state.segment(0,3).transpose(), 1;
		//		cout << "Transplane" << endl << transPlane << endl << endl;
		Vector4f inP;
		Matrix4f inC;
		inP << hypermute*newPlanes[i].plane;
		inC << hypermute*newPlanes[i].cov* hypermute.transpose();
		MatrixXf minH(4,28);
		Matrix4f minS;
		Vector4f minDiff;
		MatrixXf P_min_opt;
		float tempDist;
		int first = 1;
		int ptr = 0;
		for (int j = 0; j < (int)landmarks.size(); j++) {
			Vector4f diff;
			diff << inP - transPlane*landmarks[j];
			//			cout << "inP" << endl << inP << endl << endl;
			//			cout << "transPlane" << endl << transPlane << endl << endl;
			//			cout << "landmarks.[j]" << endl << landmarks[j] << endl << endl;
			//			cout << "Diff" << endl << diff << endl << endl;
			MatrixXf H(H_fn(j));
			//Build the optimised P.
			MatrixXf P_opt(20,20);
			P_opt << P.block(0,0, 16,16), P.block(0,16+4*j, 16,4),
				P.block(16+4*j,0,4,16), P.block(16+4*j, 16+4*j, 4, 4);
			//			cout << "P_opt" << endl << P_opt << endl << endl;
			MatrixXf S(H*P_opt*H.transpose() + inC * 100);

			float dist = diff.transpose() *  S.inverse()*   diff;
			dist = abs(dist);
			//			cout << "Distance" << endl << dist << endl << endl;
			if ((first == 1) || dist < tempDist) {
				first = 0;
				tempDist = dist;
				minH = H;
				minS = S;
				minDiff = diff;
				P_min_opt = P_opt;
				ptr = j;
			}
		}
		//Update States
		MatrixXf kalmanGain(P_min_opt * minH.transpose() * minS.inverse());
		//		cout << "minH" << endl << minH << endl << endl;
		//		cout << "P_opt" << endl << P_min_opt << endl << endl;
		//		cout << "minS inverse" << endl << minS.inverse() << endl << endl;
		//		cout << "minS" << endl << minS << endl << endl;
		//		cout << "minDiff" << endl << minDiff << endl << endl;
		//		cout << "kalmanGain" << endl << kalmanGain << endl << endl;
		VectorXf change(kalmanGain * minDiff);
		//		cout<< "state increment" << endl << change.segment(0,16)<<endl<<endl;
		state += change.segment(0,16);
		//normalise quaternions.
		state.segment(6,4)/=state.segment(6,4).norm();
		//		cout<< "state update stage" << endl << state<<endl<<endl;
		//		cout<< "Ptr" << endl << ptr<<endl<<endl;
		//Update State Covariances
		P_min_opt -= kalmanGain*minH*P_min_opt;
		//unPack P_opt into main P
		P.block(0,0, 16,16) = P_min_opt.block(0,0, 16,16);
		P.block(0,16+4*ptr, 16,4) = P_min_opt.block(0,16, 16,4);
		P.block(16+4*ptr, 0, 4, 16) = P_min_opt.block(16,0, 4,16);
		P.block(16+4*ptr,16+4*ptr,4,4) = P_min_opt.block(16,16,4,4);
		//		cout<< "P update stage" << endl << P<<endl<<endl;
		//update landmarks covariances and state.
		Vector4f delta(change.segment(16,4));
		landmarks[ptr] += delta;

		// addition to state.

	}
}

bool getNewMeasurementThalamus(){
	//static Serial SP("\\\\.\\COM22"); 
	//int SPba = SP.BytesAvailable();
int SPba = 2;
	if (SPba >= 10)
	{
		//if(SPba > 100)
			//cout << "bytebacklog is " << SPba << endl;

		char sync[2];
		//SP.ReadData(sync,sizeof(sync));

		if (sync[0] == 11)
		{
			short GyroRaw[3];
			//SP.ReadData((char*)GyroRaw,sizeof(GyroRaw));

			Vector3f gyro_t;
			gyro_t << GyroRaw[0], GyroRaw[1], GyroRaw[2];
			gyro_t /= 818.51113590117601252569;
			gyro << gyro_t(2), -gyro_t(0), -gyro_t(1);
			Vector3f gyroBias(state.segment(10,3));
			//	cout << "gyro measured" << endl << gyro << endl << endl;
			gyro  -= gyroBias;


			short AccRaw[3];
			//SP.ReadData((char*)AccRaw,sizeof(AccRaw));

			Vector3f acc_t;
			acc_t << AccRaw[0], AccRaw[1], AccRaw[2];
			acc_t*=(9.816 /pow(2.00,14));
			acc << acc_t(2), -acc_t(0), -acc_t(1);
			Vector3f accBias(state.segment(13,3));
			//	cout << "acc measured" << endl << acc << endl << endl;
			acc-=accBias;

			short MagRaw[3];
			//SP.ReadData((char*)MagRaw,sizeof(MagRaw));

			return true;
		}
	}

	return false;
}

/*
void getNewMeasurement() {
//Process Accelerometer reading.
Vector3f acc_t;
acc_t << accList[accPtr], accList[accPtr+1],accList[accPtr+2];
acc_t*=(9.816 /pow(2.00,14));
acc << acc_t(2), -acc_t(0), -acc_t(1);
Vector3f accBias(state.segment(13,3));
//	cout << "acc measured" << endl << acc << endl << endl;
acc-=accBias;

// Process Gyro Reading.
Vector3f gyro_t;
gyro_t << gyroList[gyroPtr], gyroList[gyroPtr+1],gyroList[gyroPtr+2];
gyro_t /= 818.51113590117601252569;
gyro << gyro_t(2), -gyro_t(0), -gyro_t(1);
Vector3f gyroBias(state.segment(10,3));
//	cout << "gyro measured" << endl << gyro << endl << endl;
gyro  -= gyroBias;

accPtr+=3;
gyroPtr+=3;
if(accPtr >= Steps || gyroPtr >= Steps) {
accPtr  = 0;
gyroPtr = 0;
}
//	cout << "acc" << endl << acc << endl << endl;
//	cout << "gyro" << endl << gyro << endl << endl;
}
*/


void getNewObservationLive(QCVision& vision){
	newPlanes.clear();
	vector<Plane> pv = vision.getPlanes();

	for (int i = 0; i < pv.size(); ++i)
	{
		Plane& currplane = pv[i];
		vector<vector<float> >& inC = currplane.covariance;
		Vector4f planeCloud_t;
		Matrix4f cov_t;


		planeCloud_t << currplane.A, currplane.B, currplane.C, currplane.D;
		cov_t << inC[0][0], inC[0][1], inC[0][2], inC[0][3],
				 inC[1][0], inC[1][1], inC[1][2], inC[1][3],
				 inC[2][0], inC[2][1], inC[2][2], inC[2][3],
				 inC[3][0], inC[3][1], inC[3][2], inC[3][3];

		planes_struct temp;
		temp.cov = cov_t;
		temp.plane = planeCloud_t;
		newPlanes.push_back(temp);
	}
}

/*
void getNewObservation() {
	newPlanes.clear();
	int len = planeList[planePtr++];
	for (int i = 0; i<len; i++) {
		Vector4f planeCloud_t;
		Matrix4f cov_t;
		planeCloud_t << planeList[planePtr], planeList[planePtr+1], planeList[planePtr+2],planeList[planePtr+3];
		cov_t << planeList[planePtr+4], planeList[planePtr+5], planeList[planePtr+6],planeList[planePtr+7],
			planeList[planePtr+8], planeList[planePtr+9], planeList[planePtr+10],planeList[planePtr+11],
			planeList[planePtr+12], planeList[planePtr+13], planeList[planePtr+14],planeList[planePtr+15],
			planeList[planePtr+16], planeList[planePtr+17], planeList[planePtr+18],planeList[planePtr+19];
		planes_struct temp;
		temp.cov = cov_t;
		temp.plane = planeCloud_t;
		newPlanes.push_back(temp);
		planePtr+=20;
	}
	planeSteps++;
	if (planeSteps >= Steps) {
		planePtr = 0;
		planeSteps = 0;
	}
	//newPlanes
	for (int i = 0; i < (int)newPlanes.size(); i++) {
		//		cout << "newPlane " << i<< endl << newPlanes[i].plane << endl << endl;
		//		cout << "new cov" << i << endl << newPlanes[i].cov << endl << endl;
	}
}
*/

