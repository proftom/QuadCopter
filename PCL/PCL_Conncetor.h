#ifndef PCL_CONNECTOR_H_
#define PCL_CONNECTOR_H_

#include "Vision.h"
#include "Plane.h"
#include <vector>
#include <boost/thread/thread.hpp>


//using namespace std;

class dummy 
{
//private:
	//Pointer to the QCVision object
	QCVision* vision;

public:

	dummy(QCVision* x) {
		vision = x;
	}

	void runny() 
	{


		while(1) {
			//while(!vision->bNewSetOfPlanes);
				vision->m_mutexLockPlanes.lock();
			 
				vision->bNewSetOfPlanes = false;
				vision->m_mutexLockPlanes.unlock();
				cout << "PLANES FOUND: " << vision->getPlanes().size() << endl;

		}


			//cout << "Done done" << endl;
	}

};
//
////static class PCL_Connector
////{
////
////// Other non-static member functions
////private:
////	
////	QCVision* vision;
////
////public:
////
////	boost::mutex mut;
////
////	static PCL_Connector& Instance()
////	{
////		static PCL_Connector singleton;
////		return singleton;
////	}
////
////
////	static vector<Plane> readPlaneFromBuffer()
////	{
////		return vision.getPlanes();
////	}
////
////	PCL_Connector() {}                                  // Private constructor
////	~PCL_Connector() {}
////	PCL_Connector(const PCL_Connector&);                 // Prevent copy-construction
////	PCL_Connector& operator=(const PCL_Connector&);      // Prevent assignment
////
////};
//
#endif