#ifndef _ONI
#define _ONI
#include <boost/thread/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/openni_camera/openni_driver.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/console/parse.h>
#include <pcl/common/time.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/kdtree/kdtree.h>
#include <iostream>
#include <fstream>
#include "Plane.h"
#include <time.h>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <pcl/common/transforms.h>

//#include "Depth_Correction_Array.txt"
extern float correction_table[19200][56];
using namespace std;
using namespace Eigen;
typedef Matrix< float , 16 , 16> Matrix16f;
typedef Matrix< float , 16 , 1> Vector16f;

#include <queue>

#define RESOLUTION_MODE pcl::OpenNIGrabber::OpenNI_QQVGA_30Hz

#define FPS_CALC(_WHAT_) \
do \
{ \
    static unsigned count = 0;\
    static double last = pcl::getTime ();\
    double now = pcl::getTime (); \
    ++count; \
    if (now - last >= 1.0) \
    { \
      std::cout << "Average framerate("<< _WHAT_ << "): " << double(count)/double(now - last) << " Hz" <<   std::endl; \
      count = 0; \
      last = now; \
    } \
}while(false)

extern float G_smoothsize;
extern float G_depthdepend;

typedef pcl::PointXYZRGBA PointType;

class QCVision
{

  private:
	clock_t t1, t2;
    bool new_cloud_;

	vector<Plane> planes;

  public:
    typedef pcl::PointCloud<PointType> Cloud;
	typedef Cloud::Ptr CloudPtr; //typename
    typedef Cloud::ConstPtr CloudConstPtr; //typename

	unsigned int eventflag; 
	unsigned int Perseventflag; 
	
   // pcl::visualization::CloudViewer viewer;
    std::string device_id_;
    boost::mutex mtx_;
	boost::mutex m_mutexLockPlanes;
	vector<Plane> planeBuffer;

	Vector16f* stateVector;

	Matrix3f* DCM; 


    // Data
    CloudConstPtr cloud_;						//Raw cloud
	pcl::PointCloud<pcl::Normal>::Ptr normals_; //Normal cloud
	pcl::IntegralImageNormalEstimation<PointType, pcl::Normal> ne_; //Normal estimation object
	vector<Plane> planes_;						//Found planes
	bool bNewSetOfPlanes;						//New planes computed
	//PCL_Connector con;						//
	

	QCVision (const std::string& device_id = "") :  device_id_(device_id), eventflag(1)
    {

      ne_.setNormalEstimationMethod (pcl::IntegralImageNormalEstimation<PointType, pcl::Normal>::SIMPLE_3D_GRADIENT);
	  ne_.setDepthDependentSmoothing(true);
	  ne_.setMaxDepthChangeFactor (G_depthdepend);
      ne_.setNormalSmoothingSize (G_smoothsize);
      new_cloud_ = false;
      //viewer.registerKeyboardCallback(&QCVision::keyboard_callback, *this);
	  //Plane cloud
	  planes_.assign(19200, Plane(0,0,0,0));

    }

	vector<Plane> getPlanes()
	{
		return planeBuffer;
	}

	
	void cloud_cb (const CloudConstPtr& cloud)
    {	  boost::mutex::scoped_lock lock (mtx_);
 
      //lock while we set our cloud;
      FPS_CALC ("computation");

      normals_.reset (new pcl::PointCloud<pcl::Normal>);

      ne_.setInputCloud (cloud);
	  ne_.compute (*normals_); 


 
      cloud_ = cloud;

	  static bool hasrun = false;
	  if(eventflag & 0x1){
		  if(!hasrun){


				pcl::PointCloud<PointType> pc(*cloud);
				correctDistances(cloud, &pc);

				CloudConstPtr correctedCloud(new pcl::PointCloud<PointType>(pc));
				cloud_ = correctedCloud;				
				
				//Now flood fill 
				vector<vector<int> > clusterIndicies = floodFillAll(1000, 20);

				planes = ClusterToAveragePlane(clusterIndicies);

				for (int i = 0; i < planes.size(); i++) {
					planes[i].calculateCovarianceMatrix(planes_);
			
					
				}


				m_mutexLockPlanes.lock();

				planeBuffer =  planes;
				bNewSetOfPlanes = true;
				//cout << "Unlocked planes in Vision class";
				m_mutexLockPlanes.unlock();
			

				new_cloud_ = true;
		  }
	  }else{
		  cloud_ = cloud;
		  if (!(eventflag & 0x1)){
			  new_cloud_ = true;
			  hasrun = false;
		  }
	  }

	  if(Perseventflag & 0x1){
			MatrixXf Tran(4,4);
			Tran.block<3,3>(0,0)= DCM->transpose();
			Tran.block<3,1>(0,3)= stateVector->segment<3>(0);
			Tran.block<1,1>(3,3) << 1;
			cout << Tran<< endl;
			pcl::PointCloud<PointType> pc(*cloud_);
			transformPointCloud(pc,pc,Tran);
			CloudConstPtr inputtmp(new pcl::PointCloud<PointType>(pc));
			cloud_ = inputtmp;
		}
		else{
		}
    }




    void run ()
    {
      pcl::Grabber* interface = new pcl::OpenNIGrabber (device_id_, RESOLUTION_MODE, RESOLUTION_MODE);

      boost::function<void (const CloudConstPtr&)> f = boost::bind (&QCVision::cloud_cb, this, _1);
      boost::signals2::connection c = interface->registerCallback (f);

      //viewer.runOnVisualizationThread (boost::bind(&QCVision::viz_cb, this, _1), "viz_cb");

      interface->start ();

      while (1)
      {
        boost::this_thread::sleep(boost::posix_time::seconds(1));
      }

      interface->stop ();
    }
	
	vector<Plane> ClusterToAveragePlane(vector<vector<int> > clusters) 
	{
		vector<Plane> planes;

		for(int i = 0; i < clusters.size(); i++) {
						
			float 
				accum_x = 0.0,
				accum_y = 0.0,
				accum_z = 0.0,
				accum_d = 0.0;

			for (int j = 0; j < clusters[i].size(); j++) {

				accum_x += normals_->points[clusters[i][j]].normal_x;
				accum_y += normals_->points[clusters[i][j]].normal_y;
				accum_z += normals_->points[clusters[i][j]].normal_z;
				accum_d -= ((cloud_->points[clusters[i][j]].x * normals_->points[clusters[i][j]].normal_x)
							+ (cloud_->points[clusters[i][j]].y * normals_->points[clusters[i][j]].normal_y)
							+ (cloud_->points[clusters[i][j]].z * normals_->points[clusters[i][j]].normal_z));
			}

			planes.push_back(Plane(accum_x/clusters[i].size(), accum_y/clusters[i].size(), accum_z/clusters[i].size(), accum_d/clusters[i].size(),clusters[i]));
		}

		return planes;
	}

	
	
	//Flood fill
	vector<vector<int> > floodFillAll(int minPts, int maxTries) 
	{
		vector<vector<int> > clusters;
		
		//Calculate planes for each point with normal. Required for checking if 2 points lie within the same plane
		calculatePlanes_();

		int dataSize = normals_->size();
		vector<bool> isInCluster(dataSize, false);

		int count = 0;
		//Should check if certain points checked
		for (int tries = 0; tries < maxTries; ++tries)
		{
			int seed = rand()%dataSize;
			if (!isInCluster[seed]) 
			{
				vector<int> x = floodFillNaive(seed, isInCluster, count);
				if (x.size() > minPts){
					clusters.push_back(x);
					for (int i = 0; i < x.size(); i++){
						isInCluster[x[i]] = true;
					}
				}
			}
		}

		cout << clusters.size() << " found.";
		
		return clusters;
	}

	vector<int> floodFillNaive (int rootNode, vector<bool> isInClusterOrQ, int& count) 
	{
		const int row = 160;
		vector<int> clusterPoints;
		
		std::vector<int> Q;
		Q.push_back(rootNode);

		while(!Q.empty()) {

			int currentPoint = Q.back();
			Q.pop_back();
			if (samePlaneNormal(currentPoint, rootNode, 0.2)) {
				//Add current point to cluster
				clusterPoints.push_back(currentPoint);
				count++;

				if (!isInClusterOrQ[currentPoint-1]){
					Q.push_back(currentPoint-1);	//west
					isInClusterOrQ[currentPoint-1] = true;
				}
				if (!isInClusterOrQ[currentPoint+1]){
					Q.push_back(currentPoint+1);	//east
					isInClusterOrQ[currentPoint+1] = true;
				}
				if (!isInClusterOrQ[currentPoint-row]){
					Q.push_back(currentPoint-row);	//north
					isInClusterOrQ[currentPoint-row] = true;
				}
				if (!isInClusterOrQ[currentPoint+row]){
					Q.push_back(currentPoint+row);	//south
					isInClusterOrQ[currentPoint+row] = true;
				}
			}
		}

		return clusterPoints;
	}

	void calculatePlanes_() 
	{
		for (int i = 0; i < 19200; i++) {
			planes_[i].A = normals_->points[i].normal_x;
			planes_[i].B = normals_->points[i].normal_y;
			planes_[i].C = normals_->points[i].normal_z;
			planes_[i].D = -1 * ( planes_[i].A * cloud_->points[i].x + planes_[i].B * cloud_->points[i].y + planes_[i].C * cloud_->points[i].z);
		}

	}

	bool samePlaneNormal(int pointOfInterest, int rootPoint, float epsilon) {
		if	((abs(planes_[rootPoint].A - planes_[pointOfInterest].A) < epsilon) 
		&&	(abs(planes_[rootPoint].B - planes_[pointOfInterest].B) < epsilon) 
		&&	(abs(planes_[rootPoint].C - planes_[pointOfInterest].C) < epsilon)
		&&	(abs(planes_[rootPoint].D - planes_[pointOfInterest].D) < 
					(
					abs(normals_->points[rootPoint].normal_z * cloud_->points[rootPoint].z * cloud_->points[rootPoint].z)
				+	abs(normals_->points[pointOfInterest].normal_z * cloud_->points[pointOfInterest].z * cloud_->points[pointOfInterest].z)
					) * epsilon
			)) 
			return true;
		
		return false;
	}

	//Depth Correction
	void correctDistances(pcl::PointCloud<PointType>::ConstPtr cloud, pcl::PointCloud<PointType>* mutable_cloud) {
		for(int i = 0; i < 19200; i++) {

			if (cloud->points[i].z > 0.5) {
				int index = (cloud->points[i].z * 10) - 4;
						 
				float zRatiof = 
					((correction_table[i][(int) index + 1] - correction_table[i][(int) index])
					/(((index + 1 + 4.0) / 10.0) - ((index + 4.0) / 10.0))
					* (cloud->points[i].z - ((index + 4.0) / 10.0)) 
					+ correction_table[i][(int) index]) / cloud->points[i].z;
						
				mutable_cloud->points[i].z *= zRatiof;
				mutable_cloud->points[i].y *= zRatiof;
				mutable_cloud->points[i].x *= zRatiof;
			
			}

		}
		return;

	}
};

#endif
