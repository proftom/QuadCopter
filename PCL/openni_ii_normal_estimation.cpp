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
//#include "C:/Users/Prof/Documents/GitHub/QuadCopter/PCL/tvmet/include/tvmet/Matrix.h"
#include "Depth_Correction_Array.txt"
#include <Eigen/Dense>
using namespace Eigen;
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

float G_smoothsize = 20.0f;
float G_depthdepend = 0.02f;
#endif

typedef pcl::PointXYZRGBA PointType;
typedef Matrix< float , 16 , 1> Vector16f;
class QCVision
{

  private:
	clock_t t1, t2;
    bool new_cloud_;

  public:
    typedef pcl::PointCloud<PointType> Cloud;
	typedef Cloud::Ptr CloudPtr; //typename
    typedef Cloud::ConstPtr CloudConstPtr; //typename

	Vector16f* stateVector;

	Matrix3f* DCM; 

	unsigned int eventflag; 
	unsigned int Perseventflag; 

	
    pcl::visualization::CloudViewer viewer;
    std::string device_id_;
    boost::mutex mtx_;

    // Data
    CloudConstPtr cloud_;						//Raw cloud
	pcl::PointCloud<pcl::Normal>::Ptr normals_; //Normal cloud
	pcl::IntegralImageNormalEstimation<PointType, pcl::Normal> ne_; //Normal estimation object
	vector<Plane> planes_;						//Found planes


	QCVision (const std::string& device_id = "")
      : viewer ("PCL OpenNI NormalEstimation Viewer")
    , device_id_(device_id)
	, eventflag(0)
    {
      //ne_.setNormalEstimationMethod (pcl::IntegralImageNormalEstimation<PointType, pcl::Normal>::COVARIANCE_MATRIX);
      ne_.setNormalEstimationMethod (pcl::IntegralImageNormalEstimation<PointType, pcl::Normal>::SIMPLE_3D_GRADIENT);
	  ne_.setDepthDependentSmoothing(true);
	  ne_.setMaxDepthChangeFactor (G_depthdepend);
      ne_.setNormalSmoothingSize (G_smoothsize);
      new_cloud_ = false;
      viewer.registerKeyboardCallback(&QCVision::keyboard_callback, *this);
	  planes_.assign(19200, Plane(0,0,0,0));

    }
	
	void cloud_cb (const CloudConstPtr& cloud)
    {
	  t2 = clock(); 
	  boost::mutex::scoped_lock lock (mtx_);
 
      //lock while we set our cloud;
      FPS_CALC ("computation");
      // Estimate surface normals

	  // Cloud cld (new Cloud(cloud));

      normals_.reset (new pcl::PointCloud<pcl::Normal>);
	  //cld_render_ptr.reset(new pcl::PointCloud<PointType>);
	  
      double start = pcl::getTime ();
      ne_.setInputCloud (cloud);
	  ne_.compute (*normals_); 

      double stop = pcl::getTime ();
      //std::cout << "Time for normal estimation: " << (stop - start) * 1000.0 << " ms" << std::endl;
      cloud_ = cloud;

	  
	  normals_ = normals_; //TODO unessecary copy
	  
	  
	  static bool hasrun = false;
	  if(eventflag & 0x1){
		  if(!hasrun){
				//hasrun = true;



				//ofstream myfile;
				//myfile.open("planes.txt",ios::app);
				//myfile << planes.size() << endl;

				//for (int i = 0; i < planes.size(); i++) {
				//	planes[i].calculateCovarianceMatrix(planes_);
				//	double dist = sqrt( (planes[i].A * planes[i].A) + (planes[i].B * planes[i].B) + (planes[i].C * planes[i].C));

				//	myfile << planes[i].A * (1.0 / dist) << " " << planes[i].B * (1.0 / dist) << " " << planes[i].C * (1.0 / dist) << " " << planes[i].D << endl;
				//	for (int j = 0; j < planes[i].covariance.size(); j++) {
				//		myfile << planes[i].covariance[j][0] << " " << planes[i].covariance[j][1] << " " << planes[i].covariance[j][2] << " " << planes[i].covariance[j][3] << endl;
				//	}
				//	myfile << planes[i].indicies.size() << endl;
				//	
				//}
				//
				//myfile << t2 << endl;
				//myfile << "<<<" << endl; 
				
				//vector<Plane> h = DBScanND(planes,1,0.05);

				//For displaying coloured planes

				 pcl::PointCloud<PointType> pc(*cloud);
				 correctDistances(cloud, &pc);

				 				
				vector<vector<int>> clusterIndicies = floodFillAll(1000, 20);//DBSCAN(G_epsilon, G_minpts);
				cloud_ = cloud;
				vector<Plane> planes = ClusterToAveragePlane(clusterIndicies);

				
				ofstream myfile;
				myfile.open("planes.txt",ios::app);
				myfile << planes.size() << endl;

				for (int i = 0; i < planes.size(); i++) {
					planes[i].calculateCovarianceMatrix(planes_);
					double dist = sqrt( (planes[i].A * planes[i].A) + (planes[i].B * planes[i].B) + (planes[i].C * planes[i].C));

					myfile << planes[i].A * (1.0 / dist) << " " << planes[i].B * (1.0 / dist) << " " << planes[i].C * (1.0 / dist) << " " << planes[i].D << endl;
					for (int j = 0; j < planes[i].covariance.size(); j++) {
						myfile << planes[i].covariance[j][0] << " " << planes[i].covariance[j][1] << " " << planes[i].covariance[j][2] << " " << planes[i].covariance[j][3] << endl;
					}
					myfile << planes[i].indicies.size() << endl;
					
				}
				
				myfile << t2 << endl;
				myfile << "<<<" << endl; 

				//Colour clusters
				for(int i = 0; i < clusterIndicies.size(); i++) 
				{
					char r,b,g;
					r = rand()%256;
					g = rand()%256;
					b = rand()%256;
					for(int j = 0; j < clusterIndicies[i].size(); j++) {
						pc.points[clusterIndicies[i][j]].r = r;
						pc.points[clusterIndicies[i][j]].g = g;
						pc.points[clusterIndicies[i][j]].b = b;
					}
				}

				CloudConstPtr inputtmp(new pcl::PointCloud<PointType>(pc));
				cloud_ = inputtmp;

				new_cloud_ = true;
		  }
	  }
	  else{
		  cloud_ = cloud;
		  if (!(eventflag & 0x1)){
			  new_cloud_ = true;
			  hasrun = false;
		  }
		//apply transform based on camera location 
		if(Perseventflag & 0x1){
			MatrixXf Tran(4,3);
			Tran.block<3,3>(0,0)= *DCM;
			Tran.block<3,1>(4,1)= stateVector->segment<3>(0);
		}
		else{
		}
	  }
    }

    void viz_cb (pcl::visualization::PCLVisualizer& viz)
    {
      mtx_.lock ();
      if (!cloud_ || !normals_)
      {
        //boost::this_thread::sleep(boost::posix_time::seconds(1));
        mtx_.unlock ();
        return;
      }

      CloudConstPtr temp_cloud;
      pcl::PointCloud<pcl::Normal>::Ptr temp_normals;

      temp_cloud.swap (cloud_); //here we set cloud_ to null, so that
      temp_normals.swap (normals_);
      mtx_.unlock ();

      if (!viz.updatePointCloud (temp_cloud, "OpenNICloud"))
      {
        viz.addPointCloud (temp_cloud, "OpenNICloud");
		viz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "OpenNICloud");
        viz.resetCameraViewpoint ("OpenNICloud");
      }
      // Render the data
      if (new_cloud_)
      {

        viz.removePointCloud ("normalcloud");

        //viz.addPointCloudNormals<PointType, pcl::Normal> (temp_cloud, temp_normals, 5, 0.05f, "normalcloud");
        new_cloud_ = false;

      }
    }

    void keyboard_callback (const pcl::visualization::KeyboardEvent& event, void*)
    {
		ofstream MyFile;
		MyFile.open ("data1.csv", ios::out | ios::ate | ios::app) ;
      boost::mutex::scoped_lock lock (mtx_);
      
	  switch (event.getKeyCode ())
      {
        case '1':

       /*   ne_.setNormalEstimationMethod (pcl::IntegralImageNormalEstimation<PointType, pcl::Normal>::COVARIANCE_MATRIX);
          std::cout << "switched to COVARIANCE_MATRIX method\n";
			(cloud_dbscanproc->width, cloud_dbscanproc->height, cloud_dbscanproc);*/
			for (int p = 0; p <19200; p++){

				MyFile << cloud_->points[p].x << "," << cloud_->points[p].y <<","<< cloud_->points[p].z<<"\n";

			}


			MyFile << "//\n"; 
          break;
        case '2':

		  
          ne_.setNormalEstimationMethod (pcl::IntegralImageNormalEstimation<PointType, pcl::Normal>::AVERAGE_3D_GRADIENT);
          std::cout << "switched to AVERAGE_3D_GRADIENT method\n";
          break;
        case '3':
          ne_.setNormalEstimationMethod (pcl::IntegralImageNormalEstimation<PointType, pcl::Normal>::AVERAGE_DEPTH_CHANGE);
          std::cout << "switched to AVERAGE_DEPTH_CHANGE method\n";
          break;
        case '4':
          ne_.setNormalEstimationMethod (pcl::IntegralImageNormalEstimation<PointType, pcl::Normal>::SIMPLE_3D_GRADIENT);
          std::cout << "switched to SIMPLE_3D_GRADIENT method\n";
          break;
		case '5':
			ne_.setMaxDepthChangeFactor (G_depthdepend);
			ne_.setNormalSmoothingSize (G_smoothsize);
			eventflag |= 0x1;
			break;
		case '6':
			eventflag &= ~0x1;
			break;
		case '7':
			Perseventflag |= ~0x1;
			break;
		case '8':
			Perseventflag &= ~0x1;
			break;
      }
    }

    void run ()
    {
      pcl::Grabber* interface = new pcl::OpenNIGrabber (device_id_, RESOLUTION_MODE, RESOLUTION_MODE);

      boost::function<void (const CloudConstPtr&)> f = boost::bind (&QCVision::cloud_cb, this, _1);
      boost::signals2::connection c = interface->registerCallback (f);

      viewer.runOnVisualizationThread (boost::bind(&QCVision::viz_cb, this, _1), "viz_cb");

      interface->start ();

      while (!viewer.wasStopped ())
      {
        boost::this_thread::sleep(boost::posix_time::seconds(1));
      }

      interface->stop ();
    }
	
	vector<Plane> ClusterToAveragePlane(vector<vector<int>> clusters) 
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

	vector<Plane> DBScanND(vector<Plane> planes, int minPoints, double epsilon) 
	{
		int sizeOfData = planes.size();
		vector<bool> Visited(sizeOfData, false);
		vector<bool> addedToCluster(sizeOfData, false);

		//Clusters
		vector<vector<int>> clusters;
		
		int clusterInd = 0;

		for(int i = 0; i<sizeOfData; i++){
			if(Visited[i]==false){
				Visited[i] = true;
			
				vector<int> NeighborPts = regionQuery(planes, i, epsilon);
				vector<bool> pointsInNeighborPts(sizeOfData, false);

				if(!(NeighborPts.size()<minPoints))
				{
					vector<int> currentCluster;
					expandCluster(i, NeighborPts, pointsInNeighborPts, currentCluster, epsilon, minPoints, Visited, addedToCluster, planes);
					clusters.push_back(currentCluster);
					clusterInd++;
				}
			}

		}

		cout << "Planes merging from " << planes.size() << " to " << clusters.size() << endl;
		

		//Now create a structure with the planes averaged according to how many elements are within the plane
		//Make each element unique
		std::vector<Plane> mergedPlanes;
		for (int i = 0; i < clusters.size(); i++) {

			//O(n log n)
			std::set<int> unique (clusters[i].begin(), clusters[i].end());
			Plane mergingPlane(0,0,0,0);
			int nTotalIndicies = 0;
			for (set<int>::iterator it=unique.begin() ; it != unique.end(); it++ ) {

				mergingPlane.A += (planes[(*it)].A * planes[(*it)].indicies.size());
				mergingPlane.B += (planes[(*it)].B * planes[(*it)].indicies.size());
				mergingPlane.C += (planes[(*it)].C * planes[(*it)].indicies.size());
				mergingPlane.D += (planes[(*it)].D * planes[(*it)].indicies.size());
				mergingPlane.indicies.insert(mergingPlane.indicies.end(), planes[(*it)].indicies.begin(), planes[(*it)].indicies.end());
				nTotalIndicies += planes[(*it)].indicies.size();
			
			}
			mergingPlane.A /= nTotalIndicies; mergingPlane.B /= nTotalIndicies; mergingPlane.C /= nTotalIndicies; mergingPlane.D /= nTotalIndicies;
			mergingPlane.calculateCovarianceMatrix(planes_);
			mergedPlanes.push_back(mergingPlane);

		}

		//mergeMelo(mergedPlanes[0], mergedPlanes[1]);

		//myfile << mergedPlanes.size() << endl;
		


		return mergedPlanes;
	}

	//Get all planes in region within epsilon
	vector<int> regionQuery(vector<Plane> &planes, int currentPoint, double epsilon) {
		vector<int> k_indicies;

		for( int j = 0; j < planes.size(); j++) {
			if	(	(abs(planes[currentPoint].A - planes[j].A) < epsilon) 
				&&	(abs(planes[currentPoint].B - planes[j].B) < epsilon) 
				&&	(abs(planes[currentPoint].C - planes[j].C) < epsilon)
				&&	(abs(planes[currentPoint].D - planes[j].D) < epsilon)				)
			{
				k_indicies.push_back(j);
			}
		}

		return k_indicies;
	}

	void expandCluster(	int inputInd, vector<int> NeighborPts, vector<bool> pointsInNeighborPts, vector<int> &currentCluster,
					double radius, int minPoints, vector<bool> &Visited, vector<bool> &addedToCluster, vector<Plane> &planes){

		currentCluster.push_back(inputInd);

		for(int j = 0; j < NeighborPts.size(); j++)
		{
			if(Visited[NeighborPts[j]]==false){
				Visited[NeighborPts[j]]=true;
				vector<int> secondNeighborPts = regionQuery(planes, NeighborPts[j],radius);
				if(secondNeighborPts.size()>=minPoints){
					conditionalInsert(NeighborPts, pointsInNeighborPts, secondNeighborPts);
				}
			}
			if (!addedToCluster[j])
				currentCluster.push_back(NeighborPts[j]);
				//if neighborPts[j] has yet to be added to a cluster add it to this one
		}

	}

	void conditionalInsert(std::vector<int>& destination, std::vector<bool>& isInDest, std::vector<int> source){
		for (int i = 0; i < source.size(); i++) {
			if (!isInDest[source[i]]) {
				destination.push_back(source[i]);
				isInDest[source[i]] = true;
			}
		}
	}
	
	//Flood fill
	vector<vector<int>> floodFillAll(int minPts, int maxTries) 
	{
		vector<vector<int>> clusters;
		
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

	bool samePlaneNormal(int pointOfInterest, int rootPoint, double epsilon) {
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

void usage (char ** argv)
{
  std::cout << "usage: " << argv[0] << " [<device_id>]\n\n";

  openni_wrapper::OpenNIDriver& driver = openni_wrapper::OpenNIDriver::getInstance ();
  if (driver.getNumberDevices () > 0)
  {
    for (unsigned deviceIdx = 0; deviceIdx < driver.getNumberDevices (); ++deviceIdx)
    {
      cout << "Device: " << deviceIdx + 1 << ", vendor: " << driver.getVendorName (deviceIdx) << ", product: " << driver.getProductName (deviceIdx)
              << ", connected: " << driver.getBus (deviceIdx) << " @ " << driver.getAddress (deviceIdx) << ", serial number: \'" << driver.getSerialNumber (deviceIdx) << "\'" << endl;
      cout << "device_id may be #1, #2, ... for the first second etc device in the list or" << endl
           << "                 bus@address for the device connected to a specific usb-bus / address combination (works only in Linux) or" << endl
           << "                 <serial-number> (only in Linux and for devices which provide serial numbers)"  << endl;
    }
  }
  else
    cout << "No devices connected." << endl;
}

int main (int argc, char ** argv)
{


	  std::string arg;
  if (argc > 1)
    arg = std::string (argv[1]);

  openni_wrapper::OpenNIDriver& driver = openni_wrapper::OpenNIDriver::getInstance ();
  if (arg == "--help" || arg == "-h" || driver.getNumberDevices () == 0)
  {
    usage (argv);
    return 1;
  }
  std::cout << "hi";
  std::cout << "Press following keys to switch to the different integral image normal estimation methods:\n";
  std::cout << "<1> COVARIANCE_MATRIX method\n";
  std::cout << "<2> AVERAGE_3D_GRADIENT method\n";
  std::cout << "<3> AVERAGE_DEPTH_CHANGE method\n";
  std::cout << "<4> SIMPLE_3D_GRADIENT method\n";
  std::cout << "<Q,q> quit\n\n";



  pcl::OpenNIGrabber grabber("", RESOLUTION_MODE, RESOLUTION_MODE);
  if (grabber.providesCallback<pcl::OpenNIGrabber::sig_cb_openni_point_cloud_rgba> ())
  {
    PCL_INFO ("PointXYZRGBA mode enabled.\n");
    QCVision v ("");
    v.run ();
  }
  else
  {
    PCL_INFO ("PointXYZ mode enabled crash.\n");
    //QCVision<pcl::PointXYZ> v ("");
    //v.run ();
  }

  return (0);
}