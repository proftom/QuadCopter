/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

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
      std::cout << "Average framerate("<< _WHAT_ << "): " << double(count)/double(now - last) << " Hz" <<  std::endl; \
      count = 0; \
      last = now; \
    } \
}while(false)

float G_smoothsize = 20.0f;
float G_depthdepend = 0.02f;
float G_epsilon = 0.04f;
int G_minpts = 60;

//template <typename PointType>
typedef pcl::PointXYZRGBA PointType;
//typedef NormalD PointType;
class NormalD : private pcl::Normal {
	
	NormalD(float x, float y, float z) 
	{
		this->normal_x = x;
		this->normal_y = y;
		this->normal_z = z;
	}

	float normal_d;

};


class OpenNIIntegralImageNormalEstimation
{
  public:
    typedef pcl::PointCloud<PointType> Cloud;
	typedef Cloud::Ptr CloudPtr; //typename
    typedef Cloud::ConstPtr CloudConstPtr; //typename

	unsigned int eventflag; 
	
	//This is used to hold the data used by DBSCAN
	pcl::PointCloud<pcl::Normal>::Ptr cloud_dbscanproc;

	pcl::IntegralImageNormalEstimation<PointType, pcl::Normal> ne_;

    pcl::visualization::CloudViewer viewer;
    std::string device_id_;
    boost::mutex mtx_;
    // Data
    pcl::PointCloud<pcl::Normal>::Ptr normals_;
    CloudConstPtr cloud_;
    bool new_cloud_;


	
    OpenNIIntegralImageNormalEstimation (const std::string& device_id = "")
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
      viewer.registerKeyboardCallback(&OpenNIIntegralImageNormalEstimation::keyboard_callback, *this);
    }
	
    void
    cloud_cb (const CloudConstPtr& cloud)
    {
      boost::mutex::scoped_lock lock (mtx_);
      //lock while we set our cloud;
      FPS_CALC ("computation");
      // Estimate surface normals

      normals_.reset (new pcl::PointCloud<pcl::Normal>);
	  //cld_render_ptr.reset(new pcl::PointCloud<PointType>);
	  
      double start = pcl::getTime ();
      ne_.setInputCloud (cloud);
	  ne_.compute (*normals_); 

      double stop = pcl::getTime ();
      //std::cout << "Time for normal estimation: " << (stop - start) * 1000.0 << " ms" << std::endl;
      //cloud_ = cloud;


	  cloud_dbscanproc = normals_; //TODO unessecary copy

	  static bool hasrun = false;
	  if(eventflag & 0x1){
		  if(!hasrun){
				hasrun = true;
				vector<vector<int>> clusterIndicies = floodFillAll(100, 100, 0.2);//DBSCAN(G_epsilon, G_minpts);

				pcl::PointCloud<PointType> pc(*cloud);

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
	  }else{
		  cloud_ = cloud;
		  if (!(eventflag & 0x1)){
			  new_cloud_ = true;
			  hasrun = false;
		  }
	  }

		

		//mtx_.unlock();
		//while(eventflag & 0x1);

    }

    void
    viz_cb (pcl::visualization::PCLVisualizer& viz)
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
		/*const pcl::PointCloud<pcl::PointXYZ>::ConstPtr p = pc;*/

        viz.addPointCloudNormals<PointType, pcl::Normal> (temp_cloud, temp_normals, 5, 0.05f, "normalcloud");
        new_cloud_ = false;
	
      }
    }

    void
    keyboard_callback (const pcl::visualization::KeyboardEvent& event, void*)
    {
		ofstream MyFile;
		MyFile.open ("data.csv", ios::out | ios::ate | ios::app) ;
      boost::mutex::scoped_lock lock (mtx_);
      
	  switch (event.getKeyCode ())
      {
        case '1':
          //ne_.setNormalEstimationMethod (pcl::IntegralImageNormalEstimation<PointType, pcl::Normal>::COVARIANCE_MATRIX);
          //std::cout << "switched to COVARIANCE_MATRIX method\n";
			//(cloud_dbscanproc->width, cloud_dbscanproc->height, cloud_dbscanproc);
			/*for (int p = 0; p <19200; p++){

				MyFile << cloud_dbscanproc->points[p].x << "," << cloud_dbscanproc->points[p].y <<","<< cloud_dbscanproc->points[p].z<<"\n";

			}*/


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
      }
    }

    void
    run ()
    {
      pcl::Grabber* interface = new pcl::OpenNIGrabber (device_id_, RESOLUTION_MODE, RESOLUTION_MODE);

      boost::function<void (const CloudConstPtr&)> f = boost::bind (&OpenNIIntegralImageNormalEstimation::cloud_cb, this, _1);
      boost::signals2::connection c = interface->registerCallback (f);

      viewer.runOnVisualizationThread (boost::bind(&OpenNIIntegralImageNormalEstimation::viz_cb, this, _1), "viz_cb");

      interface->start ();

      while (!viewer.wasStopped ())
      {
        boost::this_thread::sleep(boost::posix_time::seconds(1));
      }

      interface->stop ();
    }
	//
	//vector<NormalD> averageClusterNormal(vector<vector<int>> clusters) {
	//	//pcl::PointCloud<pcl::> averagedClusters = new pcl::PointCloud<PointT>();
	//	vector<int> averagedClusters;
	//	vector<NormalD> av;
	//	for(int i = 0; i < clusters.size(); i++) {
	//		float 
	//			accum_x = 0.0,
	//			accum_y = 0.0,
	//			accum_z = 0.0,
	//			accum_d = 0.0;
	//		
	//		for (int j = 0; j < clusters[i].size(); j++) {
	//			accum_x += cloud_dbscanproc->points[clusters[i][j]].normal_x;
	//			accum_y += cloud_dbscanproc->points[clusters[i][j]].normal_y;
	//			accum_z += cloud_dbscanproc->points[clusters[i][j]].normal_z;
	//			//ne_.depth_data_
	//			
	//			//cloud_dbscanproc->points[clusters[i][j]].
	//			//cloud_dbscanproc->points[clusters[i][j]]
	//				//clusters[i][j]
	//			//accum += clusters[i][j];
	//		}
	//		accum_x / clusters[i].size();
	//		accum_y / clusters[i].size();
	//		accum_z / clusters[i].size();
	//		av.push_back(NormalD(accum_x, accum_y, accum_z));
	//		//averagedClusters.push_back(accum);			
	//	}
	//	return av;
	//}

	
	////DBScan cluster in n-d space
	//vector<NormalD> DensityBasedScan(vector<int>) 
	//{

	//}

	//DBSCAN
	vector<vector<int>> DBSCAN (double epsilon, int minPoints){
		
		int sizeOfData = cloud_dbscanproc->size();
		vector<bool> Visited(sizeOfData, false);
		vector<bool> addedToCluster(sizeOfData, false);

		//Clusters
		vector<vector<int>> clusters;
		
		int clusterInd = 0;

		for(int i = 0; i<sizeOfData; i++){
			if(Visited[i]==false){
				Visited[i] = true;
			
				vector<int> NeighborPts = regionQuery(i, epsilon);
				vector<bool> pointsInNeighborPts(sizeOfData, false);

				if(!(NeighborPts.size()<minPoints))
				{
					vector<int> currentCluster;
					expandCluster(i, NeighborPts, pointsInNeighborPts, currentCluster, epsilon, minPoints, Visited, addedToCluster);
					clusters.push_back(currentCluster);
					clusterInd++;
				}
			}
		}

		cout << clusters.size() << " clusters found" << endl;

		return clusters;

	}

	//Get all points in region within esp
	vector<int> regionQuery(int currentPoint, double epsilon) {
		vector<int> k_indicies;
		

		for( int j = 0; j < cloud_dbscanproc->size(); j++) {
			if	(	(abs(cloud_dbscanproc->points[currentPoint].normal_x - cloud_dbscanproc->points[j].normal_x) < epsilon) 
				&&	(abs(cloud_dbscanproc->points[currentPoint].normal_y - cloud_dbscanproc->points[j].normal_y) < epsilon) 
				&&	(abs(cloud_dbscanproc->points[currentPoint].normal_z - cloud_dbscanproc->points[j].normal_z) < epsilon))
			{
				k_indicies.push_back(j);
			}
		}

		return k_indicies;
	}

	void expandCluster(	int inputInd, vector<int> NeighborPts, vector<bool> pointsInNeighborPts, vector<int> &currentCluster,
						double radius, int minPoints, vector<bool> &Visited, vector<bool> &addedToCluster){

		currentCluster.push_back(inputInd);

		for(int j = 0; j < NeighborPts.size(); j++){
			if(Visited[NeighborPts[j]]==false){
				Visited[NeighborPts[j]]=true;
				vector<int> secondNeighborPts = regionQuery(NeighborPts[j],radius);
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
	vector<vector<int>> floodFillAll(int minPts, int maxTries, double epsilon) 
	{
		vector<vector<int>> clusters;
		
		int dataSize = cloud_dbscanproc->size();
		vector<bool> isInCluster(dataSize, false);

		int count = 0;
		//Should check if certain points checked
		for (int tries = 0; tries < maxTries; ++tries)
		{
			int seed = rand()%dataSize;
			if (!isInCluster[seed]) 
			{
				vector<int> x = floodFillNaive(seed, isInCluster, count, epsilon);
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

	vector<int> floodFillNaive (int rootNode, vector<bool> isInClusterOrQ, int& count, double epsilon) 
	{
		
		vector<int> clusterPoints;
		
		std::vector<int> Q;
		Q.push_back(rootNode);

		while(!Q.empty()) {

			int currentPoint = Q.back();
			Q.pop_back();
			if (samePlaneNormal(currentPoint, rootNode, epsilon)) {
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
				if (!isInClusterOrQ[currentPoint-160]){
					Q.push_back(currentPoint-160);	//north
					isInClusterOrQ[currentPoint-160] = true;
				}
				if (!isInClusterOrQ[currentPoint+160]){
					Q.push_back(currentPoint+160);	//south
					isInClusterOrQ[currentPoint+160] = true;
				}
			}
			
		}

		return clusterPoints;
	}

	bool samePlaneNormal(int pointOfInterest, int rootPoint, double epsilon) {
		if	((abs(cloud_dbscanproc->points[rootPoint].normal_x - cloud_dbscanproc->points[pointOfInterest].normal_x) < epsilon) 
		&&	(abs(cloud_dbscanproc->points[rootPoint].normal_y - cloud_dbscanproc->points[pointOfInterest].normal_y) < epsilon) 
		&&	(abs(cloud_dbscanproc->points[rootPoint].normal_z - cloud_dbscanproc->points[pointOfInterest].normal_z) < epsilon))
			return true;

		return false;
	}

};

void
usage (char ** argv)
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

int
main (int argc, char ** argv)
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
    OpenNIIntegralImageNormalEstimation v ("");
    v.run ();
  }
  else
  {
    PCL_INFO ("PointXYZ mode enabled crash.\n");
    //OpenNIIntegralImageNormalEstimation<pcl::PointXYZ> v ("");
    //v.run ();
  }

  return (0);
}