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

template <typename PointType>
class OpenNIIntegralImageNormalEstimation
{
  public:
    typedef pcl::PointCloud<PointType> Cloud;
	typedef typename Cloud::Ptr CloudPtr;
    typedef typename Cloud::ConstPtr CloudConstPtr;

	typedef pcl::PointCloud<pcl::PointXYZ> CloudXYZ;
	typedef boost::shared_ptr<CloudXYZ> CloudPtrXYZ;
	typedef boost::shared_ptr<const CloudXYZ> CloudConstPtrXYZ;
	CloudConstPtr cloud_global;
	//For future
	//https://code.ros.org/trac/wg-ros-pkg/browser/trunk/stacks/drivers_experimental/dp_ptu47_pan_tilt_stage/src/extract_object_roi.cpp?rev=37618

    OpenNIIntegralImageNormalEstimation (const std::string& device_id = "")
      : viewer ("PCL OpenNI NormalEstimation Viewer")
    , device_id_(device_id)
    {
      //ne_.setNormalEstimationMethod (pcl::IntegralImageNormalEstimation<PointType, pcl::Normal>::COVARIANCE_MATRIX);
      ne_.setNormalEstimationMethod (pcl::IntegralImageNormalEstimation<PointType, pcl::Normal>::SIMPLE_3D_GRADIENT);
	  ne_.setDepthDependentSmoothing(true);
	  ne_.setMaxDepthChangeFactor (0.02f);
      ne_.setNormalSmoothingSize (15.0);
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

	  // Cloud cld (new Cloud(cloud));

      normals_.reset (new pcl::PointCloud<pcl::Normal>);

      double start = pcl::getTime ();
     ne_.setInputCloud (cloud);
	  ne_.compute (*normals_); 
	  cloud_global = cloud;

      double stop = pcl::getTime ();
      //std::cout << "Time for normal estimation: " << (stop - start) * 1000.0 << " ms" << std::endl;
      cloud_ = cloud;

	  cloud_global = cloud;
      new_cloud_ = true;

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
			for (int p = 0; p <19200; p++){
				MyFile << cloud_global->points[p].x << " " << cloud_global->points[p].y << " " << cloud_global->points[p].z << "\n";
			}

			MyFile << "//\n"; 
          break;
        case '2':
		  DBSCAN(0.2, 10);
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

	void DBSCAN (double epsilon, int minPoints){

		
		int sizeOfData = cloud_global->size();
		vector<bool> Visited;
		vector<bool> addedToCluster;
		vector<int> NeighborPts;

		//Clusters
		vector<int> currentCluster;
		vector<vector<int>> clusters;
		
		//Initialise
		for(int i = 0; i< sizeOfData; i++) 
		{
			Visited.push_back(false);
			addedToCluster.push_back(false); 
		}

		int clusterInd = 0;

		for(int i = 0; i<sizeOfData; i++){
			if(Visited[i]==false){
				Visited[i] = true;
			
				NeighborPts = regionQuery(i, 0.2);
				if(!(NeighborPts.size()<minPoints))
				{
					expandCluster(i, NeighborPts, currentCluster, epsilon, minPoints, Visited, addedToCluster);
					clusters.push_back(currentCluster);
					clusterInd++;
				}
			}
		}

		cout << clusters.size() << " clusters found" << endl;

		//Colour clusters
		for(int i = 0; i < clusters.size(); i++) 
		{
			for(int j = 0; j < clusters[j]; j++) {
				cloud_global->points[j].r = 255; //clusters[j]
			}
		}

	}

	//Get all points in region within esp
	vector<int> regionQuery(int currentPoint, double epsilon) {
		vector<int> k_indicies;
		for( int j = 0; j < cloud_global->size(); j++) {
			if	(	(abs(cloud_global->points[currentPoint].x - cloud_global->points[j].x) < epsilon) 
				&&	(abs(cloud_global->points[currentPoint].y - cloud_global->points[j].y) < epsilon) 
				&&	(abs(cloud_global->points[currentPoint].z - cloud_global->points[j].z) < epsilon))
			{
				k_indicies.push_back(j);
			}
		}
		return k_indicies;
	}

	void expandCluster(	int inputInd, vector<int> NeighborPts,vector<int> currentCluster,
						double radius, int minPoints, vector<bool> &Visited, vector<bool> &addedToCluster){

		currentCluster.push_back(inputInd);
		vector<int> secondNeighborPts;

		for(int j = 0; j < NeighborPts.size(); j++){
			if(Visited[NeighborPts[j]]==false){
				Visited[NeighborPts[j]]=true;
				secondNeighborPts = regionQuery(NeighborPts[j],radius);
				if(secondNeighborPts.size()>=minPoints){
					inplace_union(NeighborPts, secondNeighborPts);
				}
			}
			if (!addedToCluster[j])
				currentCluster.push_back(NeighborPts[j]);
				//if neighborPts[j] has yet to be added to a cluster add it to this one
		}

	}

	void inplace_union(std::vector<int>& a,  std::vector<int>& b){
		std::sort (a.begin(),a.end());
		std::sort (b.begin(),b.end());
		int mid = a.size(); //Store the end of first sorted range

		//First copy the second sorted range into the destination vector
		std::copy(b.begin(), b.end(), std::back_inserter(a));

		//Then perform the in place merge on the two sub-sorted ranges.
		std::inplace_merge(a.begin(), a.begin() + mid, a.end());

		//Remove duplicate elements from the sorted vector
		a.erase(std::unique(a.begin(), a.end()), a.end());
	}

    pcl::IntegralImageNormalEstimation<PointType, pcl::Normal> ne_;

    pcl::visualization::CloudViewer viewer;
    std::string device_id_;
    boost::mutex mtx_;
    // Data
    pcl::PointCloud<pcl::Normal>::Ptr normals_;
    CloudConstPtr cloud_;
    bool new_cloud_;
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
    OpenNIIntegralImageNormalEstimation<pcl::PointXYZRGBA> v ("");
    v.run ();
  }
  else
  {
    PCL_INFO ("PointXYZ mode enabled.\n");
    OpenNIIntegralImageNormalEstimation<pcl::PointXYZ> v ("");
    v.run ();
  }

  return (0);
}
