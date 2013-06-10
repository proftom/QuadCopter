#include <boost/thread/shared_mutex.hpp>
#include <boost/thread/thread.hpp>
//#include "PCL_Conncetor.h"
#include "Vision.h"

//prototype
int kalman(QCVision& vision);

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


//Main
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

	//Create a new grab object
	pcl::OpenNIGrabber grabber("", RESOLUTION_MODE, RESOLUTION_MODE);



	



	if (grabber.providesCallback<pcl::OpenNIGrabber::sig_cb_openni_point_cloud_rgba> ())
	{
		PCL_INFO ("PointXYZRGBA mode enabled.\n");	
		
		QCVision vision("");	
		//dummy d(&vision);
		boost::thread* thr = new boost::thread(boost::bind(&QCVision::run, &vision));
		//boost::thread dthr(boost::bind(&kalman));
		kalman(vision);	
		thr->join();
		//dthr.join();
	;

	


		//con.readPlaneFromBuffer();

	}
	else
	{
		PCL_INFO ("PointXYZ mode enabled crash.\n");
	}

	return (0);
}