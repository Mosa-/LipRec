#include <string>
#include <stdlib.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std_msgs;
using namespace std;
using namespace ros;

void imageCallback(const sensor_msgs::ImageConstPtr& msg){
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		//now cv_ptr is the matrix, do not forget "TYPE_" before "16UC1"
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO16);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	cv::imshow("view2", cv_ptr->image);
}


int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "liprec");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
	ros::NodeHandle n;

	Subscriber camImage = n.subscribe("/camera/rgb/image_raw", 1, imageCallback);

	cv::Mat img;
	cv::namedWindow("view");
	cv::startWindowThread();

	img = cv::imread("/home/felix/catkin_ws/src/liprec/src/berserk.jpeg", CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR);
	cv::imshow("view", img);

	cv::namedWindow("view2");
	cv::startWindowThread();


//	  ros::Subscriber sub2 = n.subscribe("/rqt_ccg/configparameter", 1, configParameter);

  // Subscribe output of the speech recognizer [store 2 messages]
//  ros::Subscriber sub = n.subscribe("/recognizer/output", 2, cmdParse);

  // Set publisher for publishing the resulting commands [store 5 messages]




  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();
  cv::destroyWindow("view");
  cv::destroyWindow("view2");


  return 0;
}
