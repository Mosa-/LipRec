#ifndef IMAGEPROCESSING_H_
#define IMAGEPROCESSING_H_


#include <QPixmap>

#include <string>
#include <stdlib.h>
#include <ros/ros.h>
#include <ros/package.h>

#include <std_msgs/String.h>
#include <std_srvs/Empty.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "opencv2/video/tracking.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "CommonEnums.h"

using namespace std_msgs;
using namespace std;
using namespace ros;
using namespace cv;

class ImageProcessing {
public:
	ImageProcessing();
	virtual ~ImageProcessing();
	void drawRectangle(Mat& iplImg, sensor_msgs::RegionOfInterest& roi, Scalar color);
	void cutROIfromImage(Mat& src, Mat& out, sensor_msgs::RegionOfInterest& roi);
	void squareImage(Mat& src);
	void applyHistogramForLightCorrectionGHE(Mat& mat);
	void applyHistogramForLightCorrectionAHE(Mat& mat, int clipLimit, Size size);
	void applyBlur(Mat& mat, int sbMask, BlurType bt);
	int generatePixelDifference(Mat& currentFrame, Mat& lastFrame);
	Mat createImageAbsDiff(Mat& currentFrame, Mat& lastFrame);
	Mat createMotionHistoryImage(Mat& img, Mat& mhi, bool binarization, double binarThreshold, double mhiDuration);
	QPixmap getPixmap(Mat iplImg);
};

#endif /* IMAGEPROCESSING_H_ */
