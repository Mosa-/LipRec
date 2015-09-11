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

    double calcXCircularCoordinate(int imgSize, int xSquareCoordinate);
    double calcYCircularCoordinate(int imgSize, int ySquareCoordinate);
    double calcRadiusDist(int imgSize, int xSquareCoordinate, int ySquareCoordinate);
    double calcAngleCircular(int imgSize, int xSquareCoordinate, int ySquareCoordinate);

    void setupVideoWriter(QString videoName, int frameWidth, int frameHeight);
    void writeFrameToVideo(Mat frame);
    void closeVideoWriter();

private:
    VideoWriter video;
};

#endif /* IMAGEPROCESSING_H_ */
