#include "rqt_liprec/ImageProcessing.h"

ImageProcessing::ImageProcessing() {
	// TODO Auto-generated constructor stub

}

ImageProcessing::~ImageProcessing() {
	// TODO Auto-generated destructor stub
}


Mat ImageProcessing::cutROIfromImage(Mat& src, sensor_msgs::RegionOfInterest& roi){
	Rect mouthROI(roi.x_offset, roi.y_offset, roi.width, roi.height);
	Mat mouth = src(mouthROI).clone();
	return mouth;
}

void ImageProcessing::applyHistogramForLightCorrectionGHE(Mat& mat){
	equalizeHist(mat, mat);
}
void ImageProcessing::applyHistogramForLightCorrectionAHE(Mat& mat, int clipLimit, Size size){
	Ptr<CLAHE> clahe = createCLAHE(clipLimit,size);
	clahe->apply(mat, mat);
}
void ImageProcessing::applyBlur(Mat& mat, int sbMask, BlurType bt){
	int sbMaskValue = sbMask;
	if(sbMaskValue%2 == 0){
		sbMaskValue += 1;
	}
	if(bt == BLUR){
		blur(mat, mat,Size(sbMaskValue,sbMaskValue), Point(-1,-1));
	}else if(bt == MEDIAN){
		medianBlur(mat, mat, sbMaskValue);
	}else if(bt == GAUSSIAN){
		GaussianBlur(mat, mat, Size(sbMaskValue,sbMaskValue),0,0);
	}
}

int ImageProcessing::generatePixelDifference(Mat& currentFrame, Mat& lastFrame){
	//1.squared mean difference
	double d = 0;

	int pixelDifference = 0;
	if(!currentFrame.empty() && !lastFrame.empty()){
		for (int i = 0; i < currentFrame.cols; ++i) {
			for (int j = 0; j < currentFrame.rows; ++j) {
				if(lastFrame.cols == 0){
					pixelDifference += 0 - currentFrame.at<uchar>(j,i);
				}else{
					pixelDifference += abs(lastFrame.at<uchar>(j,i) - currentFrame.at<uchar>(j,i));
				}
			}
		}
	}

	//d = pow(pixelDifference / ((double) mouthImg.cols*mouthImg.rows),2);
	d = pixelDifference / ((double) currentFrame.cols*currentFrame.rows);
	return d;
}
Mat ImageProcessing::createImageAbsDiff(Mat& currentFrame, Mat& lastFrame){
	Mat silh;
	absdiff(lastFrame, currentFrame, silh);
	return silh;
}
Mat ImageProcessing::createMotionHistoryImage(Mat& img, Mat& mhi, bool binarization, double binarThreshold, double mhiDuration){
	QPixmap pixMap;
	double timestamp = (double) clock()/CLOCKS_PER_SEC;
	Size size = Size(img.size().width, img.size().height);
	Mat silh = Mat::zeros(size, CV_8UC1);


	if(mhi.empty() || img.size != mhi.size){
		mhi.release();
		mhi = Mat(size, CV_32FC1, Scalar(0,0,0));
	}

	if(binarization)
		threshold(img,silh, binarThreshold,1,cv::THRESH_BINARY);

	updateMotionHistory(silh, mhi, timestamp, mhiDuration);
	Mat mask;
	mhi.convertTo(mask, CV_8UC1, 255.0/mhiDuration, (mhiDuration-timestamp)*255.0/mhiDuration );

	return mask;
}
QPixmap ImageProcessing::getPixmap(Mat iplImg){
	QPixmap pixMap;
	QImage dest((const uchar *) iplImg.data, iplImg.cols, iplImg.rows, iplImg.step, QImage::Format_Indexed8);
	pixMap.convertFromImage(dest,Qt::ColorOnly);
	return pixMap;
}
