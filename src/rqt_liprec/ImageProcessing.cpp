#include "rqt_liprec/ImageProcessing.h"

ImageProcessing::ImageProcessing() {
	// TODO Auto-generated constructor stub

}

ImageProcessing::~ImageProcessing() {
	// TODO Auto-generated destructor stub
}

void ImageProcessing::drawRectangle(Mat& iplImg, sensor_msgs::RegionOfInterest& roi, Scalar color){

	rectangle(iplImg, Point(roi.x_offset, roi.y_offset),
			Point(roi.x_offset + roi.width, roi.y_offset+ roi.height),
			color, 2, 8, 0);
}

void ImageProcessing::cutROIfromImage(Mat& src, Mat& out, sensor_msgs::RegionOfInterest& roi){
	Rect mouthROI(roi.x_offset, roi.y_offset, roi.width, roi.height);
	out = src(mouthROI).clone();
}

void ImageProcessing::squareImage(Mat& src){
	int max = std::max(src.rows, src.cols);
	Size squareSize(max, max);

	Mat squareImg(max, max, CV_8UC1, Scalar(0));
	cv::resize(src, src, squareSize);
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

QPixmap ImageProcessing::getPixmap(Mat& iplImg, bool useMonoImage){
	QPixmap pixMap;
    QImage dest;
    Mat temp;
    if(!iplImg.empty()){
        if(useMonoImage){
            dest = QImage((const uchar *) iplImg.data, iplImg.cols, iplImg.rows, iplImg.step, QImage::Format_Indexed8);
        }else{
            cvtColor(iplImg, temp, CV_BGR2RGB);
            dest = QImage((const uchar *) temp.data, temp.cols, temp.rows, temp.step, QImage::Format_RGB888);
        }
        dest.bits();
        pixMap.convertFromImage(dest);
    }
    return pixMap;
}

void ImageProcessing::setupVideoWriter(QString videoName, int frameWidth, int frameHeight)
{
    if(!video.isOpened()){
        Size s;
        s.width = frameWidth;
        s.height = frameHeight;
        video = VideoWriter(videoName.toStdString(), CV_FOURCC('M','J','P','G'), 30, s, false);

        if(!video.isOpened()){
            ROS_INFO("ERROR: Failed to write the video");
        }
    }
}

void ImageProcessing::writeFrameToVideo(Mat frame)
{
    Size s;
    s.width = frame.cols;
    s.height = frame.rows;
    video.write(frame);
}

void ImageProcessing::closeVideoWriter()
{
    if(video.isOpened()){
        video.release();
    }
}

void ImageProcessing::setUseMonoImage(bool use)
{
    this->useMonoImage = use;
}
