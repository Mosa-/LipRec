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

void ImageProcessing::applyLipsSegmentationSaturation(Mat &mouthImg, int saturationValue)
{
    double saturation = 0.0;
    double r,g,b;
    int saturationHistogram[100] = {};
    int s;
    int noPixelImg = 0;

    for (int y = 0; y < mouthImg.rows; ++y) {
        for (int x = 0; x < mouthImg.cols; ++x) {
            //ROS_INFO("%d %d: B=%d G=%d R=%d", y, x, mouthImg.at<cv::Vec3b>(Point(x, y))[0], mouthImg.at<cv::Vec3b>(Point(x, y))[1], mouthImg.at<cv::Vec3b>(Point(x, y))[2]);
            b = mouthImg.at<cv::Vec3b>(Point(x, y))[B];
            g = mouthImg.at<cv::Vec3b>(Point(x, y))[G];
            r = mouthImg.at<cv::Vec3b>(Point(x, y))[R];
            saturation = fabs(2 * atan((r-g)/r)/M_PI);
            s = saturation * 100;
            if(s > 0){
                saturationHistogram[s-1]++;
            }else{
                saturationHistogram[s]++;
            }
            noPixelImg++;
            //ROS_INFO("saturation: %f b: %.0f g: %.0f r: %.0f", saturation, b, g, r);
        }
    }

    int amountFacePixel = noPixelImg - (noPixelImg*saturationValue/100);
    int thresholdIndex = 0;


    for (int i = 0; i < 100; ++i) {
        if(saturationHistogram[i] > 0){
            //ROS_INFO("oneThirdMouthPxl %d, saturationHistogram[i] %d i %d", oneThirdMouthPxl, saturationHistogram[i], i);
            amountFacePixel -= saturationHistogram[i];
        }

        if(amountFacePixel <= 0){
            thresholdIndex = i;
            break;
        }
    }

    for (int y = 0; y < mouthImg.rows; ++y) {
        for (int x = 0; x < mouthImg.cols; ++x) {
            b = mouthImg.at<cv::Vec3b>(Point(x, y))[B];
            g = mouthImg.at<cv::Vec3b>(Point(x, y))[G];
            r = mouthImg.at<cv::Vec3b>(Point(x, y))[R];
            saturation = fabs(2 * atan((r-g)/r)/M_PI);
            s = saturation * 100;
            int newS = 0;

            if(s > 0){
                newS = s-1;
            }else{
                newS = s;
            }


            if(newS >= thresholdIndex){
                mouthImg.at<cv::Vec3b>(Point(x, y))[B] = 255;
                mouthImg.at<cv::Vec3b>(Point(x, y))[G] = 255;
                mouthImg.at<cv::Vec3b>(Point(x, y))[R] = 255;
            }
        }
    }


    //        int noPixel = 0;
    //        for (int i = 0; i < 100; ++i) {
    //            ROS_INFO("saturation: %d, amount: %d", i, saturationHistogram[i]);
    //            noPixel += saturationHistogram[i];
    //        }
    //        ROS_INFO("saturationHistogram[i] noPixel: %d <> noPixelImg: %d", noPixel, noPixelImg);
}

Mat ImageProcessing::calcColorHistogramEqualization(Mat &img)
{
    vector<Mat> channels;
    Mat imgHistEqualized;

    cvtColor(img, imgHistEqualized, CV_BGR2YCrCb); //change the color image from BGR to YCrCb format

    split(imgHistEqualized, channels);

    equalizeHist(channels[0], channels[0]);

    merge(channels, imgHistEqualized);

    cvtColor(imgHistEqualized, imgHistEqualized, CV_YCrCb2BGR);

    return imgHistEqualized;
}

double ImageProcessing::pseudoHuePxl(Mat img, int x, int y)
{
    double r,g,b;
    b = img.at<cv::Vec3b>(Point(x, y))[B];
    g = img.at<cv::Vec3b>(Point(x, y))[G];
    r = img.at<cv::Vec3b>(Point(x, y))[R];

    if(g+r == 0){
        return 0;
    }

    return (r/(g+r));
}

int ImageProcessing::luminancePxl(Mat img, int x, int y)
{
    double r,g,b;
    b = img.at<cv::Vec3b>(Point(x, y))[B];
    g = img.at<cv::Vec3b>(Point(x, y))[G];
    r = img.at<cv::Vec3b>(Point(x, y))[R];

    /// http://stackoverflow.com/questions/596216/formula-to-determine-brightness-of-rgb-color
    return (int) (0.2126*r + 0.7152*g + 0.0722*b);
    // return (int) (0.299*r + 0.587*g + 0.114*b);
    // return (int) (0.33*r + 0.5*g + 0.16*b);
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

QPixmap ImageProcessing::getPixmap(Mat iplImg, bool useMonoImage){
    QPixmap pixMap;
    QImage dest;
    Mat temp;
    Mat mono8_img;

    if(!iplImg.empty()){
        if(iplImg.type() == CV_32FC1){
            mono8_img = cv::Mat(iplImg.size(), CV_8UC1);
            cv::convertScaleAbs(iplImg, mono8_img, 100, 0.0);
            dest = QImage((const uchar *) mono8_img.data, mono8_img.cols, mono8_img.rows, mono8_img.step, QImage::Format_Indexed8);
        }else if(useMonoImage || iplImg.type() == CV_16UC1 || iplImg.type() == CV_8U){
            dest = QImage((const uchar *) iplImg.data, iplImg.cols, iplImg.rows, iplImg.step, QImage::Format_Indexed8);
        }else{
            cvtColor(iplImg, temp, CV_BGR2RGB);
            dest = QImage((const uchar *) temp.data, temp.cols, temp.rows, temp.step, QImage::Format_RGB888);
        }

        if(dest.byteCount() > 0){
          //dest.bits();
          pixMap = QPixmap(dest.width(), dest.height());
          pixMap.convertFromImage(dest);
        }
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
