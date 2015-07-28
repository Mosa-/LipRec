#include "rqt_liprec/LipRec.h"
#include <pluginlib/class_list_macros.h>


namespace rqt_liprec {

LipRec::LipRec()
  : rqt_gui_cpp::Plugin()
  , last(0), timeoutROIdetection(500), blackBorder(0), widget_(0), stateDetectionStartEndFrame(Idle)
{
  // Constructor is called first before initPlugin function, needless to say.

  // give QObjects reasonable names
  setObjectName("rqt_liprec");
}


void LipRec::initPlugin(qt_gui_cpp::PluginContext& context)
{
	// access standalone command line arguments
	QStringList argv = context.argv();
	// create QWidget
	widget_ = new QWidget();
	// extend the widget with all attributes and children from UI file
	ui_.setupUi(widget_);
	// add widget to the user interface
	context.addWidget(widget_);

	qRegisterMetaType<cv::Mat>("cv::Mat");

	camImage = getNodeHandle().subscribe("/liprecKinect/rgb/image_raw", 10, &LipRec::imageCallback, this);

	faceROISub = getNodeHandle().subscribe("/face_detection/faceROI", 10, &LipRec::faceROICallback, this);
	mouthROISub = getNodeHandle().subscribe("/face_detection/mouthROI", 10, &LipRec::mouthROICallback, this);

	if(argv.size() != 3){
		ROS_INFO("Three arguments necessary!");
	}else{
		int showFaceROI = argv[0].toInt();
		int showMouthROI = argv[1].toInt();
		blackBorder = argv[2].toInt();
		if(showFaceROI == 1)
			ui_.cbFaceROI->setChecked(true);

		if(showMouthROI == 1)
			ui_.cbMouthROI->setChecked(true);

		ROS_INFO("Show FaceROI: %s",argv[0].toStdString().c_str());
		ROS_INFO("Show MouthROI: %s",argv[1].toStdString().c_str());
		ROS_INFO("BlackBorder: %s",argv[2].toStdString().c_str());
	}

	MHI_DURATION = ui_.dsbMHIDuration->value();
	NO_CYCLIC_FRAME = ui_.sbMHIFC->value();

	QObject::connect(&faceROITimer, SIGNAL(timeout()), this, SLOT(faceROItimeout()));
	QObject::connect(&mouthROITimer, SIGNAL(timeout()), this, SLOT(mouthROItimeout()));

	faceROITimer.start(timeoutROIdetection);
	mouthROITimer.start(timeoutROIdetection);

	QObject::connect(this, SIGNAL(updateCam(cv::Mat)), this, SLOT(getCamPic(cv::Mat)));

}

void LipRec::shutdownPlugin()
{

}

void LipRec::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
  // TODO save intrinsic configuration, usually using:
  // instance_settings.setValue(k, v)
}

void LipRec::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
  // TODO restore intrinsic configuration, usually using:
  // v = instance_settings.value(k)
}

/*bool hasConfiguration() const
{
  return true;
}

void triggerConfiguration()
{
  // Usually used to open a dialog to offer the user a set of configuration
}*/


void LipRec::imageCallback(const sensor_msgs::ImageConstPtr& msg){
	cv::Mat img;

	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		//now cv_ptr is the matrix, do not forget "TYPE_" before "16UC1"
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	img = cv_ptr->image;

	emit updateCam(img);
}

void LipRec::faceROICallback(const sensor_msgs::RegionOfInterestConstPtr& msg){
	faceROITimer.start(timeoutROIdetection);
	faceROI = *msg;
	faceROI_detected = true;
}
void LipRec::mouthROICallback(const sensor_msgs::RegionOfInterestConstPtr& msg){
	mouthROITimer.start(timeoutROIdetection);
	mouthROI = *msg;
	mouthROI_detected = true;
}


void LipRec::getCamPic(cv::Mat img){

	MHI_DURATION = ui_.dsbMHIDuration->value();
	NO_CYCLIC_FRAME = ui_.sbMHIFC->value();
	int currentFrame = 0;

	this->drawFaceMouthROI(img);

    QPixmap pixMap = getPixmap(img);
    ui_.lbl_cam->setPixmap(pixMap);

    Mat mouthImg = this->showLips(img);

	currentFrame = updateFrameBuffer(mouthImg);

	//temporal segmentation
	//1.squared mean difference
	int pixelDifference = 0;
	if(!frameBuffer.empty()){
		for (int i = 0; i < mouthImg.cols; ++i) {
			for (int j = 0; j < mouthImg.rows; ++j) {
				if(frameBuffer[last].cols == 0){
					pixelDifference += 0- frameBuffer[currentFrame].at<uchar>(j,i);
				}else{
					pixelDifference += frameBuffer[last].at<uchar>(j,i) - frameBuffer[currentFrame].at<uchar>(j,i);
				}
			}
		}
	}

	double d = pow(pixelDifference / ((double) mouthImg.cols*mouthImg.rows),2);

	//2. moving average smoothing


	ui_.lcdPixelWiseDiff->display(QString::number(d,'f',2));
	ROS_INFO("difference %f", d);
	//temporal segmentation


	Mat imageAbsDiff = createImageAbsDiff(currentFrame);

	int activation = (int) d;

	switch (stateDetectionStartEndFrame) {
		case Idle:
			utterance.clear();
			utteranceCounter = 0;
			ROS_INFO("Enter state Idle: %d", activation);

			if(activation > 0){

				stateDetectionStartEndFrame = StartFrame;
			}else{

			}

			break;
		case StartFrame:
			ROS_INFO("Enter state StartFrame: %d", activation);

			if(activation < 1){

				stateDetectionStartEndFrame = Utterance;
			}else{

			}
			break;
		case Utterance:
			ROS_INFO("Enter state Utterance: %d", activation);
			if(activation < 1){

				if(utteranceCounter > 5){
					stateDetectionStartEndFrame = Idle;
				}
				utteranceCounter++;

			}else if(activation > 0){
				stateDetectionStartEndFrame = EndFrame;
			}
			break;
		case EndFrame:
			ROS_INFO("Enter state EndFrame: %d", activation);
			if(activation < 1){

				stateDetectionStartEndFrame = Idle;
				utterance.append(imageAbsDiff);
				ROS_INFO("Uterrance %d", utterance.size());

				//1. Generate weighted DOFs
				for (int i = 0; i < utterance.size(); ++i) {

					for (int k = 0; k < utterance.at(i).cols; ++k) {
						for (int j = 0; j < utterance.at(i).rows; ++j) {
							utterance[i].at<uchar>(j,k) = utterance[i].at<uchar>(j,k) * (i+1);
						}
					}

				}

				Size size = Size(imageAbsDiff.size().width, imageAbsDiff.size().height);
				Mat mt = Mat::zeros(size, CV_8UC3);

				//2. take max pixel intensity value
				for (int i = 0; i < utterance.size(); ++i) {

					for (int k = 0; k < utterance.at(i).cols; ++k) {
						for (int j = 0; j < utterance.at(i).rows; ++j) {
							if(mt.at<uchar>(j,k) < utterance[i].at<uchar>(j,k)){
								mt.at<uchar>(j,k) = utterance[i].at<uchar>(j,k);
							}
						}
					}
				}

				QPixmap pixMap = getPixmap(mt);
				pixMap = pixMap.scaled(ui_.lbl_lips->maximumWidth(), ui_.lbl_lips->maximumHeight(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
				ui_.lbl_rec_word->setPixmap(pixMap);

			}else{

			}
			break;
		default:
			break;
	}

	if(stateDetectionStartEndFrame == StartFrame || stateDetectionStartEndFrame == Utterance || stateDetectionStartEndFrame == EndFrame){
		utterance.append(imageAbsDiff);
	}


	if(!imageAbsDiff.empty()){
		this->createMotionHistoryImage(imageAbsDiff);
	}

	last = currentFrame;
}

Mat LipRec::createImageAbsDiff(int currentFrame){
	Mat silh;
	if(frameBuffer.at(last).cols == frameBuffer.at(currentFrame).cols
				&& frameBuffer.at(last).rows == frameBuffer.at(currentFrame).rows){

			absdiff(frameBuffer.at(last), frameBuffer.at(currentFrame), silh);
	}
	return silh;
}


void LipRec::drawFaceMouthROI(Mat& img){
	if(faceROI_detected){
		if(ui_.cbFaceROI->isChecked()){
			drawRectangle(img, faceROI);
		}
	}

	if(mouthROI_detected){
		if(ui_.cbMouthROI->isChecked()){
			drawRectangle(img, mouthROI);
		}
	}
}

Mat LipRec::showLips(Mat& img){
  Mat mouthImg = cutROIfromImage(img, mouthROI);
  QPixmap pixMap;
	if(ui_.cbLips->isChecked()){
		pixMap = getPixmap(mouthImg);
		pixMap = pixMap.scaled(ui_.lbl_lips->maximumWidth(), ui_.lbl_lips->maximumHeight(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
		ui_.lbl_lips->setPixmap(pixMap);
	}else{
		QPixmap empty;
		ui_.lbl_lips->setPixmap(empty);
	}

	return mouthImg;
}


int LipRec::updateFrameBuffer(Mat& img){
	int currentFrame = 0;
	if(frameBuffer.empty()){
		for (int var = 0; var < NO_CYCLIC_FRAME; ++var) {
			frameBuffer.append(img);
		}
	}else{
		currentFrame = (last+1) % NO_CYCLIC_FRAME;
		frameBuffer[currentFrame] = img;
	}

	return currentFrame;
}


void LipRec::createMotionHistoryImage(Mat& imageAbsDiff){
	if(mouthROI_detected && ui_.cbMHI->isChecked()){
		QPixmap pixMap;
		double timestamp = (double) clock()/CLOCKS_PER_SEC;
		Size size = Size(imageAbsDiff.size().width, imageAbsDiff.size().height);
		Mat silh = Mat::zeros(size, CV_8UC3);

		if(mhi.empty() || imageAbsDiff.size != mhi.size){
			mhi.release();
			mhi = Mat(size, CV_32FC1, Scalar(0,0,0));
		}

		pixMap = getPixmap(imageAbsDiff);
		pixMap = pixMap.scaled(ui_.lbl_lips->maximumWidth(), ui_.lbl_lips->maximumHeight(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
		ui_.lbl_rec_phonem->setPixmap(pixMap);

		if(ui_.cbMHIBinarization->isChecked())
			threshold(imageAbsDiff,imageAbsDiff,ui_.dsbMHIThreshold->value(),1,cv::THRESH_BINARY);

		updateMotionHistory(imageAbsDiff, mhi, timestamp, MHI_DURATION);
		Mat mask;
		mhi.convertTo(mask, CV_8UC1, 255.0/MHI_DURATION, (MHI_DURATION-timestamp)*255.0/MHI_DURATION );

		pixMap = getPixmap(mask);
		pixMap = pixMap.scaled(ui_.lbl_lips->maximumWidth(), ui_.lbl_lips->maximumHeight(), Qt::KeepAspectRatio, Qt::SmoothTransformation);

		ui_.lbl_MHI->setPixmap(pixMap);

	}else{
		QPixmap empty;
		ui_.lbl_MHI->setPixmap(empty);
		ui_.lbl_rec_phonem->setPixmap(empty);
		ui_.lbl_rec_word->setPixmap(empty);
	}
}


void LipRec::drawRectangle(Mat& iplImg, sensor_msgs::RegionOfInterest& roi){

	Scalar color;
	if(blackBorder==1){
		color = Scalar(0,0,0);
	}else{
		color = Scalar(255, 255, 255);
	}
	rectangle(iplImg, Point(roi.x_offset, roi.y_offset),
			Point(roi.x_offset + roi.width, roi.y_offset+ roi.height),
			color, 2, 8, 0);
}

Mat LipRec::cutROIfromImage(Mat& src, sensor_msgs::RegionOfInterest& roi){
	Rect mouthROI(roi.x_offset, roi.y_offset, roi.width, roi.height);
	Mat mouth = src(mouthROI).clone();
	return mouth;
}

QPixmap LipRec::getPixmap(Mat& iplImg){
	QPixmap pixMap;
	QImage dest((const uchar *) iplImg.data, iplImg.cols, iplImg.rows, iplImg.step, QImage::Format_Indexed8);
	pixMap.convertFromImage(dest,Qt::ColorOnly);
	return pixMap;
}

void LipRec::faceROItimeout(){
	faceROI_detected = false;
}
void LipRec::mouthROItimeout(){
	mouthROI_detected = false;
}

}
PLUGINLIB_DECLARE_CLASS(rqt_liprec, LipRec, rqt_liprec::LipRec, rqt_gui_cpp::Plugin)
