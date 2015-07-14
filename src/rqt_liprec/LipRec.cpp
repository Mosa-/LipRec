#include "rqt_liprec/LipRec.h"
#include <pluginlib/class_list_macros.h>


namespace rqt_liprec {

LipRec::LipRec()
  : rqt_gui_cpp::Plugin()
  , last(0), blackBorder(0), widget_(0)
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
	faceROI = *msg;
	faceROI_detected = true;
}
void LipRec::mouthROICallback(const sensor_msgs::RegionOfInterestConstPtr& msg){
	mouthROI = *msg;
	mouthROI_detected = true;
}


void LipRec::getCamPic(cv::Mat img){

	MHI_DURATION = ui_.dsbMHIDuration->value();
	NO_CYCLIC_FRAME = ui_.sbMHIFC->value();

	this->drawFaceMouthROI(img);

    QPixmap pixMap = getPixmap(img);
    ui_.lbl_cam->setPixmap(pixMap);

    Mat mouthImg = this->showLips(img);

    this->createMotionHistoryImage(mouthImg);

//	faceROI_detected = false;
//	mouthROI_detected = false;
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

void LipRec::createMotionHistoryImage(Mat& img){
	if(mouthROI_detected && ui_.cbMHI->isChecked()){
		QPixmap pixMap;
		double timestamp = (double) clock()/CLOCKS_PER_SEC;
		Size size = Size(img.size().width, img.size().height);
		int i, idx1 = last, idx2;
		Mat silh = Mat::zeros(size, CV_8UC3);

		if(mhi.empty()){
			for (int var = 0; var < NO_CYCLIC_FRAME; ++var) {
				Mat tmp = Mat::zeros(size, CV_8UC3);
				frameBuffer.append(img);
			}
			mhi.release();
			mhi = Mat(size, CV_32FC1, Scalar(0,0,0));
		}

		frameBuffer[last] = img;

		idx2 = (last+1) % NO_CYCLIC_FRAME;
		last = idx2;

		absdiff(frameBuffer.at(idx1), frameBuffer.at(idx2), silh);

		updateMotionHistory(silh, mhi, timestamp, MHI_DURATION);
		pixMap = getPixmap(silh);

		pixMap = pixMap.scaled(ui_.lbl_lips->maximumWidth(), ui_.lbl_lips->maximumHeight(), Qt::KeepAspectRatio, Qt::SmoothTransformation);

		ui_.lbl_MHI->setPixmap(pixMap);
	}else{
		QPixmap empty;
		ui_.lbl_MHI->setPixmap(empty);
	}
}


void LipRec::drawRectangle(Mat& iplImg, sensor_msgs::RegionOfInterest& roi){

	if(blackBorder==1){
		rectangle(iplImg, Point(roi.x_offset, roi.y_offset),
				Point(roi.x_offset + roi.width, roi.y_offset+ roi.height),
				Scalar(0,0,0),2,8,0 );

	}else{
		rectangle(iplImg, Point(roi.x_offset, roi.y_offset),
				Point(roi.x_offset + roi.width, roi.y_offset+ roi.height),
				Scalar(255, 255, 255), 2, 8, 0);
	}
}

Mat LipRec::cutROIfromImage(Mat& src, sensor_msgs::RegionOfInterest& roi){
	Rect mouthROI(roi.x_offset, roi.y_offset, roi.width, roi.height);
	Mat mouth = src(mouthROI).clone();
	return mouth;
}

QPixmap LipRec::getPixmap(cv::Mat& iplImg){
	QPixmap pixMap;
	QImage dest((const uchar *) iplImg.data, iplImg.cols, iplImg.rows, iplImg.step, QImage::Format_Indexed8);
	pixMap.convertFromImage(dest,Qt::ColorOnly);
	return pixMap;
}

}
PLUGINLIB_DECLARE_CLASS(rqt_liprec, LipRec, rqt_liprec::LipRec, rqt_gui_cpp::Plugin)
