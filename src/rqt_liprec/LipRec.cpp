#include "rqt_liprec/LipRec.h"
#include <pluginlib/class_list_macros.h>


namespace rqt_liprec {

LipRec::LipRec()
  : rqt_gui_cpp::Plugin()
  , widget_(0)
{
  // Constructor is called first before initPlugin function, needless to say.

  // give QObjects reasonable names
  setObjectName("rqt_liprec");
  showFaceROI = 1;
  showMouthROI = 1;
  blackBorder = 1;
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

	showFaceROI = argv[0].toInt();
	showMouthROI = argv[1].toInt();
	blackBorder = argv[2].toInt();

	ROS_INFO("Show FaceROI: %s",argv[0].toStdString().c_str());
	ROS_INFO("Show MouthROI: %s",argv[1].toStdString().c_str());
	ROS_INFO("BlackBorder: %s",argv[2].toStdString().c_str());

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

	IplImage iplImg = img;

	if(faceROI_detected){
		if(showFaceROI){
			drawRectangle(&iplImg, faceROI);
		}
		faceROI_detected = false;
	}

	if(mouthROI_detected){
		if(showMouthROI){
			drawRectangle(&iplImg, mouthROI);
		}
		mouthROI_detected = false;
	}


    QPixmap pixMap = getPixmap(iplImg);

    ui_.lbl_cam->setPixmap(pixMap);

    IplImage *img2 = cutROIfromImage(iplImg, mouthROI);

    pixMap = getPixmap(*img2);

    pixMap = pixMap.scaled(ui_.lbl_lips->maximumWidth(), ui_.lbl_lips->maximumHeight(), Qt::KeepAspectRatio, Qt::SmoothTransformation);

    ui_.lbl_lips->setPixmap(pixMap);
}


void LipRec::drawRectangle(IplImage* iplImg, sensor_msgs::RegionOfInterest& roi){

	if(blackBorder==1){
		cvRectangle(iplImg, cvPoint(roi.x_offset, roi.y_offset),
				cvPoint(roi.x_offset + roi.width, roi.y_offset+ roi.height),
				CV_RGB(0, 0, 0), 2, 8, 0);
	}else{
		cvRectangle(iplImg, cvPoint(roi.x_offset, roi.y_offset),
				cvPoint(roi.x_offset + roi.width, roi.y_offset+ roi.height),
				CV_RGB(255, 255, 255), 2, 8, 0);
	}
}

IplImage* LipRec::cutROIfromImage(IplImage& src, sensor_msgs::RegionOfInterest& roi){
	cvSetImageROI(&src, cvRect(roi.x_offset, roi.y_offset, roi.width, roi.height));
	IplImage *img = cvCreateImage(cvGetSize(&src),
			src.depth,
			src.nChannels);

	/* copy subimage */
	cvCopy(&src, img, NULL);

	/* always reset the Region of Interest */
	cvResetImageROI(&src);

	return img;
}

QPixmap LipRec::getPixmap(IplImage& iplImg){
	QPixmap pixMap;
	QImage dest((const uchar *) iplImg.imageData, iplImg.width, iplImg.height, QImage::Format_Indexed8);
	dest.bits(); // enforce deep copy, see documentation
	pixMap.convertFromImage(dest,Qt::ColorOnly);

	return pixMap;
}


}
PLUGINLIB_DECLARE_CLASS(rqt_liprec, LipRec, rqt_liprec::LipRec, rqt_gui_cpp::Plugin)
