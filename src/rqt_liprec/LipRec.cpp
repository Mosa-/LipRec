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

	//camImage = getNodeHandle().subscribe("/liprecKinect/rgb/image_raw", 10, &LipRec::imageCallback, this);
	camImage2 = getNodeHandle().subscribe("/kinect2/qhd/image_mono", 100, &LipRec::imageCallback, this);

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

    QObject::connect(ui_.pbUPDP, SIGNAL(clicked()), this, SLOT(clickedUtteranceDiffPlot()));

	faceROITimer.start(timeoutROIdetection);
	mouthROITimer.start(timeoutROIdetection);
    ui_.groupBoxWidget->setShown(false);
    ui_.groupBoxWidget_2->setShown(false);
    ui_.cbSignalWindow1->addItem("None");
    ui_.cbSignalWindow1->addItem("Average");
    ui_.cbSignalWindow2->addItem("None");
    ui_.cbSignalWindow2->addItem("Average");


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
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC1);
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

    QPixmap pixMap = imageProcessing.getPixmap(img);
    ui_.lbl_cam->setPixmap(pixMap);

    Mat mouthImg;
	imageProcessing.cutROIfromImage(img, mouthImg, mouthROI);

	if(ui_.rbGHE->isChecked()){
		imageProcessing.applyHistogramForLightCorrectionGHE(mouthImg);
	}else if(ui_.rbAHE->isChecked()){
		imageProcessing.applyHistogramForLightCorrectionAHE(mouthImg,
				ui_.sbClipLimit->value(), Size(ui_.sbSize->value(),ui_.sbSize->value()));
	}

	BlurType blur;

	if(ui_.rbBlur->isChecked()){
		blur = BLUR;
	}else if(ui_.rbMedian->isChecked()){
		blur = MEDIAN;
	}else if(ui_.rbGaussian->isChecked()){
		blur = GAUSSIAN;
	}

	imageProcessing.applyBlur(mouthImg, ui_.sbMask->value(), blur);

    this->showLips(mouthImg);

    if(mouthImg.cols != 0){
        //imageProcessing.squareImage(mouthImg);
    }

	currentFrame = updateFrameBuffer(mouthImg);
	Mat imageAbsDiff;
	double d = 0;

	if(frameBuffer.at(last).cols == frameBuffer.at(currentFrame).cols
				&& frameBuffer.at(last).rows == frameBuffer.at(currentFrame).rows){

		//temporal segmentation
		d = imageProcessing.generatePixelDifference(frameBuffer[currentFrame], frameBuffer[last]);

		ui_.lcdPixelWiseDiff->display(QString::number(d,'f',0));

		imageAbsDiff = imageProcessing.createImageAbsDiff(frameBuffer[currentFrame], frameBuffer[last]);
	}

	//temporal segmentation
	int activation = QString::number(d,'f',0).toInt();

	this->changeLipActivationState(activation, imageAbsDiff);

	if(!imageAbsDiff.empty()){
		pixMap = imageProcessing.getPixmap(imageAbsDiff);
		pixMap = pixMap.scaled(ui_.lbl_lips->maximumWidth(), ui_.lbl_lips->maximumHeight(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
		ui_.lbl_rec_phonem->setPixmap(pixMap);

		if(mouthROI_detected && ui_.cbMHI->isChecked()){

			Mat mask = imageProcessing.createMotionHistoryImage(imageAbsDiff, mhi, ui_.cbMHIBinarization->isChecked(),
				ui_.dsbMHIThreshold->value(), MHI_DURATION);

			pixMap = imageProcessing.getPixmap(mask);
			pixMap = pixMap.scaled(ui_.lbl_lips->maximumWidth(), ui_.lbl_lips->maximumHeight(), Qt::KeepAspectRatio, Qt::SmoothTransformation);

			ui_.lbl_MHI->setPixmap(pixMap);
		}else{
			QPixmap empty;
			ui_.lbl_MHI->setPixmap(empty);
			ui_.lbl_rec_phonem->setPixmap(empty);
			ui_.lbl_rec_word->setPixmap(empty);
		}
	}

	last = currentFrame;
}

void LipRec::drawFaceMouthROI(Mat& img){
	Scalar color;
	if(blackBorder==1){
		color = Scalar(0,0,0);
	}else{
		color = Scalar(255, 255, 255);
	}

	if(faceROI_detected){
		if(ui_.cbFaceROI->isChecked()){
			imageProcessing.drawRectangle(img, faceROI, color);
		}
	}

	if(mouthROI_detected){
		if(ui_.cbMouthROI->isChecked()){
			imageProcessing.drawRectangle(img, mouthROI, color);
		}
	}
}

void LipRec::showLips(Mat& mouthImg){
	QPixmap pixMap;
	if(ui_.cbLips->isChecked()){
		pixMap = imageProcessing.getPixmap(mouthImg);
		pixMap = pixMap.scaled(ui_.lbl_lips->maximumWidth(), ui_.lbl_lips->maximumHeight(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
		ui_.lbl_lips->setPixmap(pixMap);
	}else{
		QPixmap empty;
		ui_.lbl_lips->setPixmap(empty);
	}
}

int LipRec::updateFrameBuffer(Mat img){
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

void LipRec::changeLipActivationState(int activation, Mat& imageAbsDiff){
	QPixmap pixMap;
	switch (stateDetectionStartEndFrame) {
		case Idle:
			utterance.clear();
            utterancePixelDiff.clear();
			silenceCounter = 0;

			if(activation > ui_.sbST->value()){
				stateDetectionStartEndFrame = Utterance;
                utterancePixelDiff.append(activation);
			}else{

			}

			break;
		case Utterance:
			if(activation <= ui_.sbST->value() && silenceCounter == ui_.sbNoSF->value()){

				stateDetectionStartEndFrame = Idle;
				Mat uLast = imageAbsDiff;
				if(!utterance.isEmpty()){
					uLast = utterance.last();
				}

				// only add imageAbsDiff if the last DOF-image has the same size
				if(uLast.cols == imageAbsDiff.cols && uLast.rows == imageAbsDiff.rows){
					utterance.append(imageAbsDiff);
                    utterancePixelDiff.append(activation);
				}
				ui_.lcdUtterance->display(QString::number(utterance.size(),'f',0));

				//1. Generate weighted DOFs
				for (int i = 0; i < utterance.size(); ++i) {

					if(!utterance.at(i).empty()){
						for (int k = 0; k < utterance.at(i).cols; ++k) {
							for (int j = 0; j < utterance.at(i).rows; ++j) {
								utterance[i].at<uchar>(j,k) = utterance[i].at<uchar>(j,k) * (ui_.sbDOFboost->value());
								if(utterance[i].at<uchar>(j,k) > 255){
									utterance[i].at<uchar>(j,k) = 255;
								}

							}
						}
					}
				}

				Size size = Size(imageAbsDiff.size().width, imageAbsDiff.size().height);
				Mat mt(imageAbsDiff.rows, imageAbsDiff.cols, CV_8UC1, Scalar(0));


				//2. take max pixel intensity value
				for (int i = 0; i < utterance.size(); ++i) {

					if(!utterance.at(i).empty()){

						for (int k = 0; k < utterance.at(i).cols; ++k) {
							for (int j = 0; j < utterance.at(i).rows; ++j) {
								if(mt.at<uchar>(j,k) < utterance[i].at<uchar>(j,k)){
									mt.at<uchar>(j,k) = utterance[i].at<uchar>(j,k);
								}
							}
						}
					}
                }

                imageProcessing.squareImage(mt);


                if(ui_.cbSWT->isChecked()){
                    Mat ca = Mat::zeros(mt.rows, mt.cols, CV_8UC1);
                    Mat ch = Mat::zeros(mt.rows, mt.cols, CV_8UC1);
                    Mat cd = Mat::zeros(mt.rows, mt.cols, CV_8UC1);
                    Mat cv = Mat::zeros(mt.rows, mt.cols, CV_8UC1);

                    swt.applySwt(mt, ca, ch, cd, cv, 1, Swt::Haar);

                    pixMap = imageProcessing.getPixmap(ca);
                }else{
                    pixMap = imageProcessing.getPixmap(mt);
                }
				pixMap = pixMap.scaled(ui_.lbl_lips->maximumWidth(), ui_.lbl_lips->maximumHeight(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
				ui_.lbl_rec_word->setPixmap(pixMap);


                QString currentTextSignalWindow1 = ui_.cbSignalWindow1->currentText();
                QString currentTextSignalWindow2 = ui_.cbSignalWindow2->currentText();

                if(currentTextSignalWindow1 == "None"){
                    this->applySignalSmoothing(1, S_NONE);
                }else if(currentTextSignalWindow1 == "Average"){
                    this->applySignalSmoothing(1, AVERAGE);
                }else{

                }

                if(currentTextSignalWindow2 == "None"){
                    this->applySignalSmoothing(2, S_NONE);
                }else if(currentTextSignalWindow2 == "Average"){
                    this->applySignalSmoothing(2, AVERAGE);
                }else{

                }


//                QVector<QPointF> points;

//                this->averageSignalSmoothing();


//                ROS_INFO("##########################################1");
//                ROS_INFO("%d", utterancePixelDiff.size());
//                for (int i = 0; i < utterancePixelDiff.size(); ++i) {
//                    points.append(QPoint(i, -utterancePixelDiff.at(i)));
//                    ROS_INFO("%d", utterancePixelDiff.at(i));
//                }
//                QGraphicsScene * scene = new QGraphicsScene();
//                QPolygonF plyline;
//                QPainterPath myPath;
//                ui_.gvSignalWindow1->setScene(scene);
//                double rad = 1;
//                for (int i = 0; i < points.size(); ++i) {
//                    plyline.append(points[i]);
//                    myPath.addPolygon(plyline);
//                    scene->addEllipse(points[i].x(), points[i].y(), rad*0.3, rad*0.3, QPen(), QBrush(Qt::red, Qt::SolidPattern));
//                }
//                scene->addPath(myPath);
//                ui_.gvSignalWindow1->fitInView(scene->sceneRect(), Qt::KeepAspectRatio);

//                ROS_INFO("##########################################2");

			}else if(activation <= ui_.sbST->value()){
				silenceCounter++;
                utterancePixelDiff.append(activation);
			}else{
				silenceCounter = 0;
				Mat uLast = imageAbsDiff;
				if(!utterance.isEmpty()){
					uLast = utterance.last();
				}

				if(uLast.cols == imageAbsDiff.cols && uLast.rows == imageAbsDiff.rows){
					utterance.append(imageAbsDiff);
                    utterancePixelDiff.append(activation);
				}
			}
			break;
		default:
			break;
    }
}

void LipRec::applySignalSmoothing(int graphicView, SignalSmoothingType type)
{
    QGraphicsView* gv;
    if(graphicView == 1){
        gv = ui_.gvSignalWindow1;
    }else{
        gv = ui_.gvSignalWindow2;
    }

    QList<int> upd = utterancePixelDiff;

    switch (type) {
    case S_NONE:
        break;
    case AVERAGE:
        this->averageSignalSmoothing(upd);
        break;
    default:
        break;
    }

    QVector<QPointF> points;

    for (int i = 0; i < upd.size(); ++i) {
        points.append(QPoint(i, -upd.at(i)));
    }
    QGraphicsScene * scene = new QGraphicsScene();
    QPolygonF plyline;
    QPainterPath myPath;
    gv->setScene(scene);
    double rad = 1;
    for (int i = 0; i < points.size(); ++i) {
        plyline.append(points[i]);
        myPath.addPolygon(plyline);
        scene->addEllipse(points[i].x(), points[i].y(), rad*0.3, rad*0.3, QPen(), QBrush(Qt::red, Qt::SolidPattern));
    }
    scene->addPath(myPath);
    gv->fitInView(scene->sceneRect(), Qt::KeepAspectRatio);


}

void LipRec::averageSignalSmoothing(QList<int>& signalsSmoothing){

    for (int i = 0; i < signalsSmoothing.size(); ++i) {
        if(i > 0 && i < signalsSmoothing.size()-1){
            signalsSmoothing[i] = (signalsSmoothing[i-1] + signalsSmoothing[i] + signalsSmoothing[i+1]) / 3;
        }
    }
}

void LipRec::faceROItimeout(){
	faceROI_detected = false;
}
void LipRec::mouthROItimeout(){
    mouthROI_detected = false;
}

void LipRec::clickedUtteranceDiffPlot(){
    if(ui_.groupBoxWidget->isHidden()){
        ui_.groupBoxWidget->setShown(true);
        ui_.groupBoxWidget_2->setShown(true);

    }else{
        ui_.groupBoxWidget->setShown(false);
        ui_.groupBoxWidget_2->setShown(false);


    }
}

void LipRec::printMat(Mat& data){
	ROS_INFO("<1==============printMat================1>");
	for (int i = 0; i < data.cols; ++i) {
		for (int j = 0; j < data.rows; ++j) {
				ROS_INFO("Pixelvalue(%d,%d): %d", j,i,data.at<uchar>(j,i));
		}
	}
	ROS_INFO("<2==============printMat================2>");

}

}
PLUGINLIB_DECLARE_CLASS(rqt_liprec, LipRec, rqt_liprec::LipRec, rqt_gui_cpp::Plugin)
