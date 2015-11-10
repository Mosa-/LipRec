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

    if(argv.size() != 4){
        ROS_INFO("Four arguments necessary!");
    }else{
        int showFaceROI = argv[0].toInt();
        int showMouthROI = argv[1].toInt();
        blackBorder = argv[2].toInt();
        if(showFaceROI == 1)
            ui_.cbFaceROI->setChecked(true);

        if(showMouthROI == 1)
            ui_.cbMouthROI->setChecked(true);

        useMonoImage = argv[3].toInt();

        ROS_INFO("Show FaceROI: %s",argv[0].toStdString().c_str());
        ROS_INFO("Show MouthROI: %s",argv[1].toStdString().c_str());
        ROS_INFO("BlackBorder: %s",argv[2].toStdString().c_str());
        ROS_INFO("Use mono image: %s",argv[3].toStdString().c_str());
    }

    string kinectTopic;
    //camImage = getNodeHandle().subscribe("/liprecKinect/rgb/image_raw", 10, &LipRec::imageCallback, this);
    if(useMonoImage){
        kinectTopic = "/kinect2/qhd/image_mono";
    }else{
        kinectTopic = "/kinect2/qhd/image_color";
    }
    this->imageProcessing.setUseMonoImage(useMonoImage);

    camImage = getNodeHandle().subscribe(kinectTopic, 100, &LipRec::imageCallback, this);

    faceROISub = getNodeHandle().subscribe("/face_detection/faceROI", 10, &LipRec::faceROICallback, this);
    mouthROISub = getNodeHandle().subscribe("/face_detection/mouthROI", 10, &LipRec::mouthROICallback, this);

    NO_CYCLIC_FRAME = ui_.sbNOCF->value();

    QObject::connect(&faceROITimer, SIGNAL(timeout()), this, SLOT(faceROItimeout()));
    QObject::connect(&mouthROITimer, SIGNAL(timeout()), this, SLOT(mouthROItimeout()));

    QObject::connect(ui_.pbUPDP, SIGNAL(clicked()), this, SLOT(clickedUtteranceDiffPlot()));
    QObject::connect(ui_.pbContinueVideo, SIGNAL(clicked()), this, SLOT(clickedContinueVideo()));

    QObject::connect(ui_.pbToggleKpLines, SIGNAL(clicked(bool)), this, SLOT(toggleKpLines(bool)));
    drawKeyPointLines = false;

    QObject::connect(ui_.pbToggleSupportLines, SIGNAL(clicked(bool)), this, SLOT(toggleSupportLines(bool)));
    drawSupportLines = false;

    faceROITimer.start(timeoutROIdetection);
    mouthROITimer.start(timeoutROIdetection);
    ui_.groupBoxWidget->setShown(false);
    ui_.groupBoxWidget_2->setShown(false);
    ui_.cbSignalWindow1->addItem("None");
    ui_.cbSignalWindow1->addItem("Average");
    ui_.cbSignalWindow2->addItem("None");
    ui_.cbSignalWindow2->addItem("Average");

    QMenuBar* menuBar = new QMenuBar(widget_);

    QWidgetAction *widgetAction=new QWidgetAction(widget_);
    le = new QLineEdit(widget_);
    widgetAction->setDefaultWidget(le);
    QMenu *fm= menuBar->addMenu("File");
    fm->addAction(widgetAction);

    QMenu* video = new QMenu("Video");
    video->addAction("Record");
    video->addAction("Stop Record");
    video->addSeparator();
    video->addAction("Record Utterance");

    QWidgetAction *waGroupBox =new QWidgetAction(widget_);
    checkboxOnlyMouth = new QCheckBox("Only Mouth");
    waGroupBox->setDefaultWidget(checkboxOnlyMouth);
    video->addAction(waGroupBox);

    video->addAction("Load Utterance");
    menuBar->addMenu(video);

    connect(menuBar, SIGNAL(triggered(QAction*)), this, SLOT(triggedAction(QAction*)));

    ui_.pbUPDP->setToolTip("Plot the pixel difference of an utterance.");
    QPixmap pixmap("src/liprec/res/plot3.png");
    QIcon bi(pixmap);
    ui_.pbUPDP->setSizePolicy( QSizePolicy::Fixed, QSizePolicy::Preferred );
    ui_.pbUPDP->setFlat(true);
    ui_.pbUPDP->setIcon(bi);
    ui_.pbUPDP->setIconSize(pixmap.rect().size());
    ui_.pbUPDP->setMaximumSize(pixmap.rect().size());

    ui_.pbContinueVideo->setToolTip("Stop video stream from camera.");
    pixmap = QPixmap("src/liprec/res/videostop1.png");
    bi = QIcon(pixmap);
    ui_.pbContinueVideo->setSizePolicy( QSizePolicy::Fixed, QSizePolicy::Preferred );
    ui_.pbContinueVideo->setFlat(true);
    ui_.pbContinueVideo->setIcon(bi);
    ui_.pbContinueVideo->setIconSize(pixmap.rect().size()/2.5);
    ui_.pbContinueVideo->setMaximumSize(pixmap.rect().size()/2.5);

    recordVideo = false;
    recordUtterance = false;
    loadUtterance = false;
    initVideoWriter = false;
    useCam = true;

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
        if(useMonoImage){
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC1);
        }else{
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
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

bool yCoordLessThanPossibleKeyPoint(LipRec::PossibleKeyPoint a, LipRec::PossibleKeyPoint b)
{
    return a.keyPoint.y > b.keyPoint.y;
}

void LipRec::getCamPic(cv::Mat img){
    if(!this->loadUtterance && useCam && !img.empty()){
        this->processImage(img);
    }
}

void LipRec::processImage(Mat img)
{
    currentUtteranceFrame = img.clone();

    if(recordVideo && !recordUtterance){
        if(!le->text().isEmpty()){
            if(!initVideoWriter){
                imageProcessing.setupVideoWriter(le->text()+".avi", img.cols, img.rows);
                initVideoWriter = true;
            }

            imageProcessing.writeFrameToVideo(img);
        }else{
            recordVideo = false;
        }
    }else if(!recordUtterance){
        initVideoWriter = false;
        imageProcessing.closeVideoWriter();
    }

    NO_CYCLIC_FRAME = ui_.sbNOCF->value();
    int currentFrame = 0;

    this->drawFaceMouthROI(img);

    QPixmap pixMap = imageProcessing.getPixmap(img, useMonoImage);
    ui_.lbl_cam->setPixmap(pixMap);

    Mat mouthImg;
    imageProcessing.cutROIfromImage(img, mouthImg, mouthROI);

    if(useMonoImage){
        if(ui_.rbGHE->isChecked()){
            imageProcessing.applyHistogramForLightCorrectionGHE(mouthImg);
        }else if(ui_.rbAHE->isChecked()){
            imageProcessing.applyHistogramForLightCorrectionAHE(mouthImg,
                                                                ui_.sbClipLimit->value(), Size(ui_.sbSize->value(),ui_.sbSize->value()));
        }
    }else{
        if(ui_.rbGHE->isChecked()){
            mouthImg = calcColorHistogramEqualization(mouthImg);
        }
    }

    BlurType blur = B_NONE;

    if(ui_.rbBlur->isChecked()){
        blur = BLUR;
    }else if(ui_.rbMedian->isChecked()){
        blur = MEDIAN;
    }else if(ui_.rbGaussian->isChecked()){
        blur = GAUSSIAN;
    }

    imageProcessing.applyBlur(mouthImg, ui_.sbMask->value(), blur);

    //IDEE schwarze pixel weg nehmen

    //    if(!mouthImg.empty()){
    //        cvtColor(mouthImg, mouthImg, CV_BGR2HSV);
    //        for (int i = 0; i < mouthImg.rows; ++i) {
    //            for (int j = 0; j < mouthImg.cols; ++j) {
    //                mouthImg.at<Vec3b>(i,j)[1] = mouthImg.at<Vec3b>(i,j)[1] + (mouthImg.at<Vec3b>(i,j)[1]*0.6);
    //            }
    //        }
    //        cvtColor(mouthImg, mouthImg, CV_HSV2BGR);
    //    }

    Mat rTop(mouthImg.rows, mouthImg.cols, CV_8UC1);
    Mat rMid(mouthImg.rows, mouthImg.cols, CV_8UC1);
    Mat rLow(mouthImg.rows, mouthImg.cols, CV_8UC1);
    Mat rTopFinal(mouthImg.rows, mouthImg.cols, CV_8UC1);
    Mat rMidFinal(mouthImg.rows, mouthImg.cols, CV_8UC1);
    Mat rLowFinal(mouthImg.rows, mouthImg.cols, CV_8UC1);

    Point keyPoint1, keyPoint2, keyPoint3, keyPoint4, keyPoint5, keyPoint6;
    ///        P2         P4
    ///       --X-- P3  --X--
    ///      /     \_X_/     \
    ///     /                 \
    /// P1 X-------------------X P5
    ///     \                 /
    ///      \               /
    ///       \______X______/
    ///              P6


    if(!useMonoImage){
        if(ui_.cbLipSeg->isChecked()){
            if(ui_.rbSaturation->isChecked()){
                this->applyLipsSegmentationSaturation(mouthImg);
            }

            if(ui_.rbPseudoHue->isChecked()){
                for (int i = 0; i < mouthImg.rows; ++i) {
                    for (int j = 0; j < mouthImg.cols; ++j) {
                        rTop.at<uchar>(i,j) = (int) (pseudoHuePxl(mouthImg, j, i) - luminancePxl(mouthImg, j, i));
                        rMid.at<uchar>(i,j) = (int) (pseudoHuePxl(mouthImg, j, i) - luminancePxl(mouthImg, j, i));
                        rLow.at<uchar>(i,j) = (int) (pseudoHuePxl(mouthImg, j, i) + luminancePxl(mouthImg, j, i));
                    }
                }

                double minVal, maxVal;

                Mat rTopTemp;
                //Sobel(rTop, rTopTemp, CV_32FC1, 0, 1);
                Scharr(rTop, rTopTemp, CV_32FC1, 0, 1);
                minMaxLoc(rTopTemp, &minVal, &maxVal);
                rTopTemp.convertTo(rTopFinal, CV_8UC1, 255.0/(maxVal - minVal), -minVal * 255.0/(maxVal - minVal));

                Mat rMidTemp;
                Sobel(rMid, rMidTemp, CV_32FC1, 0, 1);
                minMaxLoc(rMidTemp, &minVal, &maxVal);
                rMidTemp.convertTo(rMidFinal, CV_8UC1, 255.0/(maxVal - minVal), -minVal * 255.0/(maxVal - minVal));

                for (int i = 0; i < mouthImg.rows; ++i) {
                    for (int j = 0; j < mouthImg.cols; ++j) {
                        rMidFinal.at<uchar>(i,j) = (int) (rMidFinal.at<uchar>(i,j) * pseudoHuePxl(mouthImg, j, i));
                    }
                }

                Mat rLowTemp;
                //Sobel(rLow, rLowTemp, CV_32FC1, 0, 1);
                Scharr(rLow, rLowTemp, CV_32FC1, 0, 1);
                minMaxLoc(rLowTemp, &minVal, &maxVal);
                rLowTemp.convertTo(rLowFinal, CV_8UC1, 255.0/(maxVal - minVal), -minVal * 255.0/(maxVal - minVal));
            }
        }

        cv::Point leftLinePoint(0, mouthROI.height/2);
        cv::Point rightLinePoint(mouthROI.width, mouthROI.height/2);

        cv::Point upLinePoint(mouthROI.width/2, 0);
        cv::Point bottomLinePoint(mouthROI.width/2, mouthROI.height);


        this->extractMouthCornerKeyPoints(rMidFinal, mouthImg, keyPoint1, keyPoint5, 24, 10);
        this->extractCupidsBowKeyPoints(rTopFinal, keyPoint2, keyPoint3, keyPoint4, keyPoint1, 25, 10);
        this->extractLowerLipKeyPoint(rLowFinal, keyPoint6, keyPoint2.x, keyPoint4.x, keyPoint1, 40, 16);

        if(ui_.rbLipsNone->isChecked()){
            if(drawSupportLines){
                line(mouthImg, keyPoint1, rightLinePoint, cv::Scalar(255,255,255), 1);
            }

            if(drawKeyPointLines){
//                line(mouthImg, keyPoint1, keyPoint2, cv::Scalar(255,255,255), 1);
//                line(mouthImg, keyPoint2, keyPoint3, cv::Scalar(255,255,255), 1);
//                line(mouthImg, keyPoint3, keyPoint4, cv::Scalar(255,255,255), 1);
//                line(mouthImg, keyPoint4, keyPoint5, cv::Scalar(255,255,255), 1);
//                line(mouthImg, keyPoint5, keyPoint6, cv::Scalar(255,255,255), 1);
//                line(mouthImg, keyPoint6, keyPoint1, cv::Scalar(255,255,255), 1);

                vector<Point> points(6);
                points.push_back(keyPoint1);
                points.push_back(keyPoint2);
                points.push_back(keyPoint3);
                points.push_back(keyPoint4);
                points.push_back(keyPoint5);
                points.push_back(keyPoint6);
                vector<Point> approxCurve;
                double epsilon = 5;
                approxPolyDP(points, approxCurve, epsilon, true);

                  for( int i = 0; i< points.size(); i++ )
              {
                  Scalar color = Scalar(255,255,255);
                  drawContours( mouthImg, approxCurve, i, color, 1, 8 );
              }
            }

            circle(mouthImg, keyPoint6, 1, Scalar(255,255,255));

            circle(mouthImg, keyPoint1, 1, Scalar(255,255,255));
            circle(mouthImg, keyPoint5, 1, Scalar(255,255,255));

            circle(mouthImg, keyPoint2, 1, Scalar(255,255,255));
            circle(mouthImg, keyPoint4, 1, Scalar(255,255,255));
            circle(mouthImg, keyPoint3, 1, Scalar(255,255,255));

            this->showLips(mouthImg);
        }else if(ui_.rbRLow->isChecked()){

            circle(rLowFinal, keyPoint6, 1, Scalar(255,255,255));

            if(drawSupportLines){
                line(rLowFinal, upLinePoint, bottomLinePoint, Scalar(255,255,255));
                line(rLowFinal, keyPoint1, rightLinePoint, cv::Scalar(255,255,255), 1);
            }
            this->showLips(rLowFinal, true);
        }else if(ui_.rbRMid->isChecked()){

            circle(rMidFinal, keyPoint1, 1, Scalar(255,255,255));
            circle(rMidFinal, keyPoint5, 1, Scalar(255,255,255));

            if(drawSupportLines){
                line(rMidFinal, keyPoint1, rightLinePoint, cv::Scalar(255,255,255), 1);
            }
            this->showLips(rMidFinal, true);
        }else{
            circle(rTopFinal, keyPoint2, 1, Scalar(255,255,255));
            circle(rTopFinal, keyPoint4, 1, Scalar(255,255,255));
            circle(rTopFinal, keyPoint3, 1, Scalar(255,255,255));

            if(drawSupportLines){
                line(rTopFinal, upLinePoint, bottomLinePoint, Scalar(255,255,255));
                line(rTopFinal, keyPoint1, rightLinePoint, Scalar(255,255,255));
            }

            this->showLips(rTopFinal, true);

            ///Show all possible keypoints
            //            if(!finalPossibleKeyPoints.empty()){
            //                for (int i = 0; i < finalPossibleKeyPoints.size(); ++i) {
            //                    circle(rTopFinal, finalPossibleKeyPoints.at(i).keyPoint, 1, Scalar(255,255,255));
            //                }
            //            }
        }


    }else{
        this->showLips(mouthImg, true);
    }



    //    if(mouthImg.cols != 0){
    //        //imageProcessing.squareImage(mouthImg);
    //    }

    //    currentFrame = updateFrameBuffer(mouthImg);
    //    Mat imageAbsDiff;
    //    double d = 0;

    //    if(frameBuffer.at(last).cols == frameBuffer.at(currentFrame).cols
    //                && frameBuffer.at(last).rows == frameBuffer.at(currentFrame).rows){

    //        //temporal segmentation
    //        d = imageProcessing.generatePixelDifference(frameBuffer[currentFrame], frameBuffer[last]);

    //        ui_.lcdPixelWiseDiff->display(QString::number(d,'f',0));

    //        imageAbsDiff = imageProcessing.createImageAbsDiff(frameBuffer[currentFrame], frameBuffer[last]);
    //    }

    //    //temporal segmentation
    //    int activation = QString::number(d,'f',0).toInt();

    //    this->changeLipActivationState(activation, imageAbsDiff, currentFrame);

    //    if(!imageAbsDiff.empty()){
    //        pixMap = imageProcessing.getPixmap(imageAbsDiff);
    //        pixMap = pixMap.scaled(ui_.lbl_lips->maximumWidth(), ui_.lbl_lips->maximumHeight(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
    //        ui_.lbl_rec_phonem->setPixmap(pixMap);

    //        if(mouthROI_detected && ui_.cbMHI->isChecked()){

    //            Mat mask = imageProcessing.createMotionHistoryImage(imageAbsDiff, mhi, ui_.cbMHIBinarization->isChecked(),
    //                ui_.dsbMHIThreshold->value(), MHI_DURATION);

    //            pixMap = imageProcessing.getPixmap(mask);
    //            pixMap = pixMap.scaled(ui_.lbl_lips->maximumWidth(), ui_.lbl_lips->maximumHeight(), Qt::KeepAspectRatio, Qt::SmoothTransformation);

    //            ui_.lbl_MHI->setPixmap(pixMap);
    //        }else{
    //            QPixmap empty;
    //            ui_.lbl_MHI->setPixmap(empty);
    //            ui_.lbl_rec_phonem->setPixmap(empty);
    //            ui_.lbl_rec_word->setPixmap(empty);
    //        }
    //    }

    last = currentFrame;
}

void LipRec::extractMouthCornerKeyPoints(Mat &rMidFinal, Mat& mouthImg, Point &keyPoint1, Point &keyPoint5, int thresholdDifferenceToAvg, int totalLineCheck)
{
    QList<PossibleKeyPoint> possibleKeyPoints;
    PossibleKeyPoint possibleKeyPoint;

    for (int i = 0; i < rMidFinal.cols; ++i) {
        for (int j = rMidFinal.rows-totalLineCheck; j > totalLineCheck/2; --j) {

            int currentDiffToAvg = 0;

            for (int k = 1; k < totalLineCheck/2 + 1; ++k) {
                currentDiffToAvg += rMidFinal.at<uchar>(j-k,i) + rMidFinal.at<uchar>(j+k,i);

            }
            currentDiffToAvg = currentDiffToAvg / totalLineCheck;

            if(currentDiffToAvg > 0){
                currentDiffToAvg = 100 - (rMidFinal.at<uchar>(j,i) * 100 / currentDiffToAvg);
            }

            if(currentDiffToAvg > thresholdDifferenceToAvg){
                possibleKeyPoint.differenceToAvg = currentDiffToAvg;
                possibleKeyPoint.keyPoint.x  = i;
                possibleKeyPoint.keyPoint.y  = j;
                possibleKeyPoints.append(possibleKeyPoint);
            }
        }
    }

    Mat contourImg(rMidFinal.rows, rMidFinal.cols, CV_8UC1, Scalar(0,0,0));
    Point p;
    for (int i = 0; i < possibleKeyPoints.size(); ++i) {
        p = possibleKeyPoints.at(i).keyPoint;
        contourImg.at<uchar>(p.y, p.x) = 255;
    }
    Mat _img;
    double otsu_thresh_val = cv::threshold(
                contourImg, _img, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU
                );

    Canny(contourImg, contourImg, otsu_thresh_val*0.5, otsu_thresh_val, 3, true);

    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours( contourImg, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
    for( int i = 0; i< contours.size(); i++ )
    {
        drawContours( contourImg, contours, i, Scalar(255,255,255), 1, 8, hierarchy, 1, Point() );
    }

    double luminanceMean = 0.0;
    double pseudoHueMean = 0.0;
    for (int i = 0; i < mouthImg.cols/2; ++i) {
        for (int j = 0; j < mouthImg.rows; ++j) {
            luminanceMean += luminancePxl(mouthImg, i, j);
            pseudoHueMean += pseudoHuePxl(mouthImg, i, j);
        }
    }
    luminanceMean /= (mouthImg.cols/2*mouthImg.rows);
    pseudoHueMean /= (mouthImg.cols/2*mouthImg.rows);

    QList<PossibleKeyPoint> pKPoints;
    PossibleKeyPoint pKPoint;

    for (int i = mouthImg.cols/2-(mouthImg.cols/2*0.4); i > 0; --i) {
        for (int j = mouthImg.rows; j > 0; --j) {
            if(contourImg.at<uchar>(j,i) == 255
                    && pseudoHuePxl(mouthImg, i, j) > pseudoHueMean && luminancePxl(mouthImg, i, j) < luminanceMean){
                pKPoint.keyPoint.x = i;
                pKPoint.keyPoint.y = j;
                pKPoints.append(pKPoint);
                break;
            }
        }
    }


    keyPoint1.x = 1000;
    for (int i = 0; i < pKPoints.size(); ++i) {
        int diffY = 0;
        if(i > 0){
            diffY = abs(pKPoints.at(i).keyPoint.y - pKPoints.at(i-1).keyPoint.y);
            //ROS_INFO("diff: %d", abs(pKPoints.at(i).keyPoint.y - pKPoints.at(i-1).keyPoint.y));
        }

        if(diffY > 3){
            break;
        }

        if(keyPoint1.x > pKPoints.at(i).keyPoint.x){
            keyPoint1.x = pKPoints.at(i).keyPoint.x;
            keyPoint1.y = pKPoints.at(i).keyPoint.y;
        }
        //circle(rMidFinal, pKPoints.at(i).keyPoint, 2, Scalar(255,255,255));
    }


    luminanceMean = 0.0;
    pseudoHueMean = 0.0;
    for (int i = mouthImg.cols/2; i < mouthImg.cols; ++i) {
        for (int j = 0; j < mouthImg.rows; ++j) {
            luminanceMean += luminancePxl(mouthImg, i, j);
            pseudoHueMean += pseudoHuePxl(mouthImg, i, j);
        }
    }
    luminanceMean /= (mouthImg.cols/2*mouthImg.rows);
    pseudoHueMean /= (mouthImg.cols/2*mouthImg.rows);

    pKPoints.clear();

    for (int i = mouthImg.cols/2+(mouthImg.cols/2*0.2); i < mouthImg.cols; ++i) {
        for (int j = mouthImg.rows; j > 0; --j) {
            if(contourImg.at<uchar>(j,i) == 255
                    && pseudoHuePxl(mouthImg, i, j) > pseudoHueMean  && luminancePxl(mouthImg, i, j) < luminanceMean){
                pKPoint.keyPoint.x = i;
                pKPoint.keyPoint.y = j;
                pKPoints.append(pKPoint);
                break;
            }
        }
    }


    keyPoint5.x = 0;
    for (int i = 0; i < pKPoints.size(); ++i) {
        int diffY = 0;
        if(i > 0){
            diffY = abs(pKPoints.at(i).keyPoint.y - pKPoints.at(i-1).keyPoint.y);
            //ROS_INFO("diff: %d", abs(pKPoints.at(i).keyPoint.y - pKPoints.at(i-1).keyPoint.y));
        }

        if(diffY > 3){
            break;
        }

        if(keyPoint5.x < pKPoints.at(i).keyPoint.x){
            keyPoint5.x = pKPoints.at(i).keyPoint.x;
            keyPoint5.y = pKPoints.at(i).keyPoint.y;
        }
        //circle(rMidFinal, pKPoints.at(i).keyPoint, 2, Scalar(255,255,255));
    }
}

void LipRec::extractCupidsBowKeyPoints(Mat& rTopFinal, Point& keyPoint2, Point& keyPoint3, Point& keyPoint4, Point& leftPointHorizontalLine, int thresholdDifferenceToAvg, int totalLineCheck)
{
    QList<PossibleKeyPoint> possibleKeyPoints;
    PossibleKeyPoint possibleKeyPoint;

    for (int i = 0; i < rTopFinal.cols; ++i) {
        for (int j = rTopFinal.rows/2; j > totalLineCheck/2; --j) {

            int currentDiffToAvg = 0;

            for (int k = 1; k < totalLineCheck/2 + 1; ++k) {
                currentDiffToAvg += rTopFinal.at<uchar>(j-k,i) + rTopFinal.at<uchar>(j+k,i);

            }
            currentDiffToAvg = currentDiffToAvg / totalLineCheck;

            if(currentDiffToAvg > 0){
                currentDiffToAvg = 100 - (rTopFinal.at<uchar>(j,i) * 100 / currentDiffToAvg);
            }

            if(currentDiffToAvg > thresholdDifferenceToAvg){
                possibleKeyPoint.differenceToAvg = currentDiffToAvg;
                possibleKeyPoint.keyPoint.x  = i;
                possibleKeyPoint.keyPoint.y  = j;
                possibleKeyPoints.append(possibleKeyPoint);
            }
        }
    }

    Mat contourImg(rTopFinal.rows, rTopFinal.cols, CV_8UC1, Scalar(0,0,0));
    Point p;
    for (int i = 0; i < possibleKeyPoints.size(); ++i) {
        p = possibleKeyPoints.at(i).keyPoint;
        contourImg.at<uchar>(p.y, p.x) = 255;
    }
    Mat _img;
    double otsu_thresh_val = cv::threshold(
                contourImg, _img, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU
                );

    Canny(contourImg, contourImg, otsu_thresh_val*0.5, otsu_thresh_val);

    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours( contourImg, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
    for( int i = 0; i< contours.size(); i++ ){
        drawContours( contourImg, contours, i, Scalar(255,255,255), 1, 8, hierarchy, 1, Point() );
    }

    keyPoint2.y = 1000;
    for (int i = 0; i < rTopFinal.rows; ++i) {
        for (int j = rTopFinal.cols/2; j > 0; --j) {
            if(contourImg.at<uchar>(i,j) == 255){
                if(keyPoint2.y >= i){
                    keyPoint2.y = i;
                    keyPoint2.x = j;
                }
            }
        }
    }

    keyPoint4.y = 1000;
    for (int i = 0; i < rTopFinal.rows; ++i) {
        for (int j = rTopFinal.cols/2; j < rTopFinal.cols; ++j) {
            if(contourImg.at<uchar>(i,j) == 255){
                if(keyPoint4.y >= i){
                    keyPoint4.y = i;
                    keyPoint4.x = j;
                }
            }
        }
    }

    keyPoint3.y = 0;
    int kp2kp3Width = keyPoint4.x  - keyPoint2.x;
    kp2kp3Width = kp2kp3Width/2;

    for (int i = keyPoint2.x; i < keyPoint4.x; ++i) {
        for (int j = 0; j < leftPointHorizontalLine.y-10; ++j) {
            if(contourImg.at<uchar>(j,i) == 255){
                if(keyPoint3.y <= j &&  i <= (keyPoint2.x + kp2kp3Width) ){
                    keyPoint3.y = j;
                    keyPoint3.x = i;
                }
            }
        }
    }
}

void LipRec::extractLowerLipKeyPoint(Mat& rLowFinal, Point& keyPoint6, int kp2X, int kp4X, Point& leftPointHorizontalLine, int thresholdDifferenceToAvg, int totalLineCheck)
{
    QList<PossibleKeyPoint> possibleKeyPoints;
    PossibleKeyPoint possibleKeyPoint;

    for (int i = 0; i < rLowFinal.cols; ++i) {
        for (int j = rLowFinal.rows/2; j < rLowFinal.rows-totalLineCheck/2; ++j) {

            int currentDiffToAvg = 0;

            for (int k = 1; k < totalLineCheck/2 + 1; ++k) {
                currentDiffToAvg += rLowFinal.at<uchar>(j-k,i) + rLowFinal.at<uchar>(j+k,i);

            }
            currentDiffToAvg = currentDiffToAvg / totalLineCheck;

            if(currentDiffToAvg > 0){
                currentDiffToAvg = 100 - (rLowFinal.at<uchar>(j,i) * 100 / currentDiffToAvg);
            }

            if(currentDiffToAvg > thresholdDifferenceToAvg){
                possibleKeyPoint.differenceToAvg = currentDiffToAvg;
                possibleKeyPoint.keyPoint.x  = i;
                possibleKeyPoint.keyPoint.y  = j;
                possibleKeyPoints.append(possibleKeyPoint);
            }
        }
    }

    Mat contourImg(rLowFinal.rows, rLowFinal.cols, CV_8UC1, Scalar(0,0,0));
    Point p;
    for (int i = 0; i < possibleKeyPoints.size(); ++i) {
        p = possibleKeyPoints.at(i).keyPoint;
        contourImg.at<uchar>(p.y, p.x) = 255;
    }
    Mat _img;
    double otsu_thresh_val = cv::threshold(
                contourImg, _img, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU
                );
    Canny(contourImg, contourImg, otsu_thresh_val*0.5, otsu_thresh_val);

    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours( contourImg, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
    for( int i = 0; i< contours.size(); i++ ){
        drawContours( contourImg, contours, i, Scalar(255,255,255), 1, 8, hierarchy, 1, Point() );
    }


    keyPoint6.y = 0;
    int kp2kp3Width = kp4X - kp2X;
    kp2kp3Width = kp2kp3Width/2;

    for (int i = kp2X; i < kp4X; ++i) {
        for (int j = rLowFinal.rows; j > leftPointHorizontalLine.y; --j) {
            if(contourImg.at<uchar>(j, i) == 255){
                if(keyPoint6.y <= j && i <= (kp2X + kp2kp3Width)){
                    keyPoint6.y = j;
                    keyPoint6.x = i;
                    break;
                }
            }
        }
    }
}

void LipRec::applyLipsSegmentationSaturation(Mat& mouthImg){
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

    int amountFacePixel = noPixelImg - (noPixelImg*ui_.sbSaturation->value()/100);
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

Mat LipRec::calcColorHistogramEqualization(Mat& img){
    vector<Mat> channels;
    Mat imgHistEqualized;

    cvtColor(img, imgHistEqualized, CV_BGR2YCrCb); //change the color image from BGR to YCrCb format

    split(imgHistEqualized, channels);

    equalizeHist(channels[0], channels[0]);

    merge(channels, imgHistEqualized);

    cvtColor(imgHistEqualized, imgHistEqualized, CV_YCrCb2BGR);

    return imgHistEqualized;
}

double LipRec::pseudoHuePxl(Mat img, int x, int y)
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

int LipRec::luminancePxl(Mat img, int x, int y)
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

void LipRec::triggedAction(QAction *action)
{
    QString currentAction = action->text();
    QString leFile = le->text();

    if(currentAction == "Record"){
        this->recordVideo = true;
    }else if(currentAction == "Stop Record"){
        this->recordVideo = false;
    }else if(currentAction == "Record Utterance"){
        this->recordUtterance = true;
    }else if(currentAction == "Load Utterance"){
        this->loadUtterance = true;
        this->changeUseCam();

        VideoCapture cap(QString(leFile + ".avi").toStdString());
        if(!cap.isOpened()){
            ROS_INFO("!!! Failed to open file: %s.avi", leFile.toStdString().c_str());
        }

        Mat img;

        while(cap.read(img)){
            cvtColor(img, img, CV_BGRA2GRAY);
            this->processImage(img);
            waitKey(40);
        }

    }else{
        ROS_INFO("Action '%s' not found.", action->text().toStdString().c_str());
    }
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

void LipRec::showLips(Mat& mouthImg, bool useMonoImage){
    QPixmap pixMap;
    if(ui_.cbLips->isChecked()){
        pixMap = imageProcessing.getPixmap(mouthImg, useMonoImage);
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

void LipRec::changeLipActivationState(int activation, Mat& imageAbsDiff, int currentFrame){
    QPixmap pixMap;
    switch (stateDetectionStartEndFrame) {
    case Idle:
        utterance.clear();
        utterancePixelDiff.clear();
        silenceCounter = 0;

        // Uterrance detected
        if(activation > ui_.sbST->value()){
            stateDetectionStartEndFrame = Utterance;
            utterancePixelDiff.append(activation);

            if(checkboxOnlyMouth->isChecked()){
                this->recordUtteranceFrame(frameBuffer[currentFrame]);
            }else{
                this->recordUtteranceFrame(currentUtteranceFrame);
            }

        }else{

        }

        break;
    case Utterance:
        if(activation <= ui_.sbST->value() && silenceCounter == ui_.sbNoSF->value()){
            //Utterrance finished

            stateDetectionStartEndFrame = Idle;
            Mat uLast = imageAbsDiff;
            if(!utterance.isEmpty()){
                uLast = utterance.last();
            }

            // only add imageAbsDiff if the last DOF-image has the same size
            if(uLast.cols == imageAbsDiff.cols && uLast.rows == imageAbsDiff.rows){
                utterance.append(imageAbsDiff);
                utterancePixelDiff.append(activation);

                if(checkboxOnlyMouth->isChecked()){
                    this->recordUtteranceFrame(frameBuffer[currentFrame]);
                }else{
                    this->recordUtteranceFrame(currentUtteranceFrame);
                }
                this->recordUtterance = false;
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


            if(ui_.rbSWT->isChecked()){
                Mat ca = Mat::zeros(mt.rows, mt.cols, CV_8UC1);
                Mat ch = Mat::zeros(mt.rows, mt.cols, CV_8UC1);
                Mat cd = Mat::zeros(mt.rows, mt.cols, CV_8UC1);
                Mat cv = Mat::zeros(mt.rows, mt.cols, CV_8UC1);

                swt.applySwt(mt, ca, ch, cd, cv, 1, Swt::Haar);

                pixMap = imageProcessing.getPixmap(ca, useMonoImage);
            }else if(ui_.rbDCT->isChecked()){
                pixMap = imageProcessing.getPixmap(mt, useMonoImage);
            }else{
                pixMap = imageProcessing.getPixmap(mt, useMonoImage);
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
        }else if(activation <= ui_.sbST->value()){
            //Silence during Utterance
            silenceCounter++;
            utterancePixelDiff.append(activation);

            if(checkboxOnlyMouth->isChecked()){
                this->recordUtteranceFrame(frameBuffer[currentFrame]);
            }else{
                this->recordUtteranceFrame(currentUtteranceFrame);
            }

        }else{
            //During Utterance
            silenceCounter = 0;
            Mat uLast = imageAbsDiff;
            if(!utterance.isEmpty()){
                uLast = utterance.last();
            }

            if(uLast.cols == imageAbsDiff.cols && uLast.rows == imageAbsDiff.rows){
                utterance.append(imageAbsDiff);
                utterancePixelDiff.append(activation);

                if(checkboxOnlyMouth->isChecked()){
                    this->recordUtteranceFrame(frameBuffer[currentFrame]);
                }else{
                    this->recordUtteranceFrame(currentUtteranceFrame);
                }
            }
        }
        break;
    default:
        break;
    }
}

void LipRec::recordUtteranceFrame(Mat currentFrame)
{
    if(recordUtterance && !recordVideo){
        if(!le->text().isEmpty()){
            if(!initVideoWriter){
                imageProcessing.setupVideoWriter(le->text()+".avi", currentFrame.cols, currentFrame.rows);
                initVideoWriter = true;
            }

            imageProcessing.writeFrameToVideo(currentFrame);
        }else{
            recordUtterance = false;
        }
    }else if(!recordVideo){
        initVideoWriter = false;
        imageProcessing.closeVideoWriter();
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

    QList<int> tmpSignalsSmoothing = signalsSmoothing;

    for (int i = 0; i < tmpSignalsSmoothing.size(); ++i) {
        if(i > 0 && i < tmpSignalsSmoothing.size()-1){
            signalsSmoothing[i] = (tmpSignalsSmoothing[i-1] + tmpSignalsSmoothing[i] + tmpSignalsSmoothing[i+1]) / 3;
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

void LipRec::clickedContinueVideo()
{
    this->loadUtterance = false;

    this->changeUseCam();
}

void LipRec::toggleKpLines(bool checked)
{
    drawKeyPointLines = checked;
}

void LipRec::toggleSupportLines(bool checked)
{
    drawSupportLines = checked;
}

void LipRec::changeUseCam()
{
    QIcon bi;
    QPixmap pixmap;

    if(useCam){
        ui_.pbContinueVideo->setToolTip("Continue video stream from camera.");
        pixmap = QPixmap("src/liprec/res/video1.png");
    }else{
        ui_.pbContinueVideo->setToolTip("Stop video stream from camera.");
        pixmap = QPixmap("src/liprec/res/videostop1.png");
    }

    bi = QIcon(pixmap);
    ui_.pbContinueVideo->setSizePolicy( QSizePolicy::Fixed, QSizePolicy::Preferred );
    ui_.pbContinueVideo->setFlat(true);
    ui_.pbContinueVideo->setIcon(bi);
    ui_.pbContinueVideo->setIconSize(pixmap.rect().size()/2.5);
    ui_.pbContinueVideo->setMaximumSize(pixmap.rect().size()/2.5);

    useCam = !useCam;
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
