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

    QObject::connect(ui_.pbPrintFeatures, SIGNAL(clicked()), this, SLOT(clickedPrintFeatures()));

    QObject::connect(ui_.pbToggleKpLines, SIGNAL(clicked()), this, SLOT(toggleKpLines()));

    drawKeyPointState = 0;
    ui_.pbToggleKpLines->setToolTip("Show keypoint lines.");
    QPixmap pixmap("src/liprec/res/lips.png");
    QIcon bi(pixmap);

    ui_.pbToggleKpLines->setSizePolicy( QSizePolicy::Fixed, QSizePolicy::Preferred );
    ui_.pbToggleKpLines->setFlat(true);
    ui_.pbToggleKpLines->setIcon(bi);
    ui_.pbToggleKpLines->setIconSize(pixmap.rect().size()/2.5);
    ui_.pbToggleKpLines->setMaximumSize(pixmap.rect().size()/2.5);

    QObject::connect(ui_.pbToggleSupportLines, SIGNAL(clicked(bool)), this, SLOT(toggleSupportLines(bool)));
    drawSupportLines = false;

    ui_.pbToggleSupportLines->setToolTip("Show support lines.");
    pixmap = QPixmap("src/liprec/res/lines.png");
    bi = QIcon(pixmap);

    ui_.pbToggleSupportLines->setSizePolicy( QSizePolicy::Fixed, QSizePolicy::Preferred );
    ui_.pbToggleSupportLines->setFlat(true);
    ui_.pbToggleSupportLines->setIcon(bi);
    ui_.pbToggleSupportLines->setIconSize(pixmap.rect().size()/2.5);
    ui_.pbToggleSupportLines->setMaximumSize(pixmap.rect().size()/2.5);

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
    pixmap = QPixmap("src/liprec/res/plot3.png");
    bi = QIcon(pixmap);
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
    printFeatures = false;

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
            mouthImg = imageProcessing.calcColorHistogramEqualization(mouthImg);
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

    Mat showMouthImg;

    Mat mouthFeatures(mouthImg.rows, mouthImg.cols, CV_8UC3, Scalar(0,0,0));

    Mat rTopFinal(mouthImg.rows, mouthImg.cols, CV_8UC1);
    Mat rMidFinal(mouthImg.rows, mouthImg.cols, CV_8UC1);
    Mat rLowFinal(mouthImg.rows, mouthImg.cols, CV_8UC1);

    Point keyPoint1, keyPoint2, keyPoint3, keyPoint4, keyPoint5, keyPoint6;

    showMouthImg = mouthImg;

    if(!useMonoImage){
        if(ui_.cbLipSeg->isChecked()){
            if(ui_.rbSaturation->isChecked()){
                imageProcessing.applyLipsSegmentationSaturation(mouthImg, ui_.sbSaturation->value());
                showMouthImg = mouthImg;
            }else if(ui_.rbPseudoHue->isChecked()){
                keyPointsDeliverer.calcGradientImages(mouthImg);
                rTopFinal = keyPointsDeliverer.getRTop();
                rMidFinal = keyPointsDeliverer.getRMid();
                rLowFinal = keyPointsDeliverer.getRLow();

                Point rightLinePoint(mouthROI.width, mouthROI.height/2);
                Point upLinePoint(mouthROI.width/2, 0);
                Point bottomLinePoint(mouthROI.width/2, mouthROI.height);

                keyPointsDeliverer.extractMouthCornerKeyPoints(mouthImg, ui_.sbTHMouthCorners->value(), ui_.sbLLMouthCorners->value());
                keyPointsDeliverer.extractCupidsBowKeyPoints(ui_.sbTHCupidsBow->value(), ui_.sbLLCupidsBow->value());
                keyPointsDeliverer.extractLowerLipKeyPoint(ui_.sbTHLowerLip->value(), ui_.sbLLLowerLip->value());

                keyPoint1 = keyPointsDeliverer.getKeyPoint1();
                keyPoint2 = keyPointsDeliverer.getKeyPoint2();
                keyPoint3 = keyPointsDeliverer.getKeyPoint3();
                keyPoint4 = keyPointsDeliverer.getKeyPoint4();
                keyPoint5 = keyPointsDeliverer.getKeyPoint5();
                keyPoint6 = keyPointsDeliverer.getKeyPoint6();

                double mouthWidth = norm(keyPoint5-keyPoint1);
                double mouthHeight = norm(keyPoint6-keyPoint3);
                double wh = mouthWidth / mouthHeight;
                double hw = mouthHeight / mouthWidth;

                double distanceKp12 = norm(keyPoint1-keyPoint2);
                double distanceKp23 = norm(keyPoint2-keyPoint3);
                double distanceKp34 = norm(keyPoint3-keyPoint4);
                double distanceKp45 = norm(keyPoint4-keyPoint5);
                double distanceKp56 = norm(keyPoint5-keyPoint6);
                double distanceKp61 = norm(keyPoint6-keyPoint1);

                double distanceKp13 = norm(keyPoint1-keyPoint3);
                double distanceKp35 = norm(keyPoint3-keyPoint5);
                double distanceKp36 = mouthHeight;

                // half perimeter of Triangle (hpot)
                double hpotA = (distanceKp12 + distanceKp23 + distanceKp13)/2;
                double hpotB = (distanceKp34 + distanceKp45 + distanceKp35)/2;
                double hpotC = (distanceKp13 + distanceKp36 + distanceKp61)/2;
                double hpotD = (distanceKp36 + distanceKp35 + distanceKp56)/2;

                double areaOfTriangleA = sqrt(hpotA*(hpotA-distanceKp12)*(hpotA-distanceKp23)*(hpotA-distanceKp13));
                double areaOfTriangleB = sqrt(hpotB*(hpotB-distanceKp34)*(hpotB-distanceKp45)*(hpotB-distanceKp35));
                double areaOfTriangleC = sqrt(hpotC*(hpotC-distanceKp13)*(hpotC-distanceKp36)*(hpotC-distanceKp61));
                double areaOfTriangleD = sqrt(hpotD*(hpotD-distanceKp36)*(hpotD-distanceKp35)*(hpotD-distanceKp56));

                this->drawMouthFeatures(mouthFeatures, keyPoint1, keyPoint2, keyPoint3, keyPoint4, keyPoint5, keyPoint6);

                if(printFeatures){
                    //ROS_INFO("MW:%f MH:%f W/H:%f H/W:%f Area:%f", mouthWidth, mouthHeight, wh, hw, areaOfTriangleA + areaOfTriangleB + areaOfTriangleC + areaOfTriangleD);
                    ROS_INFO("H/W:%f Area:%f", hw, areaOfTriangleA + areaOfTriangleB + areaOfTriangleC + areaOfTriangleD);
                }

                if(ui_.rbLipsNone->isChecked()){
                    if(drawSupportLines){
                        line(mouthImg, upLinePoint, bottomLinePoint, Scalar(255,255,255));
                        line(mouthImg, keyPoint1, rightLinePoint, Scalar(255,255,255));
                    }

                    if(drawKeyPointState == 1){
                        line(mouthImg, keyPoint1, keyPoint2, Scalar(255,255,255));
                        line(mouthImg, keyPoint2, keyPoint3, Scalar(255,255,255));
                        line(mouthImg, keyPoint3, keyPoint4, Scalar(255,255,255));
                        line(mouthImg, keyPoint4, keyPoint5, Scalar(255,255,255));
                        line(mouthImg, keyPoint5, keyPoint6, Scalar(255,255,255));
                        line(mouthImg, keyPoint6, keyPoint1, Scalar(255,255,255));
                    }else if(drawKeyPointState == 2){
                        Point points[1][6];
                        points[0][0] = keyPoint1;
                        points[0][1] = keyPoint2;
                        points[0][2] = keyPoint3;
                        points[0][3] = keyPoint4;
                        points[0][4] = keyPoint5;
                        points[0][5] = keyPoint6;

                        const Point* ppt[1] = { points[0] };
                        int npt[] = { 6 };

                        fillPoly( mouthImg,
                                  ppt,
                                  npt,
                                  1,
                                  Scalar( 255, 255, 255 ),
                                  8 );
                    }

                    circle(mouthImg, keyPoint6, 1, Scalar(255,255,255));

                    circle(mouthImg, keyPoint1, 1, Scalar(255,255,255));
                    circle(mouthImg, keyPoint5, 1, Scalar(255,255,255));

                    circle(mouthImg, keyPoint2, 1, Scalar(255,255,255));
                    circle(mouthImg, keyPoint4, 1, Scalar(255,255,255));
                    circle(mouthImg, keyPoint3, 1, Scalar(255,255,255));

                    showMouthImg = mouthImg;
                }else if(ui_.rbRLow->isChecked()){

                    circle(rLowFinal, keyPoint6, 1, Scalar(255,255,255));

                    if(drawSupportLines){
                        line(rLowFinal, upLinePoint, bottomLinePoint, Scalar(255,255,255));
                        line(rLowFinal, keyPoint1, rightLinePoint, Scalar(255,255,255));
                    }
                    showMouthImg = rLowFinal;
                }else if(ui_.rbRMid->isChecked()){

                    circle(rMidFinal, keyPoint1, 1, Scalar(255,255,255));
                    circle(rMidFinal, keyPoint5, 1, Scalar(255,255,255));

                    if(drawSupportLines){
                        line(rMidFinal, upLinePoint, bottomLinePoint, Scalar(255,255,255));
                        line(rMidFinal, keyPoint1, rightLinePoint, Scalar(255,255,255));
                    }
                    showMouthImg = rMidFinal;
                }else{
                    circle(rTopFinal, keyPoint2, 1, Scalar(255,255,255));
                    circle(rTopFinal, keyPoint4, 1, Scalar(255,255,255));
                    circle(rTopFinal, keyPoint3, 1, Scalar(255,255,255));

                    if(drawSupportLines){
                        line(rTopFinal, upLinePoint, bottomLinePoint, Scalar(255,255,255));
                        line(rTopFinal, keyPoint1, rightLinePoint, Scalar(255,255,255));
                    }

                    showMouthImg = rTopFinal;
                }
            }
        }
    }

    this->showLips(showMouthImg);


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
    bool monoImg = false;
    if(ui_.cbLips->isChecked()){

        if(mouthImg.type() == CV_8UC1){
            monoImg = true;
        }
        pixMap = imageProcessing.getPixmap(mouthImg, monoImg);

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

void LipRec::drawMouthFeatures(Mat &mouthFeatures, Point keyPoint1, Point keyPoint2, Point keyPoint3, Point keyPoint4, Point keyPoint5, Point keyPoint6)
{
    circle(mouthFeatures, keyPoint6, 1, Scalar(255,255,255));

    circle(mouthFeatures, keyPoint1, 1, Scalar(255,255,255));
    circle(mouthFeatures, keyPoint5, 1, Scalar(255,255,255));

    circle(mouthFeatures, keyPoint2, 1, Scalar(255,255,255));
    circle(mouthFeatures, keyPoint4, 1, Scalar(255,255,255));
    circle(mouthFeatures, keyPoint3, 1, Scalar(255,255,255));

    if(drawKeyPointState == 1){
        line(mouthFeatures, keyPoint1, keyPoint2, Scalar(255,255,255));
        line(mouthFeatures, keyPoint2, keyPoint3, Scalar(255,255,255));
        line(mouthFeatures, keyPoint3, keyPoint4, Scalar(255,255,255));
        line(mouthFeatures, keyPoint4, keyPoint5, Scalar(255,255,255));
        line(mouthFeatures, keyPoint5, keyPoint6, Scalar(255,255,255));
        line(mouthFeatures, keyPoint6, keyPoint1, Scalar(255,255,255));
    }else if(drawKeyPointState == 2){
        Point points[1][3];
        points[0][0] = keyPoint1;
        points[0][1] = keyPoint2;
        points[0][2] = keyPoint3;

        const Point* ppt[1] = { points[0] };
        int npt[] = { 3 };

        fillPoly( mouthFeatures,
                  ppt,
                  npt,
                  1,
                  Scalar( 255, 0, 0 ),
                  8 );

        points[0][0] = keyPoint3;
        points[0][1] = keyPoint4;
        points[0][2] = keyPoint5;

        ppt[0] = points[0];

        fillPoly( mouthFeatures,
                  ppt,
                  npt,
                  1,
                  Scalar( 0, 255, 0 ),
                  8 );

        points[0][0] = keyPoint3;
        points[0][1] = keyPoint5;
        points[0][2] = keyPoint6;

        ppt[0] = points[0];

        fillPoly( mouthFeatures,
                  ppt,
                  npt,
                  1,
                  Scalar( 0, 0, 255 ),
                  8 );

        points[0][0] = keyPoint1;
        points[0][1] = keyPoint3;
        points[0][2] = keyPoint6;

        ppt[0] = points[0];

        fillPoly( mouthFeatures,
                  ppt,
                  npt,
                  1,
                  Scalar( 0, 255, 255 ),
                  8 );

        putText(mouthFeatures, "P1", Point(keyPoint1.x-13, keyPoint1.y+3), CV_FONT_HERSHEY_PLAIN, 0.5, Scalar(255,255,255));
        putText(mouthFeatures, "P2", Point(keyPoint2.x-3, keyPoint2.y-6), CV_FONT_HERSHEY_PLAIN, 0.5, Scalar(255,255,255));
        putText(mouthFeatures, "P3", Point(keyPoint3.x-3, keyPoint3.y-6), CV_FONT_HERSHEY_PLAIN, 0.5, Scalar(255,255,255));
        putText(mouthFeatures, "P4", Point(keyPoint4.x-3, keyPoint4.y-6), CV_FONT_HERSHEY_PLAIN, 0.5, Scalar(255,255,255));
        putText(mouthFeatures, "P5", Point(keyPoint5.x+5, keyPoint5.y+3), CV_FONT_HERSHEY_PLAIN, 0.5, Scalar(255,255,255));
        putText(mouthFeatures, "P6", Point(keyPoint6.x-3, keyPoint6.y+10), CV_FONT_HERSHEY_PLAIN, 0.5, Scalar(255,255,255));

        Point a(keyPoint1.x+((keyPoint3.x-keyPoint1.x)/2), keyPoint2.y+5);
        Point b(keyPoint5.x-((keyPoint5.x-keyPoint3.x)/2), keyPoint4.y+5);
        Point c(keyPoint6.x+((keyPoint5.x-keyPoint6.x)/2)-5, keyPoint3.y+((keyPoint6.y-keyPoint3.y)/2));
        Point d(keyPoint6.x-((keyPoint6.x - keyPoint1.x)/2), keyPoint3.y+((keyPoint6.y-keyPoint3.y)/2));

        putText(mouthFeatures, "A", a, CV_FONT_HERSHEY_PLAIN, 0.5, Scalar(0,0,0));
        putText(mouthFeatures, "B", b, CV_FONT_HERSHEY_PLAIN, 0.5, Scalar(0,0,0));
        putText(mouthFeatures, "C", c, CV_FONT_HERSHEY_PLAIN, 0.5, Scalar(0,0,0));
        putText(mouthFeatures, "D", d, CV_FONT_HERSHEY_PLAIN, 0.5, Scalar(0,0,0));

        QPixmap pixMap;
        bool monoImg = false;
        if(ui_.cbLips->isChecked()){

            if(mouthFeatures.type() == CV_8UC1){
                monoImg = true;
            }
            pixMap = imageProcessing.getPixmap(mouthFeatures, monoImg);

            pixMap = pixMap.scaled(ui_.lbl_MHI->maximumWidth(), ui_.lbl_MHI->maximumHeight(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
            ui_.lbl_MHI->setPixmap(pixMap);
        }else{
            QPixmap empty;
            ui_.lbl_MHI->setPixmap(empty);
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

void LipRec::toggleKpLines()
{
    QPixmap pixmap;
    if(drawKeyPointState == 0){
        drawKeyPointState = 1;
        ui_.pbToggleKpLines->setToolTip("Fill lips.");
        pixmap = QPixmap("src/liprec/res/lipsLines.png");

    }else if(drawKeyPointState == 1){
        drawKeyPointState = 2;
        ui_.pbToggleKpLines->setToolTip("Hide keypoint lips.");
        pixmap = QPixmap("src/liprec/res/lipsFilled.png");

    }else{
        drawKeyPointState = 0;
        ui_.pbToggleKpLines->setToolTip("Show keypoint lips.");
        pixmap = QPixmap("src/liprec/res/lips.png");

    }
    QIcon bi(pixmap);

    ui_.pbToggleKpLines->setSizePolicy( QSizePolicy::Fixed, QSizePolicy::Preferred );
    ui_.pbToggleKpLines->setFlat(true);
    ui_.pbToggleKpLines->setIcon(bi);
    ui_.pbToggleKpLines->setIconSize(pixmap.rect().size()/2.5);
    ui_.pbToggleKpLines->setMaximumSize(pixmap.rect().size()/2.5);
}

void LipRec::toggleSupportLines(bool checked)
{
    drawSupportLines = checked;

    QPixmap pixmap;
    if(drawSupportLines){
        ui_.pbToggleSupportLines->setToolTip("Hide support lines.");
        pixmap = QPixmap("src/liprec/res/linesChecked.png");
    }else{
        ui_.pbToggleSupportLines->setToolTip("Show support lines.");
        pixmap = QPixmap("src/liprec/res/lines.png");
    }
    QIcon bi(pixmap);

    ui_.pbToggleSupportLines->setSizePolicy( QSizePolicy::Fixed, QSizePolicy::Preferred );
    ui_.pbToggleSupportLines->setFlat(true);
    ui_.pbToggleSupportLines->setIcon(bi);
    ui_.pbToggleSupportLines->setIconSize(pixmap.rect().size()/2.5);
    ui_.pbToggleSupportLines->setMaximumSize(pixmap.rect().size()/2.5);

}

void LipRec::clickedPrintFeatures()
{
    if(printFeatures){
        ROS_INFO(">>>>>>>>>>>End");
    }else{
        ROS_INFO(">>>>>>>>>>>Start");
    }
    printFeatures = !printFeatures;
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
