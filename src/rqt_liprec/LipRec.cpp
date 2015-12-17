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

    qRegisterMetaType<Mat>("Mat");

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
        kinectTopic = "/kinect2/qhd/image_mono_rect";
    }else{
        kinectTopic = "/kinect2/qhd/image_color_rect";
        //kinectTopic = "/kinect2/qhd/image_depth_rect";
    }
    this->imageProcessing.setUseMonoImage(useMonoImage);

    camImage = getNodeHandle().subscribe(kinectTopic, 100, &LipRec::imageCallback, this);
    camImageDepth = getNodeHandle().subscribe("/kinect2/qhd/image_depth_rect", 100, &LipRec::imageDepthCallback, this);

    faceROISub = getNodeHandle().subscribe("/face_detection/faceROI", 10, &LipRec::faceROICallback, this);
    mouthROISub = getNodeHandle().subscribe("/face_detection/mouthROI", 10, &LipRec::mouthROICallback, this);

    NO_CYCLIC_FRAME = ui_.sbNOCF->value();

    QObject::connect(&faceROITimer, SIGNAL(timeout()), this, SLOT(faceROItimeout()));
    QObject::connect(&mouthROITimer, SIGNAL(timeout()), this, SLOT(mouthROItimeout()));

    QObject::connect(ui_.pbUPDP, SIGNAL(clicked()), this, SLOT(clickedUtteranceDiffPlot()));
    QObject::connect(ui_.pbContinueVideo, SIGNAL(clicked()), this, SLOT(clickedContinueVideo()));

    QObject::connect(ui_.pbPrintFeatures, SIGNAL(clicked()), this, SLOT(clickedPrintFeatures()));

    QObject::connect(ui_.pbToggleKpLines, SIGNAL(clicked()), this, SLOT(toggleKpLines()));

    QObject::connect(ui_.pbRecordStopTrajectory, SIGNAL(clicked()), this, SLOT(clickedRecordStopTrajectory()));
    QObject::connect(ui_.pbSaveTrajectory, SIGNAL(clicked()), this, SLOT(clickedSaveTrajectory()));
    QObject::connect(ui_.pbAbortTrajectory, SIGNAL(clicked()), this, SLOT(clickedAbortTrajectory()));

    QObject::connect(ui_.pbCluster, SIGNAL(clicked()), this, SLOT(clickedCluster()));


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

    ui_.lblDbName->setText("liprec");
    QFont font = QFont("Times New Roman", 12, QFont::DemiBold);
    font.setUnderline(true);
    ui_.lblDbName->setFont(font);

    ui_.lcdArea->setDigitCount(10);
    ui_.lcdAspectRatio->setDigitCount(10);
    ui_.lcdDistance->setDigitCount(10);

    lcdUpdateTimeStamp = QDateTime::currentMSecsSinceEpoch();

    utter = false;

    recordVideo = false;
    recordUtterance = false;
    loadUtterance = false;
    initVideoWriter = false;
    useCam = true;
    printFeatures = false;
    recordTrajectoryState = None;

    availableTrajectories << "all";

    tdm.connectToDatabase("localhost", "liprec", "liprec");
    tdm.setCollectionCluster("clustering");

    QMap<QString, int> commandsWithCount = tdm.getAllCommandsWithCount();
    foreach (QString command, commandsWithCount.keys()) {
        QStringList strL = tdm.getFeatures(command);
        foreach (QString feature, strL) {

            QList<QList<double> > clusterT = tdm.getClusterTrajectories(command, feature, ui_.rbKmedoids->text());
            if(clusterT.size() > 0){
                this->setClusterTrajectories(clusterT, command, feature, ui_.rbKmedoids->text());
            }

            clusterT = tdm.getClusterTrajectories(command, feature, ui_.rbMosaCluster->text());
            if(clusterT.size() > 0){
                this->setClusterTrajectories(clusterT, command, feature, ui_.rbMosaCluster->text());
            }
        }
    }


    QObject::connect(this, SIGNAL(updateCam(Mat)), this, SLOT(getCamPic(Mat)));
    QObject::connect(this, SIGNAL(updateDepthCam(Mat)), this, SLOT(getDepthCamPic(Mat)));
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
            //cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
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

void LipRec::imageDepthCallback(const sensor_msgs::ImageConstPtr &msg)
{
    cv::Mat img;

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);

    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    img = cv_ptr->image;

    emit updateDepthCam(img);
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

void LipRec::getCamPic(Mat img){
    if(!this->loadUtterance && useCam && !img.empty()){
        this->processImage(img);
    }
}

void LipRec::getDepthCamPic(Mat img)
{
    if(!this->loadUtterance && useCam && !img.empty()){
        depthCamMtx.lock();
        depthCam = img;
        depthCamMtx.unlock();
//        QPixmap pixMap = imageProcessing.getPixmap(img, true);
//        ui_.lbl_cam->setPixmap(pixMap);
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

    ///TEST DTW
    //    QList<double> t1;
    //    QList<double> t2;

    ////    t1 << 0.43 << 0.12 << 0.8 << 0.9 << 0.5;
    ////    t2 << 0.334 << 0.23 << 0.53 << 0.82 << 0.3 << 0.11 << 0.4;
    ////    t1 << 0.490196 << 0.490196 << 0.490196 << 0.4999 << 0.48068 << 0.501497 << 0.520833 << 0.666667 << 0.773059 << 0.704364 << 0.599408
    ////       << 0.565217 << 0.553191 << 0.604036 << 0.617021 << 0.705431 << 0.583207 << 0.608696 << 0.681115 << 0.790484 << 0.860233 << 0.929792
    ////       <<0.891566 << 0.894737 << 0.837209 << 0.777778 << 0.738956 << 0.638153 << 0.604036 << 0.559888 << 0.54 << 0.52 << 0.490196
    ////       <<0.48 << 0.461453 << 0.48 << 0.479904 << 0.501497 << 0.5003 << 0.4999 << 0.479904;

    ////    t2 << 0.479904 << 0.48 << 0.490102 << 0.5 << 0.480769 << 0.470588 << 0.470498 << 0.470498 << 0.490494 << 0.587359 << 0.674419 << 0.737886 <<
    ////          0.808608 << 0.789844 << 0.766613 << 0.644444 << 0.586403 << 0.608552 << 0.651558 << 0.680235 << 0.651558 << 0.59561 << 0.599852 <<
    ////          0.636199 << 0.727085 << 0.837276 << 0.889787 << 0.918349 << 0.823971 << 0.811749 << 0.695488 << 0.637721 << 0.616882 << 0.59561 <<
    ////          0.562398 << 0.519584 << 0.4999 << 0.510098 << 0.50978 << 0.50978 << 0.4996 << 0.4996 << 0.5 << 0.479904 << 0.479904 << 0.48 << 0.479904;

    //    t1 << 881 << 873.5 << 865.5 << 853.5 << 875.5 << 846.5 << 822 << 862.5 << 900 << 918.5 << 840.5 << 844 << 866 << 944.5 << 933 << 817.5 << 927.5 <<
    //          882 << 894.5 << 952.5 << 1043.5 << 1035 << 793 << 839 << 1010.5 << 1022 << 1031 << 944 << 920 << 929 << 885.5 << 876 << 856.5 << 823.5 << 846.5 <<
    //          823.5 << 831.5 << 824.5 << 847 << 843.5 << 818.5;

    //    t2 << 830.5 << 826 << 870.5 << 851 << 880 << 852 << 857.5 << 857.5 << 867.5 << 814.5 << 806.5 << 856.5 << 926 << 974.5 << 942 << 865 << 853 << 883.5 <<
    //          927.5 << 993 << 934.5 << 906 << 848.5 << 839 << 943.5 << 1034 << 863 << 740.5 << 850 << 934 << 967.5 << 948 << 900.5 << 871 << 855 << 870 << 828.5 <<
    //          823.5 << 810 << 810 << 824 << 830 << 831.5 << 824.5 << 819 << 814 << 825;


    //    dtw.seed(t1, t2);

    //    Mat dtwMat(t1.size(), t2.size(), CV_64F, Scalar(0.0));
    //    Mat dtwMatTemp(t1.size(), t2.size(), CV_64F, Scalar(0.0));
    //    QList<Point> warpingPath;

    //    //ROS_INFO("ABS");
    //    dtw.calculateDistanceMatrix(ABS);
    //    //dtw.printDistanceMatrix();
    //    dtwMat = dtw.calculateDtwDistanceMatrix();
    //    dtwMatTemp = dtwMat;
    //    warpingPath = dtw.calculateGreedyWarpingPath();
    //    //dtw.printDtwDistanceMatric();


    //    //ROS_INFO("SQUARE");
    //    //dtw.calculateDistanceMatrix(SQUARE);
    //    //dtw.printDistanceMatrix();
    //    //dtw.calculateDtwDistanceMatrix();
    //    //dtw.printDtwDistanceMatric();

    //    //ROS_INFO("SQUARE2");
    //    //dtw.calculateDistanceMatrix(SQUARE2);
    //    //dtw.printDistanceMatrix();
    //    //dtw.calculateDtwDistanceMatrix();
    //    //dtw.printDtwDistanceMatric();

    //    Mat dtwMat2(dtwMat.rows-1, dtwMat.cols-1, CV_64F);

    //    //dtwMat.convertTo(dtwMat, CV_8U);
    //    for (int i = 1; i < dtwMat.cols; ++i) {
    //        for (int j = 1; j < dtwMat.rows; ++j) {
    //            dtwMat2.at<double>(j-1,i-1) = dtwMat.at<double>(j,i);
    //            //ROS_INFO("%f", dtwMat2.at<double>(j-1,i-1));
    //        }
    //    }

    //    double min = 0.0;
    //    double max = 0.0;
    //    minMaxIdx(dtwMat2, &min, &max);
    //    dtwMat = Mat(dtwMat2.rows, dtwMat2.cols, CV_8U);

    //    double oldRange = max - min;
    //    double newRange = 255.0 - 0;
    //    double newValue = 0.0;
    //    for (int i = 0; i < dtwMat.cols; ++i) {
    //        for (int j = 0; j < dtwMat.rows; ++j) {
    //            newValue = (((dtwMat2.at<double>(j,i)-min) *newRange)/oldRange) + 0;
    //            dtwMat.at<uchar>(j,i) = (int) newValue;
    //        }
    //    }

    //    double sum = 0.0;
    //    for (int i = 0; i < warpingPath.size(); ++i) {
    //       circle(dtwMat, warpingPath.at(i), 1, Scalar(255,255,255));
    //       sum += dtwMatTemp.at<double>(warpingPath.at(i).y, warpingPath.at(i).x);
    //    }
    //    //ROS_INFO("SUM %f", sum);
    //    warpingPath.clear();

    //    dtwMat = 255 - dtwMat;

    //    QPixmap dtwPixMap = imageProcessing.getPixmap(dtwMat, true);
    //    dtwPixMap = dtwPixMap.scaled(ui_.lblDTW->maximumWidth(), ui_.lblDTW->maximumHeight(), Qt::KeepAspectRatio, Qt::SmoothTransformation);

    //    ui_.lblDTW->setPixmap(dtwPixMap);

    ///TEST DTW

    tdm.setCollection(ui_.leCollection->text());

    QMap<QString, int> commandsWithCount = tdm.getAllCommandsWithCount();

    QString commands = "";
    QString featuresForCmd = "";

    int i = 1;
    foreach (QString command, commandsWithCount.keys()) {
        featuresForCmd.clear();
        QStringList strL = tdm.getFeatures(command);
        foreach (QString feature, strL) {
            featuresForCmd = featuresForCmd + feature + ", ";
        }
        commands = commands + QString::number(i++)+ ") " + command + " (" + QString::number(commandsWithCount[command]) +")" + " :\n" +featuresForCmd + "\n";

        if(!availableTrajectories.contains(command)){
            availableTrajectories << command;
            ui_.cbTrajectory->clear();
            ui_.cbTrajectory->addItems(availableTrajectories);
        }
    }

    ui_.lblTrajectoriesInfo->setText(commands);
    ui_.lblTrajectoriesInfo->setFont(QFont("Times New Roman", 11, QFont::Normal));


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
    Mat rawMouthImg;

    Mat mouthFeatures(mouthImg.rows, mouthImg.cols, CV_8UC3, Scalar(0,0,0));

    Mat rTopFinal(mouthImg.rows, mouthImg.cols, CV_8UC1);
    Mat rMidFinal(mouthImg.rows, mouthImg.cols, CV_8UC1);
    Mat rLowFinal(mouthImg.rows, mouthImg.cols, CV_8UC1);

    Point keyPoint1, keyPoint2, keyPoint3, keyPoint4, keyPoint5, keyPoint6;

    showMouthImg = mouthImg;
    mouthImg.copyTo(rawMouthImg);

    //    if(mouthImg.cols != 0){
    //        //imageProcessing.squareImage(mouthImg);
    //    }

    if(rawMouthImg.type() == CV_8UC3){
        cvtColor(rawMouthImg, rawMouthImg, CV_BGR2GRAY);
    }
    currentFrame = updateFrameBuffer(rawMouthImg);
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

    this->changeLipActivationState(activation, imageAbsDiff, currentFrame);

    if(!imageAbsDiff.empty()){
        pixMap = imageProcessing.getPixmap(imageAbsDiff, true);
        pixMap = pixMap.scaled(ui_.lblMouthDiff->maximumWidth(), ui_.lblMouthDiff->maximumHeight(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
        ui_.lblMouthDiff->setPixmap(pixMap);
    }

    DistanceFunction df;

    if(ui_.rbABS->isChecked()){
        df = ABS;
    }else if(ui_.rbSQUARE->isChecked()){
        df = SQUARE;
    }else{
        df = SQUARE2;
    }

    Mat depthCamTmp;
    depthCamMtx.lock();
    depthCam.copyTo(depthCamTmp);
    depthCamMtx.unlock();

    int xDepth = faceROI.x_offset+(faceROI.width/2);
    int yDepth = faceROI.y_offset+(faceROI.height*0.2);

    if(!depthCamTmp.empty()){
        //ROS_INFO("%d %d -> %f", xDepth, yDepth, depthCamTmp.at<float>(yDepth, xDepth));
    }

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

                keyPointsDeliverer.extractMouthCornerKeyPoints(mouthImg, ui_.sbTHMouthCorners->value(), ui_.sbLLMouthCorners->value(),
                                                               ui_.sbKP1BreakMouthCorners->value(), ui_.sbKP5BreakMouthCorners->value());
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

                double area = areaOfTriangleA + areaOfTriangleB + areaOfTriangleC + areaOfTriangleD;


                if(utter == true && stateDetectionStartEndFrame == Idle){

                    QList<QList<double> > clusterT;
                    QList<QList<double> > clusterT2;
                    double bestWarpingCostArea = INT_MAX;
                    double bestWarpingCostAspectRatio = INT_MAX;
                    double bestWarpingCostFusion = INT_MAX;
                    QString currentCommandArea = "";
                    QString currentCommandAspectRatio = "";
                    QString currentCommandFusion = "";
                    int indexOfLowAreaCluster = 0;
                    int indexOfLowAspectRatioCluster = 0;

                    if(currentUtteranceTrajectories.size() > 0){

                        if(ui_.rbSingleFF->isChecked()){

                            foreach (QString command, availableTrajectories) {
                                clusterT = this->getClusterTrajectories(command, ui_.cbArea->text(), ui_.rbKmedoids->text());

                                double warpingCostTmpArea = 0.0;
                                for (int i = 0; i < clusterT.size(); i++) {
                                    dtw.seed(clusterT.at(i), currentUtteranceTrajectories[ui_.cbArea->text()]);

                                    warpingCostTmpArea = dtw.calcWarpingCost(df);

                                    ROS_INFO("Area Command: %s(%d) ; Utterrance: %d -> %f",
                                             command.toStdString().c_str(), clusterT.at(i).size(), currentUtteranceTrajectories[ui_.cbArea->text()].size(), warpingCostTmpArea);

                                    if(warpingCostTmpArea < bestWarpingCostArea){
                                        indexOfLowAreaCluster = i;
                                        bestWarpingCostArea = warpingCostTmpArea;
                                        currentCommandArea = command;
                                    }
                                }

                                clusterT = this->getClusterTrajectories(command, ui_.cbAspectRatio->text(), ui_.rbKmedoids->text());

                                double warpingCostTmpAspectRatio = 0.0;
                                for (int i = 0; i < clusterT.size(); i++) {
                                    dtw.seed(clusterT.at(i), currentUtteranceTrajectories[ui_.cbAspectRatio->text()]);

                                    warpingCostTmpAspectRatio =  dtw.calcWarpingCost(df);

                                    ROS_INFO("AspectRatio Command: %s(%d) ; Utterrance: %d -> %f",
                                             command.toStdString().c_str(), clusterT.at(i).size(), currentUtteranceTrajectories[ui_.cbAspectRatio->text()].size(), warpingCostTmpAspectRatio);

                                    if(warpingCostTmpAspectRatio < bestWarpingCostAspectRatio){
                                        indexOfLowAspectRatioCluster = i;
                                        bestWarpingCostAspectRatio = warpingCostTmpAspectRatio;
                                        currentCommandAspectRatio = command;
                                    }
                                }

                                clusterT = this->getClusterTrajectories(currentCommandAspectRatio, ui_.cbAspectRatio->text(), ui_.rbKmedoids->text());
                                if(!clusterT.isEmpty()){
                                    dtw.seed(clusterT.at(indexOfLowAspectRatioCluster), currentUtteranceTrajectories[ui_.cbAspectRatio->text()]);
                                    dtw.calcWarpingCost(df);
                                    Mat dtwMat;
                                    Mat dtwMat2(dtw.getDtwDistanceMatrix().rows-1, dtw.getDtwDistanceMatrix().cols-1, CV_64F);

                                    //dtwMat.convertTo(dtwMat, CV_8U);
                                    for (int i = 1; i < dtw.getDtwDistanceMatrix().cols; ++i) {
                                        for (int j = 1; j < dtw.getDtwDistanceMatrix().rows; ++j) {
                                            dtwMat2.at<double>(j-1,i-1) = dtw.getDtwDistanceMatrix().at<double>(j,i);
                                            //ROS_INFO("%f", dtwMat2.at<double>(j-1,i-1));
                                        }
                                    }

                                    double min = 0.0;
                                    double max = 0.0;
                                    minMaxIdx(dtwMat2, &min, &max);
                                    dtwMat = Mat(dtwMat2.rows, dtwMat2.cols, CV_8U);

                                    double oldRange = max - min;
                                    double newRange = 255.0 - 0;
                                    double newValue = 0.0;
                                    for (int i = 0; i < dtwMat.cols; ++i) {
                                        for (int j = 0; j < dtwMat.rows; ++j) {
                                            newValue = (((dtwMat2.at<double>(j,i)-min) *newRange)/oldRange) + 0;
                                            dtwMat.at<uchar>(j,i) = (int) newValue;
                                        }
                                    }

                                    double sum = 0.0;
                                    for (int i = 0; i < dtw.getWarpingPath().size(); ++i) {
                                       circle(dtwMat, dtw.getWarpingPath().at(i), 1, Scalar(255,255,255));
                                       sum += dtw.getDtwDistanceMatrix().at<double>(dtw.getWarpingPath().at(i).y, dtw.getWarpingPath().at(i).x);
                                    }
                                    //ROS_INFO("SUM %f", sum);

                                    dtwMat = 255 - dtwMat;

                                    QPixmap dtwPixMap = imageProcessing.getPixmap(dtwMat, true);
                                    dtwPixMap = dtwPixMap.scaled(ui_.lblDTW->maximumWidth(), ui_.lblDTW->maximumHeight(), Qt::KeepAspectRatio, Qt::SmoothTransformation);

                                    ui_.lblDTW->setPixmap(dtwPixMap);
                                }
                            }

                            ROS_INFO("Recognize Area: %s", currentCommandArea.toStdString().c_str());
                            ROS_INFO("Recognize AspectRatio: %s", currentCommandAspectRatio.toStdString().c_str());

                        }else if(ui_.rbFusionFF->isChecked()){

                            foreach (QString command, availableTrajectories) {

                                clusterT = this->getClusterTrajectories(command, ui_.cbArea->text(), ui_.rbKmedoids->text());
                                clusterT2 = this->getClusterTrajectories(command, ui_.cbAspectRatio->text(), ui_.rbKmedoids->text());

                                double warpingCostTmpArea = INT_MAX;
                                double warpingCostTmpAspectRatio = INT_MAX;
                                double warpingCostFusionTmp = 0.0;

                                for (int i = 0; i < clusterT.size(); ++i) {
                                    dtw.seed(clusterT.at(i), currentUtteranceTrajectories[ui_.cbArea->text()]);
                                    double wpArea = dtw.calcWarpingCost(df);

                                    ROS_INFO("Area Command: %s(%d) ; Utterrance: %d -> %f",
                                             command.toStdString().c_str(), clusterT.at(i).size(),
                                             currentUtteranceTrajectories[ui_.cbArea->text()].size(), wpArea);

                                    if(wpArea < warpingCostTmpArea){
                                        indexOfLowAreaCluster = i;
                                        warpingCostTmpArea = wpArea;
                                    }
                                }

                                for (int i = 0; i < clusterT2.size(); ++i) {
                                    dtw.seed(clusterT2.at(i), currentUtteranceTrajectories[ui_.cbAspectRatio->text()]);
                                    double wpAspectRatio =  dtw.calcWarpingCost(df);

                                    ROS_INFO("Aspect Ratio Command: %s(%d) ; Utterrance: %d -> %f",
                                             command.toStdString().c_str(), clusterT2.at(i).size(),
                                             currentUtteranceTrajectories[ui_.cbAspectRatio->text()].size(), wpAspectRatio);

                                    if(wpAspectRatio < warpingCostTmpAspectRatio){
                                        indexOfLowAspectRatioCluster = i;
                                        warpingCostTmpAspectRatio = wpAspectRatio;
                                    }
                                }

                                warpingCostFusionTmp = warpingCostTmpArea + warpingCostTmpAspectRatio;

                                if(warpingCostFusionTmp < bestWarpingCostArea){
                                    bestWarpingCostFusion = warpingCostFusionTmp;
                                    currentCommandFusion = command;
                                }


                                clusterT2 = this->getClusterTrajectories(currentCommandFusion, ui_.cbAspectRatio->text(), ui_.rbKmedoids->text());

                                if(!clusterT2.isEmpty()){
                                    dtw.seed(clusterT2.at(indexOfLowAspectRatioCluster), currentUtteranceTrajectories[ui_.cbAspectRatio->text()]);
                                    dtw.calcWarpingCost(df);
                                    Mat dtwMat;
                                    Mat dtwMat2(dtw.getDtwDistanceMatrix().rows-1, dtw.getDtwDistanceMatrix().cols-1, CV_64F);

                                    //dtwMat.convertTo(dtwMat, CV_8U);
                                    for (int i = 1; i < dtw.getDtwDistanceMatrix().cols; ++i) {
                                        for (int j = 1; j < dtw.getDtwDistanceMatrix().rows; ++j) {
                                            dtwMat2.at<double>(j-1,i-1) = dtw.getDtwDistanceMatrix().at<double>(j,i);
                                            //ROS_INFO("%f", dtwMat2.at<double>(j-1,i-1));
                                        }
                                    }

                                    double min = 0.0;
                                    double max = 0.0;
                                    minMaxIdx(dtwMat2, &min, &max);
                                    dtwMat = Mat(dtwMat2.rows, dtwMat2.cols, CV_8U);

                                    double oldRange = max - min;
                                    double newRange = 255.0 - 0;
                                    double newValue = 0.0;
                                    for (int i = 0; i < dtwMat.cols; ++i) {
                                        for (int j = 0; j < dtwMat.rows; ++j) {
                                            newValue = (((dtwMat2.at<double>(j,i)-min) *newRange)/oldRange) + 0;
                                            dtwMat.at<uchar>(j,i) = (int) newValue;
                                        }
                                    }

                                    double sum = 0.0;
                                    for (int i = 0; i < dtw.getWarpingPath().size(); ++i) {
                                       circle(dtwMat, dtw.getWarpingPath().at(i), 1, Scalar(255,255,255));
                                       sum += dtw.getDtwDistanceMatrix().at<double>(dtw.getWarpingPath().at(i).y, dtw.getWarpingPath().at(i).x);
                                    }
                                    //ROS_INFO("SUM %f", sum);

                                    dtwMat = 255 - dtwMat;

                                    QPixmap dtwPixMap = imageProcessing.getPixmap(dtwMat, true);
                                    dtwPixMap = dtwPixMap.scaled(ui_.lblDTW->maximumWidth(), ui_.lblDTW->maximumHeight(), Qt::KeepAspectRatio, Qt::SmoothTransformation);

                                    ui_.lblDTW->setPixmap(dtwPixMap);
                                }
                            }

                            ROS_INFO("Recognize Fusion: %s", currentCommandFusion.toStdString().c_str());
                        }

                    }

                    currentUtteranceTrajectories.clear();
                    utter = false;
                }else if(utter){
                    currentUtteranceTrajectories[ui_.cbArea->text()].append(area);
                    currentUtteranceTrajectories[ui_.cbAspectRatio->text()].append(hw);
                }


                this->drawMouthFeatures(mouthFeatures, keyPoint1, keyPoint2, keyPoint3, keyPoint4, keyPoint5, keyPoint6);

                double areaMean = 0.0;
                double aspectRatioMean = 0.0;

                switch (recordTrajectoryState) {
                case None:
                    break;
                case Recording:
                    if(ui_.cbAspectRatio->isChecked()){
                        recordTrajectory[ui_.cbAspectRatio->text()].append(hw);
                    }

                    if(ui_.cbArea->isChecked()){
                        recordTrajectory[ui_.cbArea->text()].append(area);
                    }

                    break;
                case Save:
                    if(ui_.cbAspectRatio->isChecked()){
                        tdm.insertTrajectory(recordTrajectory[ui_.cbAspectRatio->text()], ui_.leCommand->text(), ui_.cbAspectRatio->text());
                    }

                    if(ui_.cbArea->isChecked()){
                        tdm.insertTrajectory(recordTrajectory[ui_.cbArea->text()], ui_.leCommand->text(), ui_.cbArea->text());
                    }

                    ui_.lblNTValues->setText(QString::number(recordTrajectory[ui_.cbAspectRatio->text()].size()));

                    if(recordTrajectory[ui_.cbAspectRatio->text()].size() > 0){
                        for (int i = 0; i < recordTrajectory[ui_.cbAspectRatio->text()].size(); ++i) {
                            aspectRatioMean += recordTrajectory[ui_.cbAspectRatio->text()].at(i);
                        }
                    }

                    if(recordTrajectory[ui_.cbArea->text()].size() > 0){
                        for (int i = 0; i < recordTrajectory[ui_.cbArea->text()].size(); ++i) {
                            areaMean += recordTrajectory[ui_.cbArea->text()].at(i);
                        }
                    }

                    areaMean /= recordTrajectory[ui_.cbAspectRatio->text()].size();
                    aspectRatioMean /= recordTrajectory[ui_.cbAspectRatio->text()].size();

                    ui_.lblNTMean->setText(QString("area: %1 ; aspect ratio: %2").arg(areaMean, aspectRatioMean));

                    recordTrajectory[ui_.cbAspectRatio->text()].clear();
                    recordTrajectory[ui_.cbArea->text()].clear();
                    recordTrajectoryState = None;
                    break;
                case Abort:

                    recordTrajectory[ui_.cbAspectRatio->text()].clear();
                    recordTrajectory[ui_.cbArea->text()].clear();
                    recordTrajectoryState = None;
                    break;
                default:
                    break;
                }


                if(QDateTime::currentMSecsSinceEpoch() > lcdUpdateTimeStamp + 500){
                    ui_.lcdArea->display(QString::number(area, 'f', 3));
                    ui_.lcdAspectRatio->display(QString::number(hw,'f', 3));
                    ui_.lcdDistance->display(QString::number(depthCamTmp.at<float>(yDepth, xDepth), 'f', 3));
                    lcdUpdateTimeStamp = QDateTime::currentMSecsSinceEpoch();
                }

                if(printFeatures){
                    //ROS_INFO("MW:%f MH:%f W/H:%f H/W:%f Area:%f", mouthWidth, mouthHeight, wh, hw, areaOfTriangleA + areaOfTriangleB + areaOfTriangleC + areaOfTriangleD);
                    ROS_INFO("H/W:%f Area:%f", hw, area);
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
            utter = true;
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

            pixMap = imageProcessing.getPixmap(mt, useMonoImage);

            pixMap = pixMap.scaled(ui_.lblMouthDiffSum->maximumWidth(), ui_.lblMouthDiffSum->maximumHeight(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
            ui_.lblMouthDiffSum->setPixmap(pixMap);

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

QList<QList<double> > LipRec::getClusterTrajectories(QString command, QString feature, QString clusterMethod)
{
    QString clusterKey(QString("%1_%2_%3").arg(clusterMethod, command,feature));

    return clusterTrajectoriesOfCommand[clusterKey];
}

void LipRec::setClusterTrajectories(QList<QList<double> > clusterT, QString command, QString feature, QString clusterMethod)
{
    clusterTrajectoriesOfCommand[QString("%1_%2_%3").arg(clusterMethod, command, feature)] = clusterT;
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

        putText(mouthFeatures, "A", a, CV_FONT_HERSHEY_PLAIN, 0.5, Scalar(255,255,255));
        putText(mouthFeatures, "B", b, CV_FONT_HERSHEY_PLAIN, 0.5, Scalar(255,255,255));
        putText(mouthFeatures, "C", c, CV_FONT_HERSHEY_PLAIN, 0.5, Scalar(0,0,0));
        putText(mouthFeatures, "D", d, CV_FONT_HERSHEY_PLAIN, 0.5, Scalar(0,0,0));
    }

    QPixmap pixMap;
    bool monoImg = false;
    if(ui_.cbLips->isChecked()){

        if(mouthFeatures.type() == CV_8UC1){
            monoImg = true;
        }
        pixMap = imageProcessing.getPixmap(mouthFeatures, monoImg);

        pixMap = pixMap.scaled(ui_.lblMouthFeatureRaw->maximumWidth(), ui_.lblMouthFeatureRaw->maximumHeight(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
        ui_.lblMouthFeatureRaw->setPixmap(pixMap);
    }else{
        QPixmap empty;
        ui_.lblMouthFeatureRaw->setPixmap(empty);
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

void LipRec::clickedRecordStopTrajectory()
{
    if ((recordTrajectoryState == None || recordTrajectoryState == Save || recordTrajectoryState == Abort)
            && !ui_.pbSaveTrajectory->isEnabled()) {
        ui_.pbRecordStopTrajectory->setText("Stop recording");

        recordTrajectoryState = Recording;
    }else{
        ui_.pbRecordStopTrajectory->setText("Record trajectory");

        ui_.pbRecordStopTrajectory->setEnabled(false);
        ui_.pbSaveTrajectory->setEnabled(true);
        ui_.pbAbortTrajectory->setEnabled(true);
        recordTrajectoryState = None;
    }

}

void LipRec::clickedSaveTrajectory()
{
    QTimer::singleShot(1500, this, SLOT(clickedAbortOrSaveTrajectory()));
    QString fontColor = tr("<font color='%1'>%2</font>");
    ui_.lblMsg->setText( fontColor.arg( "red", "Saved" ) );
    ui_.lblMsg->setFont(QFont("Times New Roman", 10, QFont::Bold));
    ui_.pbRecordStopTrajectory->setEnabled(true);
    ui_.pbSaveTrajectory->setEnabled(false);
    ui_.pbAbortTrajectory->setEnabled(false);
    ui_.gbNewTrajectory->setEnabled(true);
    recordTrajectoryState = Save;
}

void LipRec::clickedAbortTrajectory()
{
    QTimer::singleShot(1500, this, SLOT(clickedAbortOrSaveTrajectory()));
    QString fontColor = tr("<font color='%1'>%2</font>");
    ui_.lblMsg->setText( fontColor.arg( "red", "Abort" ) );
    ui_.lblMsg->setFont(QFont("Times New Roman", 10, QFont::Bold));
    ui_.pbRecordStopTrajectory->setEnabled(true);
    ui_.pbSaveTrajectory->setEnabled(false);
    ui_.pbAbortTrajectory->setEnabled(false);
    recordTrajectoryState = Abort;
}

void LipRec::clickedAbortOrSaveTrajectory()
{
    ui_.lblMsg->setText("");
    ui_.gbNewTrajectory->setEnabled(false);
}

void LipRec::clickedCluster()
{

    QString clusterMethod;
    DistanceFunction df;

    if(ui_.rbKmedoids->isChecked()){
        clusterMethod = ui_.rbKmedoids->text();
    }else{
        clusterMethod = ui_.rbMosaCluster->text();
    }

    if(ui_.rbABS->isChecked()){
        df = ABS;
    }else if(ui_.rbSQUARE->isChecked()){
        df = SQUARE;
    }else{
        df = SQUARE2;
    }
    clustering.clearTrajectoriesSet();

    if(ui_.cbTrajectory->currentText() == "all"){

        QMap<QString, int> commandsWithCount = tdm.getAllCommandsWithCount();
        foreach (QString command, commandsWithCount.keys()) {

            if(ui_.cbArea->isChecked()){
                this->applyCluster(clusterMethod, df, command, ui_.cbArea->text());
            }

            clustering.clearTrajectoriesSet();

            if(ui_.cbAspectRatio->isChecked()){
                this->applyCluster(clusterMethod, df, command, ui_.cbAspectRatio->text());
            }

            clustering.clearTrajectoriesSet();
        }

    }else{

        if(ui_.cbArea->isChecked()){
            this->applyCluster(clusterMethod, df, ui_.cbTrajectory->currentText(), ui_.cbArea->text());
        }

        clustering.clearTrajectoriesSet();

        if(ui_.cbAspectRatio->isChecked()){
            this->applyCluster(clusterMethod, df, ui_.cbTrajectory->currentText(), ui_.cbAspectRatio->text());
        }
        clustering.clearTrajectoriesSet();

    }
}

void LipRec::applyCluster(QString clusterMethod, DistanceFunction df, QString command, QString feature){
    QList<QList<double> > clusterT = tdm.getClusterTrajectories(command, feature, clusterMethod);
    QList<QList<double> > traj = tdm.getTrajectory(command, feature);

    if(traj.size() > 0){
        clustering.addTrajectories(traj);

        if(ui_.rbKmedoids->isChecked()){
            clustering.setK(ui_.sbKMethod->value());
            traj = clustering.kMedoidsClustering(df);
        }else{
            traj = clustering.mosaClustering(df, ui_.sbKMethod->value());
        }

        this->setClusterTrajectories(traj, command, feature, clusterMethod);

        if(clusterT.size() > 0){
            tdm.updateClusterTrajectories(traj, command, feature, clusterMethod);
        }else{
            tdm.insertClusterTrajectories(traj, command, feature, clusterMethod);
        }
    }else{
        ROS_INFO("No trajectories found for %s:%s", command.toStdString().c_str(), feature.toStdString().c_str());
    }
}

void LipRec::printTrajectory(QList<double> trajectory)
{
    ROS_INFO(">>>Start printTrajectory");
    for (int i = 0; i < trajectory.size(); ++i) {
        ROS_INFO("%f", trajectory.at(i));
    }
    ROS_INFO(">>>End printTrajectory");
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
