#include "rqt_liprec/LipRec.h"
#include <pluginlib/class_list_macros.h>


namespace rqt_liprec {

LipRec::LipRec()
  : rqt_gui_cpp::Plugin()
  , widget_(0), blackBorder(0), last(0),  stateDetectionStartEndFrame(Idle), timeoutROIdetection(500)
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
    blackBorder = false;
    ui_.cbFaceROI->setChecked(false);
    ui_.cbMouthROI->setChecked(false);
    useMonoImage = false;
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
  //camImageDepth = getNodeHandle().subscribe("/kinect2/qhd/image_depth_rect", 100, &LipRec::imageDepthCallback, this);

  faceROISub = getNodeHandle().subscribe("/face_detection/faceROI", 10, &LipRec::faceROICallback, this);
  mouthROISub = getNodeHandle().subscribe("/face_detection/mouthROI", 10, &LipRec::mouthROICallback, this);

  NO_CYCLIC_FRAME = ui_.sbNOCF->value();

  QObject::connect(&faceROITimer, SIGNAL(timeout()), this, SLOT(faceROItimeout()));
  QObject::connect(&mouthROITimer, SIGNAL(timeout()), this, SLOT(mouthROItimeout()));

  QObject::connect(ui_.pbContinueVideo, SIGNAL(clicked()), this, SLOT(clickedContinueVideo()));

  QObject::connect(ui_.pbRecorderOptions, SIGNAL(clicked()), this, SLOT(clickedRecorderOptions()));

  QObject::connect(ui_.pbPrintFeatures, SIGNAL(clicked()), this, SLOT(clickedPrintFeatures()));

  QObject::connect(ui_.pbToggleKpLines, SIGNAL(clicked()), this, SLOT(toggleKpLines()));

  QObject::connect(ui_.pbRecordStopTrajectory, SIGNAL(clicked()), this, SLOT(clickedRecordStopTrajectory()));
  QObject::connect(ui_.pbSaveTrajectory, SIGNAL(clicked()), this, SLOT(clickedSaveTrajectory()));
  QObject::connect(ui_.pbAbortTrajectory, SIGNAL(clicked()), this, SLOT(clickedAbortTrajectory()));

  QObject::connect(ui_.pbCluster, SIGNAL(clicked()), this, SLOT(clickedCluster()));

  QObject::connect(ui_.pbUpdateRecognizedText, SIGNAL(clicked(bool)), this, SLOT(clickedUpdateRecognizedText(bool)));

  QObject::connect(ui_.pbRecordRecognition, SIGNAL(clicked(bool)), this, SLOT(clickedRecordRecognized(bool)));
  QObject::connect(ui_.pbSaveRecognition, SIGNAL(clicked()), this, SLOT(clickedSaveRecordRecognition()));
  QObject::connect(ui_.pbDeclineRecogntion, SIGNAL(clicked()), this, SLOT(clickedDeclineRecordRecognition()));

  QObject::connect(ui_.pbUtter, SIGNAL(clicked()), this, SLOT(clickedUtter()));


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

  ui_.pbRecorderOptions->setToolTip("Show options for recording results of the recognition.");
  pixmap = QPixmap("src/liprec/res/plot3.png");
  bi = QIcon(pixmap);
  ui_.pbRecorderOptions->setSizePolicy( QSizePolicy::Fixed, QSizePolicy::Preferred );
  ui_.pbRecorderOptions->setFlat(true);
  ui_.pbRecorderOptions->setIcon(bi);
  ui_.pbRecorderOptions->setIconSize(pixmap.rect().size());
  ui_.pbRecorderOptions->setMaximumSize(pixmap.rect().size());

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

  ui_.lcdArea->setDigitCount(15);
  ui_.lcdAspectRatio->setDigitCount(10);
  ui_.lcdDistance->setDigitCount(10);

  ui_.gbLipRecRecorder->setShown(false);

  lcdUpdateTimeStamp = QDateTime::currentMSecsSinceEpoch();

  utter = false;

  recordVideo = false;
  recordUtterance = false;
  loadUtterance = false;
  initVideoWriter = false;
  useCam = true;
  printFeatures = false;
  updateRecognizedText = true;
  recordRecognitionState = RRNONE;
  currentRecordRecognitionFilename = "";
  recordTrajectoryState = None;

  availableTrajectories << "all";


  tdm.connectToDatabase("localhost", "liprec", ui_.leCollection->text());
  tdm.setCollectionCluster("clustering");
  tdm.setCollection(ui_.leCollection->text());

  connect(ui_.leCollection, SIGNAL(textEdited(const QString&)), this, SLOT(lineEditChanged(const QString&)));

  ui_.lwTrajectoriesInfo->setStyleSheet("QListWidget::item {"
                                        "border-bottom: 1px solid black;"
                                        "padding-bottom: 2px;"
                                        "padding-top: 2px;"
                                         "}"
                                        "QListWidget:item:selected { background: #3555FF; }");
  font = QFont ("Courier");
  font.setStyleHint (QFont::Monospace);
  font.setPointSize (9);
  font.setFixedPitch (true);
  ui_.lwTrajectoriesInfo->setFont(font);
  ui_.lwTrajectoriesInfo->setWordWrap(true);

  this->updateClusterTrajectories();

  this->calculateAndInitWeightsForDTW();
  weightedDtwActive = false;
  ui_.lwDTWWeights->setStyleSheet(   "QListWidget::item {"
                                     "border-bottom: 6px solid black;"
                                     "border-color: black;"
                                     "margin-bottom: -5px;"
                                     "margin-top: -5px;"
                                  "}"
                                  "QListWidget::item:selected {"
                                     "background-color: #3555FF;"
                                  "}");
  QObject::connect(ui_.gbDynamicTimeWarping, SIGNAL(clicked(bool)), this, SLOT(clickedWeightedDtw(bool)));
  QObject::connect(ui_.sbDTWMaxWeightCoefficient, SIGNAL(valueChanged(int)), this, SLOT(spinBoxChanged(int)));

  this->updateTrajectoriesInfoGUIAndSetWeightsForDTW();

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

  tdm.setCollection(ui_.leCollection->text());

  NO_CYCLIC_FRAME = ui_.sbNOCF->value();

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
  int currentFrame = 0;
  currentFrame = updateFrameBuffer(rawMouthImg);

  if(!rawMouthImg.empty()){
    this->lipsActivation(currentFrame);
  }

  DistanceFunction df;
  if(ui_.rbABS->isChecked()){
    df = ABS;
  }else if(ui_.rbSQUARE->isChecked()){
    df = SQUARE;
  }else{
    df = SQUARE2;
  }

  if(ui_.cbDtwWindowSizeAdaptable->isChecked()){
    ui_.cbDtwWindowSizeActivate->setChecked(true);
  }

  Mat depthCamTmp;
  int xDepth = 0;
  int yDepth = 0;
  depthCamMtx.lock();
  if(!depthCam.empty()){
    depthCam.copyTo(depthCamTmp);
    int xDepth = faceROI.x_offset+(faceROI.width/2);
    int yDepth = faceROI.y_offset+(faceROI.height*0.16);

    circle(img, Point(xDepth, yDepth), 2, Scalar(255,255,255));

    pixMap = imageProcessing.getPixmap(img, useMonoImage);
    ui_.lbl_cam->setPixmap(pixMap);
  }
  depthCamTmp.convertTo(depthCamTmp, CV_16U);
  depthCamMtx.unlock();

  int windowSize = ui_.sbDtwWindowSize->value();

  if(!useMonoImage && !mouthImg.empty() && recordRecognitionState != RRDECISION){
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
        //double wh = mouthWidth / mouthHeight;
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

        uint16_t distanceNormalized = 0;
        if(!depthCamTmp.empty())
          distanceNormalized = depthCamTmp.at<uint16_t>(yDepth, xDepth);

        if(distanceNormalized > 645){
          distanceNormalized = 645;
        }else if(distanceNormalized < 500){
          distanceNormalized = 500;
        }

        uint relativeArea = area;
        //distanceNormalized /= 10; // mm to cm
        //distanceNormalized = (distanceNormalized - 520)/(655-520);

        bool loadUtteranceFile = ui_.leFilenameLoadRecord->text() != "" && ui_.cbLoadUtteranceFile->isChecked();

        if(loadUtteranceFile){
          currentUtteranceTrajectories.clear();
          currentUtteranceTrajectories = lipRecRecorder.readUtteranceFromTextFile(ui_.leFilenameLoadRecord->text());
        }

        bool activateAlgorithmForRecognition = utter && stateDetectionStartEndFrame == Idle && recordTrajectoryState != Recording && !loadUtteranceFile;

        if(activateAlgorithmForRecognition || (loadUtteranceFile && applyUtteranceOfLoadedFile)){
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

          DtwStepPattern stepPattern = TYPE1;
          QString currentStepPattern = ui_.cbDTWStepPattern->currentText();
          if(currentStepPattern == "TYPE1"){
            stepPattern = TYPE1;
          }else if(currentStepPattern == "TYPE2"){
            stepPattern = TYPE2;
          }else if(currentStepPattern == "TYPE3"){
            stepPattern = TYPE3;
          }else if(currentStepPattern == "TYPE4"){
            stepPattern = TYPE4;
          }else if(currentStepPattern == "TYPE5"){
            stepPattern = TYPE5;
          }else{
            stepPattern = ITAKURA;
          }

          if(currentUtteranceTrajectories.size() > 0){

            recordRecognitionData.clear();

            CommandWithCost commandWithCost;

            if(ui_.rbDTWSA->isChecked()){
              if(ui_.rbSingleFF->isChecked()){

                QList<CommandWithCost> areaCommandsWithCost;
                QList<CommandWithCost> aspectRatioCommandsWithCost;

                foreach (QString command, availableTrajectories) {
                  clusterT = this->getClusterTrajectories(command, ui_.cbArea->text(), ui_.rbKmedoids->text());
                  double localCost = INT_MAX;

                  double warpingCostTmpArea = 0.0;
                  for (int i = 0; i < clusterT.size(); i++) {

                    dtw.seed(clusterT.at(i), currentUtteranceTrajectories[ui_.cbArea->text()], stepPattern, ui_.cbDtwSlopeWeights->isChecked());

                    warpingCostTmpArea = dtw.calcWarpingCost(df, ui_.cbDtwWindowSizeActivate->isChecked(), windowSize, ui_.cbDtwWindowSizeAdaptable->isChecked());

                    if(weightedDtwActive){
                      warpingCostTmpArea += warpingCostTmpArea * weightedDtw[command];
                    }

                    if(warpingCostTmpArea < localCost){
                      localCost = warpingCostTmpArea;
                      commandWithCost.command = command;
                      commandWithCost.cost = warpingCostTmpArea;
                    }

                    if(!ui_.cbSOLR->isChecked()){
                      recordRecognitionData.commandArea[command].append(warpingCostTmpArea);
                    }

                    //                    ROS_INFO("Area Command: %s(%d) ; Utterrance: %d -> %f",
                    //                             command.toStdString().c_str(), clusterT.at(i).size(), currentUtteranceTrajectories[ui_.cbArea->text()].size(), warpingCostTmpArea);

                    if(warpingCostTmpArea < bestWarpingCostArea){
                      indexOfLowAreaCluster = i;
                      bestWarpingCostArea = warpingCostTmpArea;
                      currentCommandArea = command;
                    }
                  }

                  if(commandWithCost.command != ""){
                    areaCommandsWithCost.append(commandWithCost);

                    if(ui_.cbSOLR->isChecked()){
                      recordRecognitionData.commandArea[commandWithCost.command].append(commandWithCost.cost);
                    }
                    commandWithCost.command = "";
                  }

                  localCost = INT_MAX;

                  clusterT = this->getClusterTrajectories(command, ui_.cbAspectRatio->text(), ui_.rbKmedoids->text());

                  double warpingCostTmpAspectRatio = 0.0;
                  for (int i = 0; i < clusterT.size(); i++) {
                    dtw.seed(clusterT.at(i), currentUtteranceTrajectories[ui_.cbAspectRatio->text()], stepPattern, ui_.cbDtwSlopeWeights->isChecked());

                    warpingCostTmpAspectRatio = dtw.calcWarpingCost(df, ui_.cbDtwWindowSizeActivate->isChecked(), windowSize, ui_.cbDtwWindowSizeAdaptable->isChecked());

                    if(weightedDtwActive){
                      warpingCostTmpAspectRatio += warpingCostTmpAspectRatio * weightedDtw[command];
                    }

                    if(warpingCostTmpAspectRatio < localCost){
                      localCost = warpingCostTmpArea;
                      commandWithCost.command = command;
                      commandWithCost.cost = warpingCostTmpAspectRatio;
                    }

                    if(!ui_.cbSOLR->isChecked()){
                      recordRecognitionData.commandAspectRatio[command].append(warpingCostTmpAspectRatio);
                    }

                    //                    ROS_INFO("AspectRatio Command: %s(%d) ; Utterrance: %d -> %f",
                    //                             command.toStdString().c_str(), clusterT.at(i).size(), currentUtteranceTrajectories[ui_.cbAspectRatio->text()].size(), warpingCostTmpAspectRatio);

                    if(warpingCostTmpAspectRatio < bestWarpingCostAspectRatio){
                      indexOfLowAspectRatioCluster = i;
                      bestWarpingCostAspectRatio = warpingCostTmpAspectRatio;
                      currentCommandAspectRatio = command;
                    }
                  }

                  if(commandWithCost.command != ""){
                    aspectRatioCommandsWithCost.append(commandWithCost);

                    if(ui_.cbSOLR->isChecked()){
                      recordRecognitionData.commandAspectRatio[commandWithCost.command].append(commandWithCost.cost);
                    }
                    commandWithCost.command = "";
                  }
                }

                QPixmap dtwPixMap = this->drawDTWPixmap(currentCommandArea, ui_.cbArea->text(), indexOfLowAreaCluster, ui_.rbKmedoids->text(), df, stepPattern);
                dtwPixMap = dtwPixMap.scaled(ui_.lblDTW->maximumWidth(), ui_.lblDTW->maximumHeight(), Qt::KeepAspectRatio, Qt::SmoothTransformation);

                ui_.lblDTW->setPixmap(dtwPixMap);

                dtwPixMap = this->drawDTWPixmap(currentCommandAspectRatio, ui_.cbAspectRatio->text(), indexOfLowAspectRatioCluster, ui_.rbKmedoids->text(), df, stepPattern);
                dtwPixMap = dtwPixMap.scaled(ui_.lblMouthDiff->maximumWidth(), ui_.lblMouthDiff->maximumHeight(), Qt::KeepAspectRatio, Qt::SmoothTransformation);

                ui_.lblMouthDiff->setPixmap(dtwPixMap);

                if(updateRecognizedText){
                  ui_.lwArea->clear();
                  ui_.lwAspectRatio->clear();
                  ui_.labelArea->setText("Area");
                  ui_.labelAspectRatio->setText("Aspect ratio");

                  qSort(areaCommandsWithCost);
                  qSort(aspectRatioCommandsWithCost);

                  for (int i = 0; i < areaCommandsWithCost.size(); ++i) {
                    commandWithCost.command = areaCommandsWithCost.at(i).command;
                    commandWithCost.cost = areaCommandsWithCost.at(i).cost;
                    ui_.lwArea->addItem(QString("%1: %2").arg(commandWithCost.command, -14).arg(commandWithCost.cost, 8));
                  }

                  for (int i = 0; i < aspectRatioCommandsWithCost.size(); ++i) {
                    commandWithCost.command = aspectRatioCommandsWithCost.at(i).command;
                    commandWithCost.cost = aspectRatioCommandsWithCost.at(i).cost;
                    ui_.lwAspectRatio->addItem(QString("%1 : %2").arg(commandWithCost.command, -14).arg(commandWithCost.cost, 8));
                  }
                }

                this->changeRecordRecognitionStateToDecisionIfPossible();

                //                ROS_INFO("Recognize Area: %s", currentCommandArea.toStdString().c_str());
                //                ROS_INFO("Recognize AspectRatio: %s", currentCommandAspectRatio.toStdString().c_str());

              }else if(ui_.rbFusionFF->isChecked()){

                int fusionAreaIndex = 0;
                int fusionAspectRatioIndex = 0;

                QList<CommandWithCost> commandsWithCost;

                foreach (QString command, availableTrajectories) {

                  clusterT = this->getClusterTrajectories(command, ui_.cbArea->text(), ui_.rbKmedoids->text());
                  clusterT2 = this->getClusterTrajectories(command, ui_.cbAspectRatio->text(), ui_.rbKmedoids->text());

                  double warpingCostTmpArea = INT_MAX;
                  double warpingCostTmpAspectRatio = INT_MAX;
                  double warpingCostFusionTmp = 0.0;

                  for (int i = 0; i < clusterT.size(); ++i) {
                    dtw.seed(clusterT.at(i), currentUtteranceTrajectories[ui_.cbArea->text()], stepPattern, ui_.cbDtwSlopeWeights->isChecked());

                    double wpArea = dtw.calcWarpingCost(df, ui_.cbDtwWindowSizeActivate->isChecked(), windowSize, ui_.cbDtwWindowSizeAdaptable->isChecked());

                    if(weightedDtwActive){
                      wpArea += wpArea * weightedDtw[command];
                    }

                    //                    ROS_INFO("Area Command: %s(%d) ; Utterrance: %d -> %f",
                    //                             command.toStdString().c_str(), clusterT.at(i).size(),
                    //                             currentUtteranceTrajectories[ui_.cbArea->text()].size(), wpArea);

                    if(wpArea < warpingCostTmpArea){
                      indexOfLowAreaCluster = i;
                      warpingCostTmpArea = wpArea;
                    }
                  }

                  for (int i = 0; i < clusterT2.size(); ++i) {
                    dtw.seed(clusterT2.at(i), currentUtteranceTrajectories[ui_.cbAspectRatio->text()], stepPattern, ui_.cbDtwSlopeWeights->isChecked());

                    double wpAspectRatio = 0.0;

                    wpAspectRatio = dtw.calcWarpingCost(df, ui_.cbDtwWindowSizeActivate->isChecked(), windowSize, ui_.cbDtwWindowSizeAdaptable->isChecked());

                    if(weightedDtwActive){
                      wpAspectRatio += wpAspectRatio * weightedDtw[command];
                    }

                    //                    ROS_INFO("Aspect Ratio Command: %s(%d) ; Utterrance: %d -> %f",
                    //                             command.toStdString().c_str(), clusterT2.at(i).size(),
                    //                             currentUtteranceTrajectories[ui_.cbAspectRatio->text()].size(), wpAspectRatio);

                    if(wpAspectRatio < warpingCostTmpAspectRatio){
                      indexOfLowAspectRatioCluster = i;
                      warpingCostTmpAspectRatio = wpAspectRatio;
                    }
                  }

                  warpingCostFusionTmp = warpingCostTmpArea + warpingCostTmpAspectRatio;

                  if(command != "all"){
                    commandWithCost.command = command;
                    commandWithCost.cost = warpingCostFusionTmp;
                    commandsWithCost.append(commandWithCost);
                  }

                  recordRecognitionData.commandFusion[commandWithCost.command].append(commandWithCost.cost);

                  if(warpingCostFusionTmp < bestWarpingCostFusion){
                    bestWarpingCostFusion = warpingCostFusionTmp;
                    fusionAreaIndex = indexOfLowAreaCluster;
                    fusionAspectRatioIndex = indexOfLowAspectRatioCluster;
                    currentCommandFusion = command;
                  }
                }

                QPixmap dtwPixMap = this->drawDTWPixmap(currentCommandFusion, ui_.cbArea->text(), fusionAreaIndex, ui_.rbKmedoids->text(), df, stepPattern);
                dtwPixMap = dtwPixMap.scaled(ui_.lblDTW->maximumWidth(), ui_.lblDTW->maximumHeight(), Qt::KeepAspectRatio, Qt::SmoothTransformation);

                ui_.lblDTW->setPixmap(dtwPixMap);

                dtwPixMap = this->drawDTWPixmap(currentCommandFusion, ui_.cbAspectRatio->text(), fusionAspectRatioIndex, ui_.rbKmedoids->text(), df, stepPattern);
                dtwPixMap = dtwPixMap.scaled(ui_.lblMouthDiff->maximumWidth(), ui_.lblMouthDiff->maximumHeight(), Qt::KeepAspectRatio, Qt::SmoothTransformation);

                ui_.lblMouthDiff->setPixmap(dtwPixMap);

                if(updateRecognizedText){
                  ui_.lwArea->clear();
                  ui_.lwAspectRatio->clear();
                  ui_.labelArea->setText("Fusion (Area + Aspect ratio)");
                  ui_.labelAspectRatio->setText("");

                  qSort(commandsWithCost);
                  for (int i = 0; i < commandsWithCost.size(); ++i) {
                    commandWithCost.command = commandsWithCost.at(i).command;
                    commandWithCost.cost = commandsWithCost.at(i).cost;
                    ui_.lwArea->addItem(QString("%1: %2").arg(commandWithCost.command, -14).arg(commandWithCost.cost, 8));
                  }
                }

                this->changeRecordRecognitionStateToDecisionIfPossible();

                //                ROS_INFO("Recognize Fusion: %s", currentCommandFusion.toStdString().c_str());
              }

            }else if(ui_.rbEuclideanDistSA->isChecked()){

              double bestEuclideanDistanceArea = INT_MAX;
              double bestEuclideanDistanceAspectRatio = INT_MAX;

              QList<CommandWithCost> areaCommandsWithCost;
              QList<CommandWithCost> aspectRatioCommandsWithCost;

              foreach (QString command, availableTrajectories) {
                double localCost = INT_MAX;
                clusterT = this->getClusterTrajectories(command, ui_.cbArea->text(), ui_.rbKmedoids->text());

                double euclideanDistanceTmpArea = 0.0;
                for (int i = 0; i < clusterT.size(); i++) {
                  QList<double> shortenTrajectory;
                  QList<double> keptTrajectory;
                  if(currentUtteranceTrajectories[ui_.cbArea->text()].size() > clusterT.at(i).size()){
                    keptTrajectory = clusterT.at(i);
                    shortenTrajectory = currentUtteranceTrajectories[ui_.cbArea->text()].mid(0, clusterT.at(i).size());
                    //                    shortenTrajectory = currentUtteranceTrajectories[ui_.cbArea->text()].mid(
                    //                          currentUtteranceTrajectories[ui_.cbArea->text()].size()-clusterT.at(i).size(), clusterT.at(i).size());
                  }else if(currentUtteranceTrajectories[ui_.cbArea->text()].size() < clusterT.at(i).size()){
                    keptTrajectory = currentUtteranceTrajectories[ui_.cbArea->text()];
                    shortenTrajectory = clusterT.at(i).mid(0, currentUtteranceTrajectories[ui_.cbArea->text()].size());
                    //                    shortenTrajectory = clusterT.at(i).mid(
                    //                          clusterT.at(i).size()-currentUtteranceTrajectories[ui_.cbArea->text()].size(), currentUtteranceTrajectories[ui_.cbArea->text()].size());
                  }else{
                    shortenTrajectory = currentUtteranceTrajectories[ui_.cbArea->text()];
                    keptTrajectory = clusterT.at(i);
                  }

                  euclideanDistanceTmpArea = calculateEuclideanDistance(keptTrajectory, shortenTrajectory);

                  if(euclideanDistanceTmpArea < localCost){
                    localCost = euclideanDistanceTmpArea;
                    commandWithCost.command = command;
                    commandWithCost.cost = euclideanDistanceTmpArea;
                  }

                  if(!ui_.cbSOLR->isChecked()){
                    recordRecognitionData.commandArea[command].append(euclideanDistanceTmpArea);
                  }

                  //                  ROS_INFO("Area Command: %s(%d) ; Utterrance: %d -> %f",
                  //                           command.toStdString().c_str(), clusterT.at(i).size(), currentUtteranceTrajectories[ui_.cbArea->text()].size(), euclideanDistanceTmpArea);

                  if(euclideanDistanceTmpArea < bestEuclideanDistanceArea){
                    indexOfLowAreaCluster = i;
                    bestEuclideanDistanceArea = euclideanDistanceTmpArea;
                    currentCommandArea = command;
                  }
                }

                if(commandWithCost.command != ""){
                  areaCommandsWithCost.append(commandWithCost);
                  if(ui_.cbSOLR->isChecked()){
                    recordRecognitionData.commandArea[commandWithCost.command].append(commandWithCost.cost);
                  }
                  commandWithCost.command = "";
                }

                clusterT = this->getClusterTrajectories(command, ui_.cbAspectRatio->text(), ui_.rbKmedoids->text());
                localCost = INT_MAX;

                double euclideanDistanceTmpAspectRatio = 0.0;
                for (int i = 0; i < clusterT.size(); i++) {
                  QList<double> shortenTrajectory;
                  QList<double> keptTrajectory;
                  if(currentUtteranceTrajectories[ui_.cbAspectRatio->text()].size() > clusterT.at(i).size()){
                    keptTrajectory = clusterT.at(i);
                    shortenTrajectory = currentUtteranceTrajectories[ui_.cbAspectRatio->text()].mid(0, clusterT.at(i).size());
                    //                    shortenTrajectory = currentUtteranceTrajectories[ui_.cbAspectRatio->text()].mid(
                    //                          currentUtteranceTrajectories[ui_.cbAspectRatio->text()].size()-clusterT.at(i).size(), clusterT.at(i).size());
                  }else if(currentUtteranceTrajectories[ui_.cbAspectRatio->text()].size() < clusterT.at(i).size()){
                    keptTrajectory = currentUtteranceTrajectories[ui_.cbAspectRatio->text()];
                    shortenTrajectory = clusterT.at(i).mid(0, currentUtteranceTrajectories[ui_.cbAspectRatio->text()].size());
                    //                    shortenTrajectory = clusterT.at(i).mid(
                    //                          clusterT.at(i).size()-currentUtteranceTrajectories[ui_.cbAspectRatio->text()].size(), currentUtteranceTrajectories[ui_.cbAspectRatio->text()].size());
                  }else{
                    shortenTrajectory = currentUtteranceTrajectories[ui_.cbAspectRatio->text()];
                    keptTrajectory = clusterT.at(i);
                  }

                  euclideanDistanceTmpAspectRatio = calculateEuclideanDistance(keptTrajectory, shortenTrajectory);

                  if(euclideanDistanceTmpAspectRatio < localCost){
                    localCost = euclideanDistanceTmpAspectRatio;
                    commandWithCost.command = command;
                    commandWithCost.cost = euclideanDistanceTmpAspectRatio;
                  }

                  if(!ui_.cbSOLR->isChecked()){
                    recordRecognitionData.commandAspectRatio[command].append(euclideanDistanceTmpAspectRatio);
                  }

                  //                  ROS_INFO("Aspect Ratio Command: %s(%d) ; Utterrance: %d -> %f",
                  //                           command.toStdString().c_str(), clusterT.at(i).size(), currentUtteranceTrajectories[ui_.cbAspectRatio->text()].size(), euclideanDistanceTmpAspectRatio);

                  if(euclideanDistanceTmpAspectRatio < bestEuclideanDistanceAspectRatio){
                    indexOfLowAreaCluster = i;
                    bestEuclideanDistanceAspectRatio = euclideanDistanceTmpAspectRatio;
                    currentCommandAspectRatio = command;
                  }
                }

                if(commandWithCost.command != ""){
                  aspectRatioCommandsWithCost.append(commandWithCost);
                  if(ui_.cbSOLR->isChecked()){
                    recordRecognitionData.commandAspectRatio[commandWithCost.command].append(commandWithCost.cost);
                  }
                  commandWithCost.command = "";
                }

              }

              if(updateRecognizedText){
                ui_.lwArea->clear();
                ui_.lwAspectRatio->clear();
                ui_.labelArea->setText("Area");
                ui_.labelAspectRatio->setText("Aspect ratio");

                qSort(areaCommandsWithCost);
                qSort(aspectRatioCommandsWithCost);

                for (int i = 0; i < areaCommandsWithCost.size(); ++i) {
                  commandWithCost.command = areaCommandsWithCost.at(i).command;
                  commandWithCost.cost = areaCommandsWithCost.at(i).cost;
                  ui_.lwArea->addItem(QString("%1: %2").arg(commandWithCost.command, -14).arg(commandWithCost.cost, 8));
                }

                for (int i = 0; i < aspectRatioCommandsWithCost.size(); ++i) {
                  commandWithCost.command = aspectRatioCommandsWithCost.at(i).command;
                  commandWithCost.cost = aspectRatioCommandsWithCost.at(i).cost;
                  ui_.lwAspectRatio->addItem(QString("%1 : %2").arg(commandWithCost.command, -14).arg(commandWithCost.cost, 8));
                }
              }

              this->changeRecordRecognitionStateToDecisionIfPossible();

              //              ROS_INFO("Recognize Area: %s", currentCommandArea.toStdString().c_str());
              //              ROS_INFO("Recognize AspectRatio: %s", currentCommandAspectRatio.toStdString().c_str());
            }
          }

          int utteranceLength = currentUtteranceTrajectories[ui_.cbAspectRatio->text()].size();

          if(updateRecognizedText){
            ui_.labelUtteranceLength->setText(QString::number(utteranceLength));
          }

          if(recordRecognitionState != RRDECISION){
            currentUtteranceTrajectories.clear();
          }
          utter = false;
          applyUtteranceOfLoadedFile = false;

        }else if(utter && recordTrajectoryState != Recording){
          currentUtteranceTrajectories[ui_.cbArea->text()].append(relativeArea);
          currentUtteranceTrajectories[ui_.cbAspectRatio->text()].append(hw);
        }

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

          ui_.lblNTMean->setText(QString("area: %1;\naspectRatio: %2").arg(areaMean).arg(aspectRatioMean));

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
          //ROS_INFO("Distance to cam %f * Area: %f -> %f", distanceNormalized, area, relativeArea);
          ui_.lcdArea->display(QString::number(relativeArea, 'f', 3));
          ui_.lcdAspectRatio->display(QString::number(hw,'f', 3));
          ui_.lcdDistance->display(QString::number(distanceNormalized, 'f', 3));
          lcdUpdateTimeStamp = QDateTime::currentMSecsSinceEpoch();
        }

        if(printFeatures){
          //ROS_INFO("MW:%f MH:%f W/H:%f H/W:%f Area:%f", mouthWidth, mouthHeight, wh, hw, areaOfTriangleA + areaOfTriangleB + areaOfTriangleC + areaOfTriangleD);
          ROS_INFO("H/W:%f Area:%f", hw, area);
        }

        this->drawMouthFeatures(mouthFeatures, keyPoint1, keyPoint2, keyPoint3, keyPoint4, keyPoint5, keyPoint6);
        showMouthImg = this->drawMouthFeaturesOnGUI(mouthImg, rLowFinal, rMidFinal, rTopFinal,
                                                    upLinePoint, bottomLinePoint, rightLinePoint,
                                                    keyPoint1, keyPoint2, keyPoint3, keyPoint4, keyPoint5, keyPoint6);
      }
    }
  }

  this->showLips(showMouthImg);

  last = currentFrame;
}

QPixmap LipRec::drawDTWPixmap(QString currentCommand, QString feature, int indexOfLowCluster, QString clusterMethod, DistanceFunction df, DtwStepPattern stepPattern){
  QPixmap dtwPixMap;
  QList<QList<double> > clusterT = this->getClusterTrajectories(currentCommand, feature, clusterMethod);

  int windowSize = ui_.sbDtwWindowSize->value();

  if(!clusterT.isEmpty()){
    dtw.seed(clusterT.at(indexOfLowCluster), currentUtteranceTrajectories[feature], stepPattern, ui_.cbDtwSlopeWeights->isChecked());

    dtw.calcWarpingCost(df, ui_.cbDtwWindowSizeActivate->isChecked(), windowSize, ui_.cbDtwWindowSizeAdaptable->isChecked());

    Mat dtwMat;
    Mat dtwMat2(dtw.getDtwDistanceMatrix().rows-1, dtw.getDtwDistanceMatrix().cols-1, CV_64F);

    //dtwMat.convertTo(dtwMat, CV_8U);
    double tmpDtwDistance = 0.0;
    for (int i = 1; i < dtw.getDtwDistanceMatrix().cols; ++i) {
      for (int j = 1; j < dtw.getDtwDistanceMatrix().rows; ++j) {
        tmpDtwDistance = dtw.getDtwDistanceMatrix().at<double>(j,i);
        if(tmpDtwDistance >= INT_MAX){
          tmpDtwDistance = 0.0;
        }
        dtwMat2.at<double>(j-1,i-1) = tmpDtwDistance;
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

      if(dtw.getWarpingPath().at(i).y < dtw.getDtwDistanceMatrix().rows && dtw.getWarpingPath().at(i).x < dtw.getDtwDistanceMatrix().cols){
        sum += dtw.getDtwDistanceMatrix().at<double>(dtw.getWarpingPath().at(i));
      }
    }

    //ROS_INFO("SUM %f", sum);

    dtwMat = 255 - dtwMat;

    dtwPixMap = imageProcessing.getPixmap(dtwMat, true);
  }

  return dtwPixMap;
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

void LipRec::lineEditChanged(const QString &str)
{
  tdm.setCollection(ui_.leCollection->text());
  availableTrajectories.clear();
  availableTrajectories << "all";
  this->updateClusterTrajectories();
  this->updateTrajectoriesInfoGUIAndSetWeightsForDTW();
}

void LipRec::spinBoxChanged(int value)
{
  tdm.setCollection(ui_.leCollection->text());
  availableTrajectories.clear();
  availableTrajectories << "all";
  this->updateClusterTrajectories();
  this->calculateAndInitWeightsForDTW();
  this->updateTrajectoriesInfoGUIAndSetWeightsForDTW();
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

        //Size size = Size(imageAbsDiff.size().width, imageAbsDiff.size().height);
        Mat mt(imageAbsDiff.rows, imageAbsDiff.cols, CV_8UC1, Scalar(-1));

        //2. take max pixel intensity value
        for (int i = 0; i < utterance.size(); ++i) {
          if(!utterance.at(i).empty()){

            for (int k = 0; k < utterance.at(i).cols; ++k) {
              for (int j = 0; j < utterance.at(i).rows; ++j) {
                if(mt.at<uchar>(j,k) < utterance.at(i).at<uchar>(j,k)){
                  mt.at<uchar>(j,k) = utterance.at(i).at<uchar>(j,k);
                }
              }
            }
          }
        }

        //imageProcessing.squareImage(mt);

        if(mt.at<double>(0,0) != -1){
          pixMap = imageProcessing.getPixmap(mt, useMonoImage);

          pixMap = pixMap.scaled(ui_.lblMouthDiffSum->maximumWidth(), ui_.lblMouthDiffSum->maximumHeight(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
          ui_.lblMouthDiffSum->setPixmap(pixMap);
        }
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

double LipRec::calculateEuclideanDistance(QList<double> &trj1, QList<double> &trj2)
{

  double euclideanDistance = 0.0;
  for (int i = 0; i < trj1.size(); ++i) {
    euclideanDistance += ((trj1.at(i)-trj2.at(i))*(trj1.at(i)-trj2.at(i)));
  }

  sqrt(euclideanDistance);

  return euclideanDistance;
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

void LipRec::clickedContinueVideo()
{
  this->loadUtterance = false;

  this->changeUseCam();
}

void LipRec::clickedRecorderOptions()
{
  if(ui_.gbLipRecRecorder->isHidden()){
    ui_.gbLipRecRecorder->setShown(true);
  }else{
    ui_.gbLipRecRecorder->setShown(false);
  }

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


    QString fontColor = tr("<font color='%1'>%2</font>");
    ui_.lblMsg->setText( fontColor.arg( "red", QString::number(recordTrajectory[ui_.cbArea->text()].size())));
    ui_.lblMsg->setFont(QFont("Times New Roman", 10, QFont::Bold));

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

void LipRec::clickedUpdateRecognizedText(bool checked)
{
  if(checked){
    ui_.pbUpdateRecognizedText->setText("Continue");
    updateRecognizedText = false;
  }else{
    ui_.pbUpdateRecognizedText->setText("Freeze");
    updateRecognizedText = true;
  }
}


void LipRec::clickedSaveRecordRecognition(){
  if(recordRecognitionState == RRDECISION){

    QTimer::singleShot(1500, this, SLOT(clickedDeclineOrSaveRecordRecognition()));
    QString fontColor = tr("<font color='%1'>%2</font>");
    ui_.lblMsgRecognition->setText( fontColor.arg( "red", "Save" ) );
    ui_.lblMsgRecognition->setFont(QFont("Times New Roman", 10, QFont::Bold));

    ui_.pbSaveRecognition->setEnabled(false);
    ui_.pbDeclineRecogntion->setEnabled(false);
    ui_.pbRecordRecognition->setText("Record");
    ui_.pbRecordRecognition->setChecked(false);

    if(ui_.cbSaveUtterance->isChecked()){
      lipRecRecorder.writeUtteranceToTextFile(ui_.leFilenameRecord->text(), currentUtteranceTrajectories);
    }

    if(ui_.cbContinueAppending->isChecked() || currentRecordRecognitionFilename == ui_.leFilenameRecord->text()){
      lipRecRecorder.writeToTextFile(ui_.leFilenameRecord->text(), recordRecognitionData);
    }else{
      currentRecordRecognitionFilename = ui_.leFilenameRecord->text();
      lipRecRecorder.writeHeaderToTextFile(ui_.leFilenameRecord->text(), recordRecognitionData);
      lipRecRecorder.writeToTextFile(ui_.leFilenameRecord->text(), recordRecognitionData);
    }

    currentUtteranceTrajectories.clear();

    recordRecognitionState = RRNONE;
  }
}

void LipRec::clickedDeclineRecordRecognition(){
  if(recordRecognitionState == RRDECISION){

    QTimer::singleShot(1500, this, SLOT(clickedDeclineOrSaveRecordRecognition()));
    QString fontColor = tr("<font color='%1'>%2</font>");
    ui_.lblMsgRecognition->setText( fontColor.arg( "red", "Decline" ) );
    ui_.lblMsgRecognition->setFont(QFont("Times New Roman", 10, QFont::Bold));

    ui_.pbSaveRecognition->setEnabled(false);
    ui_.pbDeclineRecogntion->setEnabled(false);
    ui_.pbRecordRecognition->setText("Record");
    ui_.pbRecordRecognition->setChecked(false);

    recordRecognitionState = RRNONE;
  }
}

void LipRec::clickedDeclineOrSaveRecordRecognition()
{
  ui_.lblMsgRecognition->setText("");
}

void LipRec::clickedUtter()
{
  this->applyUtteranceOfLoadedFile = true;
}

void LipRec::clickedWeightedDtw(bool checked)
{
  if(!checked){
    ui_.gbDynamicTimeWarping->setChecked(true);
  }
  if(weightedDtwActive){
    ui_.gbDynamicTimeWarping->setTitle("Dynamic-time-warping");
    ui_.gbDynamicTimeWarping->setToolTip("Change to weighted DTW");

    ui_.gbDynamicTimeWarping->setStyleSheet(
"          QGroupBox#gbDynamicTimeWarping {"
"              border: 1px solid black;"
"            font-weight: bold;"
"           }"

"          QGroupBox::title#gbDynamicTimeWarping{"
"            background-color: transparent;"
"              subcontrol-position: top left;"
"              padding: 1px;"
"          }"
"           QGroupBox::indicator#gbDynamicTimeWarping {"
          "        width: 27px;"
          "        height: 27px;"
         " padding-right: 80px;"
         " padding-left: 13;"
          "      image: url(noWeightSelect.png)"
          "    }"

          "    QGroupBox::indicator:checked:hover#gbDynamicTimeWarping {"
          "      image: url(weight.png)"
          "    }"

     );

  }else{
    ui_.gbDynamicTimeWarping->setTitle("Weighted Dynamic-time-warping");
    ui_.gbDynamicTimeWarping->setToolTip("Change to normal DTW");

    ui_.gbDynamicTimeWarping->setStyleSheet(
"          QGroupBox#gbDynamicTimeWarping { "
"              border: 1px solid black; "
"            font-weight: bold;"
"           } "

"          QGroupBox::title#gbDynamicTimeWarping{ "
"            background-color: transparent;"
"              subcontrol-position: top left;"
"              padding: 1px;"
"          } "

          "QGroupBox::indicator#gbDynamicTimeWarping {"
          "       width: 27px;"
          "       height: 27px;"
"          padding-right: 80px;"
         " padding-left: 13;"
          "       image: url(weightSelect.png)"
          "    }"

          "    QGroupBox::indicator:checked:hover#gbDynamicTimeWarping {"
          "      image: url(noWeight.png)"
          "    }"

     );
  }
  weightedDtwActive = !weightedDtwActive;
}

void LipRec::clickedRecordRecognized(bool checked)
{
  if(checked){
    if(!ui_.leFilenameRecord->text().isEmpty()){
      recordRecognitionState = RRRECORDING;
      ui_.pbRecordRecognition->setText("Stop Recording");
    }else{
      ui_.pbRecordRecognition->setChecked(false);
    }
  }else{
    if(recordRecognitionState == RRRECORDING || recordRecognitionState == RRDECISION){
      ui_.pbRecordRecognition->setChecked(true);
    }else{
      ui_.pbRecordRecognition->setText("Record");
    }
  }
}



void LipRec::applyCluster(QString clusterMethod, DistanceFunction df, QString command, QString feature){
  QList<QList<double> > clusterT = tdm.getClusterTrajectories(command, feature, clusterMethod);
  QList<QList<double> > traj = tdm.getTrajectory(command, feature);

  DtwStepPattern stepPattern = TYPE1;
  QString currentStepPattern = ui_.cbDTWStepPattern->currentText();
  if(currentStepPattern == "TYPE1"){
    stepPattern = TYPE1;
  }else if(currentStepPattern == "TYPE2"){
    stepPattern = TYPE2;
  }else if(currentStepPattern == "TYPE3"){
    stepPattern = TYPE3;
  }else if(currentStepPattern == "TYPE4"){
    stepPattern = TYPE4;
  }else if(currentStepPattern == "TYPE5"){
    stepPattern = TYPE5;
  }else{
    stepPattern = ITAKURA;
  }

  if(traj.size() > 0){
    clustering.addTrajectories(traj);

    if(ui_.rbKmedoids->isChecked()){
      clustering.setK(ui_.sbKMethod->value());
      traj = clustering.kMedoidsClustering(df, stepPattern, ui_.cbDtwSlopeWeights->isChecked(), ui_.cbDtwWindowSizeActivate->isChecked(), ui_.sbDtwWindowSize->value(), ui_.cbDtwWindowSizeAdaptable->isChecked());
    }else{
      traj = clustering.simpleClustering(df, ui_.sbKMethod->value());
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

void LipRec::updateClusterTrajectories(){
  clusterTrajectoriesOfCommand.clear();
  QMap<QString, int> commandsWithCount = tdm.getAllCommandsWithCount();
  int clusterK = 0;
  foreach (QString command, commandsWithCount.keys()) {
    QStringList strL = tdm.getFeatures(command);
    foreach (QString feature, strL) {

      QList<QList<double> > clusterT = tdm.getClusterTrajectories(command, feature, ui_.rbKmedoids->text());
      if(clusterT.size() > 0){
        this->setClusterTrajectories(clusterT, command, feature, ui_.rbKmedoids->text());
      }

      if(clusterT.size() > clusterK){
        clusterK = clusterT.size();
      }

      clusterT = tdm.getClusterTrajectories(command, feature, ui_.rbMosaCluster->text());
      if(clusterT.size() > 0){
        this->setClusterTrajectories(clusterT, command, feature, ui_.rbMosaCluster->text());
      }


    }
  }
  ui_.sbKMethod->setValue(clusterK);
}

void LipRec::changeRecordRecognitionStateToDecisionIfPossible()
{
  if(recordRecognitionState == RRRECORDING){
    ui_.pbSaveRecognition->setEnabled(true);
    ui_.pbDeclineRecogntion->setEnabled(true);
    recordRecognitionState = RRDECISION;

    this->setLblMsgRecordRecognition(QString::number(currentUtteranceTrajectories[ui_.cbAspectRatio->text()].size()));

    this->fillRecordRecognitionData();
  }
}

void LipRec::setLblMsgRecordRecognition(QString msg)
{
  QString fontColor = tr("<font color='%1'>%2</font>");
  ui_.lblMsgRecognition->setText( fontColor.arg( "red", msg) );
  ui_.lblMsgRecognition->setFont(QFont("Times New Roman", 10, QFont::Bold));
}

void LipRec::fillRecordRecognitionData()
{
  recordRecognitionData.fileName = ui_.leFilenameRecord->text();
  recordRecognitionData.onlyLowestRecognition = ui_.cbSOLR->isChecked();

  if(ui_.rbDTWSA->isChecked()){
    recordRecognitionData.similarityAlgorithm = "DTW";
  }else{
    recordRecognitionData.similarityAlgorithm = "EUCLIDEAN";
  }

  if(ui_.rbABS->isChecked()){
    recordRecognitionData.distanceFunction =  "ABS";
  }else if(ui_.rbSQUARE->isChecked()){
    recordRecognitionData.distanceFunction =  "SQUARE";
  }else{
    recordRecognitionData.distanceFunction = "SQUARE2";
  }

  recordRecognitionData.weightedDtw = weightedDtwActive;
  recordRecognitionData.dtwStepPattern = ui_.cbDTWStepPattern->currentText();
  recordRecognitionData.dtwWindowActive = ui_.cbDtwWindowSizeActivate->isChecked();
  recordRecognitionData.dtwWindowSize = ui_.sbDtwWindowSize->value();
  recordRecognitionData.dtwWindowAdaptable = ui_.cbDtwWindowSizeAdaptable->isChecked();
  recordRecognitionData.dtwSlopeWeights = ui_.cbDtwSlopeWeights->isChecked();

  if(ui_.rbSingleFF->isChecked()){
    recordRecognitionData.featureFusion = "SINGLE";
  }else if(ui_.rbFusionFF->isChecked()){
    recordRecognitionData.featureFusion = "FUSION";
  }else{
    recordRecognitionData.featureFusion = "BOTH";
  }

  if(ui_.rbKmedoids->isChecked()){
    recordRecognitionData.clusterMethod = "K-MEDOIDS";
  }else{
    recordRecognitionData.clusterMethod = "SIMPLE-CLUSTER";
  }

  recordRecognitionData.clusterK = ui_.sbKMethod->value();

  recordRecognitionData.utteranceLength = currentUtteranceTrajectories[ui_.cbAspectRatio->text()].size();
}

void LipRec::printTrajectory(QList<double> trajectory)
{
  ROS_INFO(">>>Start printTrajectory");
  for (int i = 0; i < trajectory.size(); ++i) {
    ROS_INFO("%f", trajectory.at(i));
  }
  ROS_INFO(">>>End printTrajectory");
}


void LipRec::updateTrajectoriesInfoGUIAndSetWeightsForDTW()
{
  QMap<QString, int> commandsWithCount = tdm.getAllCommandsWithCount();

  QString commands = "";
  QString featuresForCmd = "";

  ui_.lwTrajectoriesInfo->clear();
  ui_.lwDTWWeights->clear();

  int i = 1;
  foreach (QString command, commandsWithCount.keys()) {
    featuresForCmd.clear();

    this->setWeightsForDTW(command);

    QStringList strL = tdm.getFeatures(command);
    foreach (QString feature, strL) {
      featuresForCmd = featuresForCmd + feature + ", ";
    }

    ui_.lwTrajectoriesInfo->addItem(QString::number(i++)+ ") " + command + " (" + QString::number(commandsWithCount[command]) +")" + "\n" +featuresForCmd);

    if(!availableTrajectories.contains(command)){
      availableTrajectories << command;
      ui_.cbTrajectory->clear();
      ui_.cbTrajectory->addItems(availableTrajectories);
    }
  }
}

void LipRec::calculateAndInitWeightsForDTW()
{
  weightedDtw.insert("grasp close", 0.28);
  weightedDtw.insert("grasp open", 0.25);
  weightedDtw.insert("look backward", 0.165);
  weightedDtw.insert("look forward", 0.165);
  weightedDtw.insert("look left", 0.3);
  weightedDtw.insert("look right", 0.28);
  weightedDtw.insert("move backward", 0.16);
  weightedDtw.insert("move fast", 0.28);
  weightedDtw.insert("move forward", 0.165);
  weightedDtw.insert("move slow", 0.28);
  weightedDtw.insert("robo go", 0.18);
  weightedDtw.insert("robo off", 0.18);
  weightedDtw.insert("robo start", 0.17);
  weightedDtw.insert("robo stop", 0.17);
  weightedDtw.insert("turn backward", 0.165);
  weightedDtw.insert("turn left", 0.28);
  weightedDtw.insert("turn right", 0.3);

  weightedDtw.clear();

  int maxTrajectoryLength = 0;
  int minTrajectoryLength = INT_MAX;
  QMap<QString, int> meanLengthForCommand;

  QMap<QString, int> commandsWithCount = tdm.getAllCommandsWithCount();
  foreach (QString command, commandsWithCount.keys()) {
    QList<QList<double> > clusterTrajectories = this->getClusterTrajectories(command, ui_.cbArea->text(), ui_.rbKmedoids->text());

    int currentTrajectoryLengthMean = 0;

    for (int i = 0; i < clusterTrajectories.size(); ++i) {
      currentTrajectoryLengthMean += clusterTrajectories.at(i).size();
    }
    currentTrajectoryLengthMean /= clusterTrajectories.size();

    meanLengthForCommand[command] = currentTrajectoryLengthMean;

    if(currentTrajectoryLengthMean > maxTrajectoryLength){
      maxTrajectoryLength = currentTrajectoryLengthMean;
    }
    if(currentTrajectoryLengthMean < minTrajectoryLength){
      minTrajectoryLength = currentTrajectoryLengthMean;
    }
  }

  int differenceMaxMin = 0;
  double dtwWeightCoefficient = 0.0;

  differenceMaxMin = maxTrajectoryLength - minTrajectoryLength;

  foreach (QString command, meanLengthForCommand.keys()) {
    dtwWeightCoefficient = (((double) maxTrajectoryLength - meanLengthForCommand[command]) * ui_.sbDTWMaxWeightCoefficient->value())/differenceMaxMin;
    weightedDtw.insert(command, dtwWeightCoefficient/100.0);
  }

}

void LipRec::setWeightsForDTW(QString command)
{
  int listWidgetItemHeight = 34;
  QListWidgetItem* item = new QListWidgetItem();

  QDoubleSpinBox* dsb = new QDoubleSpinBox();
  QLabel* lbl = new QLabel(command);

  dsb->setValue(weightedDtw[command]);

  QHBoxLayout *layout = new QHBoxLayout;
  layout->addWidget(dsb);
  layout->addWidget(lbl);

  QWidget* widget = new QWidget;
  widget->setLayout(layout);

  ui_.lwDTWWeights->addItem(item);
  ui_.lwDTWWeights->setItemWidget(item, widget);
  item->setSizeHint(QSize(item->sizeHint().width(), listWidgetItemHeight));
}

void LipRec::lipsActivation(int currentFrame)
{
  Mat imageAbsDiff;
  double d = 0;
  QPixmap pixMap;

  if(frameBuffer.at(last).cols == frameBuffer.at(currentFrame).cols
     && frameBuffer.at(last).rows == frameBuffer.at(currentFrame).rows){

    //temporal segmentation
    d = imageProcessing.generatePixelDifference(frameBuffer[currentFrame], frameBuffer[last]);

    ui_.lcdPixelWiseDiff->display(QString::number(d,'f',0));

    imageAbsDiff = imageProcessing.createImageAbsDiff(frameBuffer[currentFrame], frameBuffer[last]);

    //temporal segmentation
    int activation = QString::number(d,'f',0).toInt();

    if(!imageAbsDiff.empty()){
      this->changeLipActivationState(activation, imageAbsDiff, currentFrame);
      //    pixMap = imageProcessing.getPixmap(imageAbsDiff, true);
      //    pixMap = pixMap.scaled(ui_.lblMouthDiff->maximumWidth(), ui_.lblMouthDiff->maximumHeight(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
      //    ui_.lblMouthDiff->setPixmap(pixMap);
    }

  }
}

Mat LipRec::drawMouthFeaturesOnGUI(Mat &mouthImg, Mat &rLowFinal, Mat &rMidFinal, Mat &rTopFinal, Point upLinePoint, Point bottomLinePoint, Point rightLinePoint, Point keyPoint1, Point keyPoint2, Point keyPoint3, Point keyPoint4, Point keyPoint5, Point keyPoint6)
{
  Mat showMouthImg;

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

  return showMouthImg;
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
