#ifndef rqt_liprec__my_plugin_H
#define rqt_liprec__my_plugin_H

#include <rqt_gui_cpp/plugin.h>
#include <ui_liprec.h>
#include <QWidget>
#include <QStringList>
#include <QList>
#include <QIcon>
#include <QMessageBox>
#include <QImage>
#include <QPainter>
#include <QTimer>
#include <QDebug>
#include <QToolBar>
#include <QMenuBar>
#include <QDir>
#include <QWidgetAction>
#include <QLineEdit>
#include <QCheckBox>

#include <string>
#include <stdlib.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>

#include <std_msgs/String.h>
#include <std_srvs/Empty.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <cv.h>
#include <sensor_msgs/image_encodings.h>
#include "opencv2/video/tracking.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <time.h>
#include <QtAlgorithms>
#include <QDateTime>

#include <QKeyEvent>

#include "KeyPointsDeliverer.h"
#include "ImageProcessing.h"
#include "Dtw.h"
#include "TrajectoriesDataManager.h"
#include "Clustering.h"
#include "LipRecRecorder.h"

#include "CommonData.h"

using namespace std_msgs;
using namespace std;
using namespace ros;
using namespace cv;

namespace rqt_liprec {

typedef enum{
    Idle,
    Utterance,
}DetectStartEndFrame;

enum RecordTrajectoryState{
    None,
    Recording,
    Save,
    Abort
};

enum RecordRecognitionState{
  RRNONE,
  RRRECORDING,
  RRDECISION,
  RRSAVE,
  RRDECLINE
};

class LipRec : public rqt_gui_cpp::Plugin
{
    Q_OBJECT
public:
    LipRec();
    virtual void initPlugin(qt_gui_cpp::PluginContext& context);
    virtual void shutdownPlugin();
    virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
    virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void imageDepthCallback(const sensor_msgs::ImageConstPtr& msg);
    void faceROICallback(const sensor_msgs::RegionOfInterestConstPtr& msg);
    void mouthROICallback(const sensor_msgs::RegionOfInterestConstPtr& msg);

private:
    Ui_Form ui_;
    QWidget* widget_;

    ros::Subscriber camImage;
    ros::Subscriber camImageDepth;
    ros::Subscriber faceROISub;
    ros::Subscriber mouthROISub;

    sensor_msgs::RegionOfInterest faceROI;
    sensor_msgs::RegionOfInterest mouthROI;

    QTimer faceROITimer;
    QTimer mouthROITimer;
    bool faceROI_detected;
    bool mouthROI_detected;

    int blackBorder;
    bool useMonoImage;

    QMutex depthCamMtx;
    Mat depthCam;

    int NO_CYCLIC_FRAME;
    QList<Mat> frameBuffer;
    int last;
    Mat mhi;

    DetectStartEndFrame stateDetectionStartEndFrame;
    QList<Mat> utterance;
    int silenceCounter;

    int timeoutROIdetection;

    QList<int> utterancePixelDiff;

    QLineEdit* le;
    QCheckBox *checkboxOnlyMouth;
    Mat currentUtteranceFrame;
    bool initVideoWriter;
    bool recordVideo;
    bool recordUtterance;
    bool loadUtterance;
    bool useCam;

    int drawKeyPointState;

    bool drawSupportLines;

    bool printFeatures;

    bool updateRecognizedText;

    bool autoRecord;
    int autoRecordFileNameCounter;

    QMutex clusterMtx;
    QMutex utteranceMtx;

    RecordRecognitionState recordRecognitionState;
    RecordRecognitionData recordRecognitionData;
    QString currentRecordRecognitionFilename;

    qint64 lcdUpdateTimeStamp;

    RecordTrajectoryState recordTrajectoryState;
    QMap<QString, QList<double> > recordTrajectory;

    QStringList availableTrajectories;

    QMap<QString, QList<QList<double> > > clusterTrajectoriesOfCommand;
    QMap<QString, QList<double> > currentUtteranceTrajectories;
    bool utter;

    bool applyUtteranceOfLoadedFile;

    QMap<QString, double> weightedDtw;
    bool weightedDtwActive;

    KeyPointsDeliverer keyPointsDeliverer;
    ImageProcessing imageProcessing;
    Dtw dtw;
    TrajectoriesDataManager tdm;
    Clustering clustering;
    LipRecRecorder lipRecRecorder;

    void processImage(Mat img);

    void calculateDTWCostSingle(QList<CommandWithCost> &areaCommandsWithCost, QList<CommandWithCost> &aspectRatioCommandsWithCost, DtwStepPattern stepPattern,
                                DistanceFunction df, int windowSize, CommandWithCost &commandWithCost, int &indexOfLowAreaCluster, int &indexOfLowAspectRatioCluster,
                                QString &currentCommandArea, QString &currentCommandAspectRatio);
    void calculateDTWCostFusion(QList<CommandWithCost>& fusionCommandsWithCost, int &fusionAreaIndex, int &fusionAspectRatioIndex,
                                DtwStepPattern stepPattern, DistanceFunction df, int windowSize, int& indexOfLowAreaCluster, int& indexOfLowAspectRatioCluster,
                                CommandWithCost &commandWithCost, QString &currentCommandFusion);
    void calculateEuclideanCostSingle(CommandWithCost &commandWithCost, QList<CommandWithCost> &areaCommandsWithCost, QList<CommandWithCost> &aspectRatioCommandsWithCost);
    void calculateEuclideanCostFusion(CommandWithCost &commandWithCost, QList<CommandWithCost> &fusionCommandsWithCost, int &indexOfLowAreaCluster, int &indexOfLowAspectRatioCluster,
                                      int& fusionAreaIndex, int& fusionAspectRatioIndex, QString& currentCommandFusion);
    void showDTWwithPathOnGUI(QPixmap &dtwPixMapArea, QPixmap &dtwPixMapAspectRatio, QString currentCommandArea, int indexOfLowAreaCluster,
                               QString currentCommandAspectRatio, int indexOfLowAspectRatioCluster,
                               QString currentCommandFusion, int fusionAreaIndex, int fusionAspectRatioIndex,
                               DistanceFunction df, DtwStepPattern stepPattern);

    void drawFaceMouthROI(Mat& img);

    void showLips(Mat& mouthImg, bool useMonoImage = false);

    int updateFrameBuffer(Mat img);

    void changeLipActivationState(int activation, Mat& imageAbsDiff, int currentFrame);

    void recordUtteranceFrame(Mat currentFrame);

    QList<QList<double> > getClusterTrajectories(QString command, QString feature, QString clusterMethod);
    void setClusterTrajectories(QList<QList<double> > clusterT, QString command, QString feature, QString clusterMethod);

    double calculateEuclideanDistance(QList<double>& trj1, QList<double>& trj2);

    void changeUseCam();

    void averageSignalSmoothing(QList<int>& signalsSmoothing);

    void drawMouthFeatures(Mat& mouth, Point keyPoint1, Point keyPoint2, Point keyPoint3, Point keyPoint4, Point keyPoint5, Point keyPoint6);

    void printMat(Mat& data);

    void setupModel();

    void applyCluster(QString clusterMethod, DistanceFunction df, QString command, QString feature);

    void printTrajectory(QList<double> trajectory);

    void calculateAndInitWeightsForDTW();
    void setWeightsForDTW(QString command);

    void updateTrajectoriesInfoGUIAndSetWeightsForDTW();
    QPixmap drawDTWPixmap(QString currentCommand, QString feature, int indexOfLowCluster, QString clusterMethod, DistanceFunction df, DtwStepPattern stepPattern);
    void lipsActivation(int currentFrame);
    Mat drawMouthFeaturesOnGUI(Mat& mouthImg, Mat& rLowFinal, Mat& rMidFinal, Mat& rTopFinal,
                          Point upLinePoint, Point bottomLinePoint, Point rightLinePoint,
                               Point keyPoint1, Point keyPoint2, Point keyPoint3, Point keyPoint4, Point keyPoint5, Point keyPoint6);
    void updateClusterTrajectories();

    void changeRecordRecognitionStateToDecisionIfPossible();
    void setLblMsgRecordRecognition(QString msg);
    void fillRecordRecognitionData();

public slots:
    void getCamPic(Mat img);
    void getDepthCamPic(Mat img);

    void triggedAction(QAction* action);
    void lineEditChanged(const QString& str);
    void spinBoxChanged(int value);

signals:
    void updateCam(Mat img);
    void updateDepthCam(Mat img);

private slots:
    void faceROItimeout();
    void mouthROItimeout();
    void clickedContinueVideo();
    void clickedRecorderOptions();
    void toggleKpLines();
    void toggleSupportLines(bool checked);
    void clickedPrintFeatures();

    void clickedRecordStopTrajectory();
    void clickedSaveTrajectory();
    void clickedAbortTrajectory();
    void clickedAbortOrSaveTrajectory();

    void clickedCluster();

    void clickedUpdateRecognizedText(bool checked);
    void clickedRecordRecognized(bool checked);

    void clickedDeclineRecordRecognition();
    void clickedSaveRecordRecognition();
    void clickedDeclineOrSaveRecordRecognition();

    void clickedUtter();

    void clickedWeightedDtw(bool checked);

    void clickedAutoRecord();
};
} // namespace
#endif
