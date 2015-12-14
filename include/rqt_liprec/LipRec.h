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

#include "KeyPointsDeliverer.h"
#include "ImageProcessing.h"
#include "Dtw.h"
#include "TrajectoriesDataManager.h"
#include "Clustering.h"

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

class LipRec
        : public rqt_gui_cpp::Plugin
{
    Q_OBJECT
public:
    LipRec();
    virtual void initPlugin(qt_gui_cpp::PluginContext& context);
    virtual void shutdownPlugin();
    virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
    virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void faceROICallback(const sensor_msgs::RegionOfInterestConstPtr& msg);
    void mouthROICallback(const sensor_msgs::RegionOfInterestConstPtr& msg);

private:
    Ui_Form ui_;
    QWidget* widget_;

    ros::Subscriber camImage;
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

    qint64 lcdUpdateTimeStamp;

    RecordTrajectoryState recordTrajectoryState;
    QMap<QString, QList<double> > recordTrajectory;

    QStringList availableTrajectories;

    QMap<QString, QList<QList<double> > > clusterTrajectoriesOfCommand;
    QMap<QString, QList<double> > currentUtteranceTrajectories;
    bool utter;

    KeyPointsDeliverer keyPointsDeliverer;
    ImageProcessing imageProcessing;
    Dtw dtw;
    TrajectoriesDataManager tdm;
    Clustering clustering;

    void processImage(Mat img);

    void drawFaceMouthROI(Mat& img);

    void showLips(Mat& mouthImg, bool useMonoImage = false);

    int updateFrameBuffer(Mat img);

    void changeLipActivationState(int activation, Mat& imageAbsDiff, int currentFrame);

    void recordUtteranceFrame(Mat currentFrame);

    QList<QList<double> > getClusterTrajectories(QString command, QString feature, QString clusterMethod);
    void setClusterTrajectories(QList<QList<double> > clusterT, QString command, QString feature, QString clusterMethod);

    void changeUseCam();

    void applySignalSmoothing(int graphicView, SignalSmoothingType type);
    void averageSignalSmoothing(QList<int>& signalsSmoothing);

    void drawMouthFeatures(Mat& mouth, Point keyPoint1, Point keyPoint2, Point keyPoint3, Point keyPoint4, Point keyPoint5, Point keyPoint6);

    void printMat(Mat& data);

    void setupModel();

    void applyCluster(QString clusterMethod, DistanceFunction df, QString command, QString feature);
public slots:
    void getCamPic(cv::Mat img);

    void triggedAction(QAction* action);

signals:
    void updateCam(cv::Mat img);

private slots:
    void faceROItimeout();
    void mouthROItimeout();
    void clickedUtteranceDiffPlot();
    void clickedContinueVideo();
    void toggleKpLines();
    void toggleSupportLines(bool checked);
    void clickedPrintFeatures();

    void clickedRecordStopTrajectory();
    void clickedSaveTrajectory();
    void clickedAbortTrajectory();
    void clickedAbortOrSaveTrajectory();

    void clickedCluster();

};
} // namespace
#endif
