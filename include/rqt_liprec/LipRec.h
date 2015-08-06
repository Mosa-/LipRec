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

#include <string>
#include <stdlib.h>
#include <ros/ros.h>
#include <ros/package.h>

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

using namespace std_msgs;
using namespace std;
using namespace ros;
using namespace cv;

namespace rqt_liprec {

typedef enum{
	Idle,
	StartFrame,
	Utterance,
	EndFrame
}DetectStartEndFrame;

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
  void imageCallback2(const sensor_msgs::ImageConstPtr& msg);
  void faceROICallback(const sensor_msgs::RegionOfInterestConstPtr& msg);
  void mouthROICallback(const sensor_msgs::RegionOfInterestConstPtr& msg);


private:
  Ui_Form ui_;
  QWidget* widget_;

  ros::Subscriber camImage;
  ros::Subscriber camImage2;
  ros::Subscriber faceROISub;
  ros::Subscriber mouthROISub;

  sensor_msgs::RegionOfInterest faceROI;
  sensor_msgs::RegionOfInterest mouthROI;

  QTimer faceROITimer;
  QTimer mouthROITimer;
  bool faceROI_detected;
  bool mouthROI_detected;

  int blackBorder;

  double MHI_DURATION;
  int NO_CYCLIC_FRAME;
  QList<Mat> frameBuffer;
  int last;
  Mat mhi;

  DetectStartEndFrame stateDetectionStartEndFrame;
  QList<Mat> utterance;
  int silenceCounter;


  int timeoutROIdetection;

  void drawRectangle(Mat& iplImg, sensor_msgs::RegionOfInterest& roi);
  Mat cutROIfromImage(Mat& src, sensor_msgs::RegionOfInterest& roi);
  QPixmap getPixmap(Mat& iplImg);
  void drawFaceMouthROI(Mat& img);
  void showLips(Mat& mouthImg);
  int updateFrameBuffer(Mat img);
  void createMotionHistoryImage(Mat& img);
  Mat createImageAbsDiff(int currentFrame);
  void printMat(Mat& data);

  void setupModel();

public slots:
	void getCamPic(cv::Mat img);

signals:
	void updateCam(cv::Mat img);

private slots:
	void faceROItimeout();
	void mouthROItimeout();



};
} // namespace
#endif
