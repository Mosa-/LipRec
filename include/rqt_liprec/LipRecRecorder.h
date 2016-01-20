#ifndef LIPRECRECORDER_H_
#define LIPRECRECORDER_H_


#include <QPixmap>

#include <string>
#include <stdlib.h>
#include <ros/ros.h>
#include <ros/package.h>

#include <std_msgs/String.h>
#include <std_srvs/Empty.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "opencv2/video/tracking.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <QFile>
#include <QTextStream>

#include "CommonData.h"

using namespace std_msgs;
using namespace std;
using namespace ros;
using namespace cv;

class LipRecRecorder {
public:
  LipRecRecorder();
  virtual ~LipRecRecorder();

  void setFileName(QString fileName);

  void writeToTextFile(QString fileName, RecordRecognitionData &recordRecognitionData);
private:
  QString fileName;


};

#endif /* LIPRECRECORDER_H_ */
