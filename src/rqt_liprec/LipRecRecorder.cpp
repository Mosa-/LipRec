#include "rqt_liprec/LipRecRecorder.h"

LipRecRecorder::LipRecRecorder() {
	// TODO Auto-generated constructor stub

}

LipRecRecorder::~LipRecRecorder() {
  // TODO Auto-generated destructor stub
}

void LipRecRecorder::setFileName(QString fileName)
{
  this->fileName = fileName;
}

void LipRecRecorder::writeToTextFile(QString fileName, RecordRecognitionData& recordRecognitionData){
  this->fileName = fileName;

  QFile txtFile(fileName+".txt");

  if(txtFile.open(QIODevice::Append | QIODevice::Text)){

    ROS_INFO("write!");

    QTextStream stream(&txtFile);

    stream << "test 1223\n";

    stream.flush();
    txtFile.close();

  }else{
    ROS_INFO("QFile txtFile failed!");
  }

}

