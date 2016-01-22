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

    QTextStream stream(&txtFile);
    QString text;

    text += "Utterance length: " + recordRecognitionData.utteranceLength;
    text += "\n";

    if(recordRecognitionData.featureFusion == "SINGLE"){
      this->composeRecords(text, "_a", recordRecognitionData.commandArea);
      text += "\n";
      this->composeRecords(text, "_ar", recordRecognitionData.commandAspectRatio);
    }else if(recordRecognitionData.featureFusion == "FUSION"){
      this->composeRecords(text, "_f", recordRecognitionData.commandFusion);
    }else{
      this->composeRecords(text, "_a", recordRecognitionData.commandArea);
      text += "\n";
      this->composeRecords(text, "_ar", recordRecognitionData.commandAspectRatio);
      text += "\n";
      this->composeRecords(text, "_f", recordRecognitionData.commandFusion);
    }

    text += "###############################################\n";

    stream << text;

    stream.flush();
    txtFile.close();

  }else{
    ROS_INFO("QFile txtFile failed!");
  }
}

void LipRecRecorder::writeHeaderToTextFile(QString fileName, RecordRecognitionData &recordRecognitionData)
{
  this->fileName = fileName;

  QFile txtFile(fileName+".txt");

  if(txtFile.open(QIODevice::Append | QIODevice::Text)){

    QTextStream stream(&txtFile);
    QString text;

    int borderLength = 20;
    int firstGap = 4;
    int betweenGap = 2;

    for (int i = 0; i < borderLength; ++i) {
      text += "-";
    }
    text += "\n|";
    text += "\n|";
    for (int i = 0; i < firstGap; ++i) {
      text += " ";
    }
    text += QString("%1 %2\n|").arg("$Filenname:", firstGap).arg(recordRecognitionData.fileName, 15);
    text += QString("%1 %2\n|").arg("$Only lowest recognition:", firstGap).arg(recordRecognitionData.onlyLowestRecognition, 15);
    text += QString("%1 %2\n|").arg("$Similarity Algorithm:", firstGap).arg(recordRecognitionData.similarityAlgorithm, 15);
    text += QString("%1 %2\n|").arg("$Distance function:", firstGap).arg(recordRecognitionData.distanceFunction, 15);
    text += QString("%1 %2\n|").arg("$DTW step pattern:", firstGap).arg(recordRecognitionData.dtwStepPattern, 15);
    text += QString("%1 %2\n|").arg("$DTW window active:", firstGap).arg(recordRecognitionData.dtwWindowActive, 15);
    text += QString("%1 %2\n|").arg("$DTW window size:", firstGap).arg(recordRecognitionData.dtwWindowSize, 15);
    text += QString("%1 %2\n|").arg("$DTW window adaptable:", firstGap).arg(recordRecognitionData.dtwWindowAdaptable, 15);
    text += QString("%1 %2\n|").arg("$DTW slope weights:", firstGap).arg(recordRecognitionData.dtwSlopeWeights, 15);
    text += QString("%1 %2\n|").arg("$Feature fusion:", firstGap).arg(recordRecognitionData.featureFusion, 15);
    text += QString("%1 %2\n|").arg("$Cluster method:", firstGap).arg(recordRecognitionData.clusterMethod, 15);
    text += QString("%1 %2\n|").arg("$Cluster K:", firstGap).arg(recordRecognitionData.clusterK, 15);
    for (int i = 0; i < firstGap; ++i) {
      text += " ";
    }
    for (int i = 0; i < borderLength; ++i) {
      text += "_";
    }
    text += " |\n\n";

    stream << text;

    stream.flush();
    txtFile.close();

  }else{
    ROS_INFO("QFile txtFile failed!");
  }
}

void LipRecRecorder::composeRecords(QString &text, QString postFix, QMap<QString, QList<double> >& records)
{
  foreach (QString command, records.keys()) {
    for (int i = 0; i < records[command].size(); ++i) {
      text += command + postFix + ": " + QString::number(records[command].at(i)) + "\n";
    }
  }
}

