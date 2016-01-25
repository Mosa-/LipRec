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
  recordRecognitionData.sortCommands();

  this->fileName = fileName;

  QFile txtFile(fileName+".txt");

  if(txtFile.open(QIODevice::Append | QIODevice::Text)){

    QTextStream stream(&txtFile);
    QString text;

    text += QString("Utterance length: %1\n\n").arg(recordRecognitionData.utteranceLength);

    QList<CommandWithCost> tmpCommandFusion;
    QList<CommandWithCost> tmpCommandArea;
    QList<CommandWithCost> tmpCommandAspectRatio;

    if(recordRecognitionData.onlyLowestRecognition){
      CommandWithCost commandWithCost;

      foreach (QString command, recordRecognitionData.commandFusion.keys()) {
        commandWithCost.command = command;
        commandWithCost.cost = recordRecognitionData.commandFusion[command].at(0);

        tmpCommandFusion.append(commandWithCost);
      }
      foreach (QString command, recordRecognitionData.commandArea.keys()) {
        commandWithCost.command = command;
        commandWithCost.cost = recordRecognitionData.commandArea[command].at(0);

        tmpCommandArea.append(commandWithCost);
      }
      foreach (QString command, recordRecognitionData.commandAspectRatio.keys()) {
        commandWithCost.command = command;
        commandWithCost.cost = recordRecognitionData.commandAspectRatio[command].at(0);

        tmpCommandAspectRatio.append(commandWithCost);
      }

      qSort(tmpCommandFusion);
      qSort(tmpCommandArea);
      qSort(tmpCommandAspectRatio);
    }

    if(recordRecognitionData.featureFusion == "SINGLE"){
      if(recordRecognitionData.onlyLowestRecognition){
        this->composeRecords(text, "_a", tmpCommandArea);
      }else{
        this->composeRecords(text, "_a", recordRecognitionData.commandArea);
      }
      text += "\n";
      if(recordRecognitionData.onlyLowestRecognition){
        this->composeRecords(text, "_ar", tmpCommandAspectRatio);
      }else{
        this->composeRecords(text, "_ar", recordRecognitionData.commandAspectRatio);
      }
    }else if(recordRecognitionData.featureFusion == "FUSION"){
      if(recordRecognitionData.onlyLowestRecognition){
        this->composeRecords(text, "_f", tmpCommandFusion);
      }else{
        this->composeRecords(text, "_f", recordRecognitionData.commandFusion);
      }
    }else{
      if(recordRecognitionData.onlyLowestRecognition){
        this->composeRecords(text, "_a", tmpCommandArea);
      }else{
        this->composeRecords(text, "_a", recordRecognitionData.commandArea);
      }
      text += "\n";
      if(recordRecognitionData.onlyLowestRecognition){
        this->composeRecords(text, "_ar", tmpCommandAspectRatio);
      }else{
        this->composeRecords(text, "_ar", recordRecognitionData.commandAspectRatio);
      }
      text += "\n";
      if(recordRecognitionData.onlyLowestRecognition){
        this->composeRecords(text, "_f", tmpCommandFusion);
      }else{
        this->composeRecords(text, "_f", recordRecognitionData.commandFusion);
      }
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
  recordRecognitionData.sortCommands();

  this->fileName = fileName;

  QFile txtFile(fileName+".txt");

  if(txtFile.open(QIODevice::Append | QIODevice::Text)){

    QTextStream stream(&txtFile);
    QString text;

    int firstGap = -30;
    int secondGap = -15;
    int gap = 5;
    int borderLength = 1+abs(firstGap) + abs(secondGap) + abs(gap);


    text += "\n|";
    for (int i = 0; i < borderLength; ++i) {
      text += "-";
    }
    text += "|\n|";
    for (int i = 0; i < gap; ++i) {
      text += " ";
    }
    text += QString("%1 %2|\n|").arg("$Filenname:", firstGap).arg(recordRecognitionData.fileName, secondGap);
    for (int i = 0; i < gap; ++i) {
      text += " ";
    }
    text += QString("%1 %2|\n|").arg("$Only lowest recognition:", firstGap).arg(recordRecognitionData.onlyLowestRecognition, secondGap);
    for (int i = 0; i < gap; ++i) {
      text += " ";
    }
    text += QString("%1 %2|\n|").arg("$Similarity Algorithm:", firstGap).arg(recordRecognitionData.similarityAlgorithm, secondGap);
    for (int i = 0; i < gap; ++i) {
      text += " ";
    }
    text += QString("%1 %2|\n|").arg("$Distance function:", firstGap).arg(recordRecognitionData.distanceFunction, secondGap);
    for (int i = 0; i < gap; ++i) {
      text += " ";
    }
    text += QString("%1 %2|\n|").arg("$DTW step pattern:", firstGap).arg(recordRecognitionData.dtwStepPattern, secondGap);
    for (int i = 0; i < gap; ++i) {
      text += " ";
    }
    text += QString("%1 %2|\n|").arg("$DTW window active:", firstGap).arg(recordRecognitionData.dtwWindowActive, secondGap);
    for (int i = 0; i < gap; ++i) {
      text += " ";
    }
    text += QString("%1 %2|\n|").arg("$DTW window size:", firstGap).arg(recordRecognitionData.dtwWindowSize, secondGap);
    for (int i = 0; i < gap; ++i) {
      text += " ";
    }
    text += QString("%1 %2|\n|").arg("$DTW window adaptable:", firstGap).arg(recordRecognitionData.dtwWindowAdaptable, secondGap);
    for (int i = 0; i < gap; ++i) {
      text += " ";
    }
    text += QString("%1 %2|\n|").arg("$DTW slope weights:", firstGap).arg(recordRecognitionData.dtwSlopeWeights, secondGap);
    for (int i = 0; i < gap; ++i) {
      text += " ";
    }
    text += QString("%1 %2|\n|").arg("$Feature fusion:", firstGap).arg(recordRecognitionData.featureFusion, secondGap);
    for (int i = 0; i < gap; ++i) {
      text += " ";
    }
    text += QString("%1 %2|\n|").arg("$Cluster method:", firstGap).arg(recordRecognitionData.clusterMethod, secondGap);
    for (int i = 0; i < gap; ++i) {
      text += " ";
    }
    text += QString("%1 %2|\n|").arg("$Cluster K:", firstGap).arg(recordRecognitionData.clusterK, secondGap);

    for (int i = 0; i < borderLength-1; ++i) {
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

void LipRecRecorder::composeRecords(QString &text, QString postFix, QList<CommandWithCost >& records)
{
  for (int i = 0; i < records.size(); ++i) {
    text += records.at(i).command + postFix + ": " + QString::number(records.at(i).cost) + "\n";
  }
}

