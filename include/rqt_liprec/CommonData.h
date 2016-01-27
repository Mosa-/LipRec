#ifndef COMMONEDATA_H_
#define COMMONEDATA_H_

#include <QMap>
#include <QList>
#include <QString>

enum HistogramType{
  H_NONE,
  GHE,
  AHE
};

enum BlurType{
  B_NONE,
  BLUR,
  MEDIAN,
  GAUSSIAN
};

enum SignalSmoothingType{
  S_NONE,
  AVERAGE
};

enum RGB{
  B = 0,
  G = 1,
  R = 2
};

enum DistanceFunction{
  SQUARE,
  SQUARE2,
  ABS
};

enum DtwStepPattern{
  TYPE1,
  TYPE2,
  TYPE3,
  TYPE4,
  TYPE5,
  ITAKURA
};

struct CommandWithCost{
  bool operator<(const CommandWithCost& other) const {
      return cost < other.cost;
  }
  QString command;
  double cost;
};

struct RecordRecognitionData{
  QString fileName;
  bool onlyLowestRecognition;
  QString similarityAlgorithm;
  QString distanceFunction;

  bool weightedDtw;
  QString dtwStepPattern;
  bool dtwWindowActive;
  int dtwWindowSize;
  bool dtwWindowAdaptable;
  bool dtwSlopeWeights;

  QString featureFusion;

  QString clusterMethod;
  int clusterK;

  int utteranceLength;

  QMap<QString, QList<double> > commandFusion;
  QMap<QString, QList<double> > commandArea;
  QMap<QString, QList<double> > commandAspectRatio;

  void clear(){
    commandFusion.clear();
    commandArea.clear();
    commandAspectRatio.clear();
  }

  void sortCommands(){

    foreach (QString command, commandFusion.keys()) {
      qSort(commandFusion[command].begin(), commandFusion[command].end());
    }
    foreach (QString command, commandArea.keys()) {
      qSort(commandArea[command].begin(), commandArea[command].end());
    }
    foreach (QString command, commandAspectRatio.keys()) {
      qSort(commandAspectRatio[command].begin(), commandAspectRatio[command].end());
    }

  }
};

#endif /* COMMONENUMS_H_ */
