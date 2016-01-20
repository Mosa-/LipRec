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
  BELLMANSTEP,
  DIAGONALSTEP,
  FIVERSTEP,
  ITAKURASTEP
};

struct RecordRecognitionData{
  QString fileName;
  bool onlyLowestRecognition;
  QString similarityAlgorithm;
  QString distanceFunction;

  QString dtwStepPattern;
  bool dtwWindowActive;
  int dtwWindowSize;
  bool dtwWindowAdaptable;

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
};




#endif /* COMMONENUMS_H_ */
