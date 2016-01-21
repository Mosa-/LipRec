#include "rqt_liprec/Dtw.h"


Dtw::Dtw()
{

}

Dtw::Dtw(QList<double> trajectory1, QList<double> trajectory2, DtwStepPattern stepPattern, bool slopeWeights){

  this->trajectory1 = trajectory1;
  this->trajectory2 = trajectory2;
  this->stepPattern = stepPattern;
  this->slopeWeights = slopeWeights;
}

Dtw::~Dtw(){

}

void Dtw::seed(QList<double> trajectory1, QList<double> trajectory2, DtwStepPattern stepPattern, bool slopeWeights)
{
  this->trajectory1 = trajectory1;
  this->trajectory2 = trajectory2;
  this->stepPattern = stepPattern;
  this->slopeWeights = slopeWeights;

  double min1 = *std::min_element(this->trajectory1.begin(), this->trajectory1.end());
  double min2 = *std::min_element(this->trajectory2.begin(), this->trajectory2.end());
  double max1 = *std::max_element(this->trajectory1.begin(), this->trajectory1.end());
  double max2 = *std::max_element(this->trajectory2.begin(), this->trajectory2.end());

  double minFinal = qMin(min1, min2);
  double maxFinal = qMax(max1, max2);

  double newVal = 0.0;
  for (int i = 0; i < this->trajectory1.size(); ++i) {
    newVal = (this->trajectory1[i] - minFinal) / (maxFinal - minFinal);
    //ROS_INFO("val: %f, newVal: %f, min: %f, max: %f", this->trajectory1[i], newVal, minFinal, maxFinal);
    this->trajectory1[i] = newVal;
  }

  for (int i = 0; i < this->trajectory2.size(); ++i) {
    newVal = (this->trajectory2[i] - minFinal) / (maxFinal - minFinal);
    this->trajectory2[i] = newVal;
  }
}

Mat Dtw::calculateDistanceMatrix(DistanceFunction distanceFunction){
  distanceMatrix = Mat(trajectory1.size(), trajectory2.size(), CV_64F, Scalar(0.0));

  double distance = 0.0;
  for (int i = 0; i < distanceMatrix.cols; ++i) {
    for (int j = 0; j < distanceMatrix.rows; ++j) {
      if(distanceFunction == SQUARE){
        distance = squareDistance(trajectory1.at(j), trajectory2.at(i));
      }else if(distanceFunction == SQUARE2){
        distance = square2Distance(trajectory1.at(j), trajectory2.at(i));
      }else{
        distance = absDistance(trajectory1.at(j), trajectory2.at(i));
      }
      distanceMatrix.at<double>(j,i) = distance;
    }
  }

  return distanceMatrix;
}

Mat Dtw::calculateDtwDistanceMatrix(bool activeWindowSize, int windowSize, bool windowAdapted){
  dtwDistanceMatrix = distanceMatrix;

  vector<Mat> matrices;

  Mat infRow(dtwDistanceMatrix.rows, 1, CV_64F, Scalar(INT_MAX));
  Mat infCol(1, dtwDistanceMatrix.cols+1, CV_64F, Scalar(INT_MAX));

  matrices.push_back(infRow);
  matrices.push_back(dtwDistanceMatrix);

  hconcat(matrices, dtwDistanceMatrix);

  matrices.clear();
  matrices.push_back(infCol);
  matrices.push_back(dtwDistanceMatrix);

  vconcat(matrices, dtwDistanceMatrix);


  if(activeWindowSize){
    for (int i = 0; i < dtwDistanceMatrix.cols; ++i) {
      for (int j = 0; j < dtwDistanceMatrix.rows; ++j) {
        dtwDistanceMatrix.at<double>(j,i) = INT_MAX;
      }
    }
    dtwDistanceMatrix.at<double>(1,1) = distanceMatrix.at<double>(0,0);
  }
  dtwDistanceMatrix.at<double>(0,0) = 0.0;


  if(stepPattern == TYPE1){
    applyStepPatternType1(activeWindowSize, windowSize, windowAdapted);
  }else if(stepPattern == TYPE2){
    applyStepPatternType2(activeWindowSize, windowSize, windowAdapted);
  }else if(stepPattern == TYPE3){
    applyStepPatternType3(activeWindowSize, windowSize, windowAdapted);
  }else if(stepPattern == TYPE4){
    applyStepPatternType4(activeWindowSize, windowSize, windowAdapted);
  }else if(stepPattern == TYPE5){
    applyStepPatternType5(activeWindowSize, windowSize, windowAdapted);
  }else{
    applyStepPatternItakura(activeWindowSize, windowSize, windowAdapted);
  }

  return dtwDistanceMatrix;
}


QList<Point> Dtw::calculateGreedyWarpingPath()
{
  QList<Point> warpingPath;
  int i = dtwDistanceMatrix.cols-1;
  int j = dtwDistanceMatrix.rows-1;

  double insertion = 0.0;
  double deletion = 0.0;
  double match = 0.0;
  double insertion2 = 0.0;
  double deletion2 = 0.0;
  double match2 = 0.0;


  double insertionDistance = 0.0;
  double deletionDistance = 0.0;
  double matchDistance = 0.0;
  double match2Distance = 0.0;

  if(stepPattern == TYPE1){

    while(i>1 && j>1){
      if(j==1){
        i--;
      }else if(i==1){
        j--;
      }else{
        insertion = dtwDistanceMatrix.at<double>(j, i-1);
        deletion = dtwDistanceMatrix.at<double>(j-1, i);
        match = dtwDistanceMatrix.at<double>(j-1, i-1);

        if(slopeWeights){
          insertion += distanceMatrix.at<double>(j-1,i-1);
          deletion +=  distanceMatrix.at<double>(j-1,i-1);
          match += 2*distanceMatrix.at<double>(j-1,i-1);
        }

        if(deletion == minimum(insertion, deletion, match)){
          j--;
        }else if(insertion == minimum(insertion, deletion, match)){
          i--;
        }else{
          i--;
          j--;
        }
        warpingPath.append(Point(j,i));
      }
    }

  }else if(stepPattern == TYPE2){

    while(i>1 && j>1){
      if(j==1){
        i--;
      }else if(i==1){
        j--;
      }else{
        insertion = dtwDistanceMatrix.at<double>(j-1, i-2);
        deletion = dtwDistanceMatrix.at<double>(j-1, i-1);
        match = dtwDistanceMatrix.at<double>(j-2, i-1);

        insertionDistance = distanceMatrix.at<double>(j-1,i-2) + distanceMatrix.at<double>(j-1,i-1);
        deletionDistance = distanceMatrix.at<double>(j-1,i-1);
        matchDistance = distanceMatrix.at<double>(j-2,i-1) + distanceMatrix.at<double>(j-1,i-1);

        if(slopeWeights){
          insertion += 0.5*(insertionDistance);
          deletion +=  deletionDistance;
          match += 0.5*(matchDistance);
        }else{
          insertion += insertionDistance;
          deletion +=  deletionDistance;
          match += matchDistance;
        }

        if(deletion == minimum(insertion, deletion, match)){
          i--;
          j--;
        }else if(insertion == minimum(insertion, deletion, match)){
          j--;
          i -=2;
        }else{
          j-=2;
          i--;
        }
        warpingPath.append(Point(j,i));
      }
    }

  }else if(stepPattern == TYPE3){

    while(i>1 && j>1){
      if(j==1){
        i--;
      }else if(i==1){
        j--;
      }else{
        insertion = dtwDistanceMatrix.at<double>(j-1, i-1);
        deletion = dtwDistanceMatrix.at<double>(j-2, i-1);
        match = dtwDistanceMatrix.at<double>(j-1, i-2);

        if(slopeWeights){
          insertion += 2*distanceMatrix.at<double>(j-1,i-1);
          deletion +=  3*distanceMatrix.at<double>(j-1,i-1);
          match += 3*distanceMatrix.at<double>(j-1,i-1);
        }

        if(deletion == minimum(insertion, deletion, match)){
          j-=2;
          i--;
        }else if(insertion == minimum(insertion, deletion, match)){
          j--;
          i --;
        }else{
          j--;
          i=-2;
        }
        warpingPath.append(Point(j,i));
      }
    }

  }else if(stepPattern == TYPE4){

    while(i>1 && j>1){
      if(j==1){
        i--;
      }else if(i==1){
        j--;
      }else{
        insertion = dtwDistanceMatrix.at<double>(j-1, i-2);
        deletion = dtwDistanceMatrix.at<double>(j-2, i-2);
        match = dtwDistanceMatrix.at<double>(j-1, i-1);
        match2 = dtwDistanceMatrix.at<double>(j-2, i-1);

        insertionDistance = distanceMatrix.at<double>(j-1,i-2) + distanceMatrix.at<double>(j-1,i-1);
        deletionDistance = distanceMatrix.at<double>(j-1,i-2) + distanceMatrix.at<double>(j-1,i-1);
        matchDistance =  distanceMatrix.at<double>(j-1,i-1);
        match2Distance = distanceMatrix.at<double>(j-1,i-1);

        insertion += insertionDistance;
        deletion += deletionDistance;
        match += matchDistance;
        match2 += match2Distance;

        double tmpMin = minimum(insertion, deletion, match);

        if(deletion == minimum(tmpMin, deletion, match2)){
          j-=2;
          i-=2;
        }else if(insertion == minimum(tmpMin, deletion, match2)){
          j--;
          i-=2;
        }else if(match == minimum(tmpMin, deletion, match2)){
          j--;
          i--;
        }else{
          j-=2;
          i--;
        }
        warpingPath.append(Point(j,i));
      }
    }

  }else if(stepPattern == TYPE5){

    while(i>1 && j>1){
      if(j==1){
        i--;
      }else if(i==1){
        j--;
      }else{
        insertion = dtwDistanceMatrix.at<double>(j-1, i-3);
        deletion = dtwDistanceMatrix.at<double>(j-1, i-2);
        match = dtwDistanceMatrix.at<double>(j-1, i-1);
        insertion2 = dtwDistanceMatrix.at<double>(j-2, i-1);
        deletion2 = dtwDistanceMatrix.at<double>(j-3, i-1);

        if(slopeWeights){
          insertion += 2*distanceMatrix.at<double>(j-1,i-3)+distanceMatrix.at<double>(j-1,i-2)+distanceMatrix.at<double>(j-1,i-1);
          deletion +=  2*distanceMatrix.at<double>(j-1,i-2)+distanceMatrix.at<double>(j-1,i-1);
          match += 2*distanceMatrix.at<double>(j-1,i-1);
          insertion2 += 2*distanceMatrix.at<double>(j-2,i-1)+distanceMatrix.at<double>(j-1,i-1);
          deletion2 += 2*distanceMatrix.at<double>(j-3,i-1)+distanceMatrix.at<double>(j-2,i-1)+distanceMatrix.at<double>(j-1,i-1);
        }else{
          insertion += distanceMatrix.at<double>(j-1,i-3)+distanceMatrix.at<double>(j-1,i-2)+distanceMatrix.at<double>(j-1,i-1);
          deletion +=  distanceMatrix.at<double>(j-1,i-2)+distanceMatrix.at<double>(j-1,i-1);
          match += distanceMatrix.at<double>(j-1,i-1);
          insertion2 += distanceMatrix.at<double>(j-2,i-1)+distanceMatrix.at<double>(j-1,i-1);
          deletion2 += distanceMatrix.at<double>(j-3,i-1)+distanceMatrix.at<double>(j-2,i-1)+distanceMatrix.at<double>(j-1,i-1);
        }

        double tmpMin = minimum(insertion, deletion, match);

        if(deletion == minimum(tmpMin, insertion2, deletion2)){
          j--;
          i-=2;
        }else if(insertion == minimum(tmpMin, insertion2, deletion2)){
          j--;
          i-=3;
        }else if(match == minimum(tmpMin, insertion2, deletion2)){
          j--;
          i--;
        }else if(insertion2 == minimum(tmpMin, insertion2, deletion2)){
          j-=2;
          i--;
        }else{
          j-=3;
          i--;
        }
        warpingPath.append(Point(j,i));
      }
    }

  }else{

    while(i>1 && j>1){
      if(j==1){
        i--;
      }else if(i==1){
        j--;
      }else{
        insertion = dtwDistanceMatrix.at<double>(j, i-1);
        deletion = dtwDistanceMatrix.at<double>(j-1, i-1);
        match = dtwDistanceMatrix.at<double>(j-2, i-2);

        insertion += distanceMatrix.at<double>(j-1,i-1);;
        deletion += distanceMatrix.at<double>(j-1,i-1);;
        match += distanceMatrix.at<double>(j-1,i-1);;

        if(deletion == minimum(insertion, deletion, match)){
          j--;
          i--;
        }else if(insertion == minimum(insertion, deletion, match)){
          i--;
        }else{
          i-=2;
          j-=2;
        }
        warpingPath.append(Point(j,i));
      }
    }
  }

  this->warpingPath = warpingPath;

  return warpingPath;
}

void Dtw::applyStepPatternType1(bool activeWindowSize, int windowSize, bool windowAdapted)
{
  double insertion = 0.0;
  double deletion = 0.0;
  double match = 0.0;

  double dtwValue = 0.0;

  int currentWindowSize = windowSize;
  if(activeWindowSize && windowAdapted){
    currentWindowSize = max(windowSize, abs(dtwDistanceMatrix.cols - dtwDistanceMatrix.rows));
  }

  int newI = 1;
  int newCondition = dtwDistanceMatrix.rows;

  for (int i = 1; i < dtwDistanceMatrix.cols; ++i) {

    if(activeWindowSize){
      newI =  max(1, i-currentWindowSize);
      newCondition = min(dtwDistanceMatrix.rows,i+currentWindowSize);
    }

    for (int j = newI; j < newCondition; ++j) {
      insertion = dtwDistanceMatrix.at<double>(j, i-1);
      deletion = dtwDistanceMatrix.at<double>(j-1, i);
      match = dtwDistanceMatrix.at<double>(j-1, i-1);

      if(slopeWeights){
        insertion += distanceMatrix.at<double>(j-1,i-1);
        deletion +=  distanceMatrix.at<double>(j-1,i-1);
        match += 2*distanceMatrix.at<double>(j-1,i-1);

        dtwValue =  minimum(insertion, deletion, match);
      }else{
        dtwValue =  distanceMatrix.at<double>(j-1,i-1) + minimum(insertion, deletion, match);
      }
      //ROS_INFO("match %f value %f", match, dtwValue);


      dtwDistanceMatrix.at<double>(j,i) = dtwValue;
    }
  }
}

void Dtw::applyStepPatternType2(bool activeWindowSize, int windowSize, bool windowAdapted)
{
  double insertion = 0.0;
  double deletion = 0.0;
  double match = 0.0;

  double insertionDistance = 0.0;
  double deletionDistance = 0.0;
  double matchDistance = 0.0;

  double dtwValue = 0.0;

  int currentWindowSize = windowSize;
  if(activeWindowSize && windowAdapted){
    currentWindowSize = max(windowSize, abs(dtwDistanceMatrix.cols - dtwDistanceMatrix.rows));
  }

  int newI = 1;
  int newCondition = dtwDistanceMatrix.rows;

  for (int i = 1; i < dtwDistanceMatrix.cols; ++i) {

    if(activeWindowSize){
      newI =  max(1, i-currentWindowSize);
      newCondition = min(dtwDistanceMatrix.rows,i+currentWindowSize);
    }

    for (int j = newI; j < newCondition; ++j) {
      insertion = dtwDistanceMatrix.at<double>(j-1, i-2);
      deletion = dtwDistanceMatrix.at<double>(j-1, i-1);
      match = dtwDistanceMatrix.at<double>(j-2, i-1);

      insertionDistance = distanceMatrix.at<double>(j-1,i-2) + distanceMatrix.at<double>(j-1,i-1);
      deletionDistance = distanceMatrix.at<double>(j-1,i-1);
      matchDistance = distanceMatrix.at<double>(j-2,i-1) + distanceMatrix.at<double>(j-1,i-1);

      if(slopeWeights){
        insertion += 0.5*(insertionDistance);
        deletion +=  deletionDistance;
        match += 0.5*(matchDistance);
      }else{
        insertion += insertionDistance;
        deletion +=  deletionDistance;
        match += matchDistance;
      }

      dtwValue =  minimum(insertion, deletion, match);

      dtwDistanceMatrix.at<double>(j,i) = dtwValue;
    }
  }
}

void Dtw::applyStepPatternType3(bool activeWindowSize, int windowSize, bool windowAdapted)
{
  double insertion = 0.0;
  double deletion = 0.0;
  double match = 0.0;

  double dtwValue = 0.0;

  int currentWindowSize = windowSize;
  if(activeWindowSize && windowAdapted){
    currentWindowSize = max(windowSize, abs(dtwDistanceMatrix.cols - dtwDistanceMatrix.rows));
  }

  int newI = 1;
  int newCondition = dtwDistanceMatrix.rows;

  for (int i = 1; i < dtwDistanceMatrix.cols; ++i) {

    if(activeWindowSize){
      newI =  max(1, i-currentWindowSize);
      newCondition = min(dtwDistanceMatrix.rows,i+currentWindowSize);
    }

    for (int j = newI; j < newCondition; ++j) {
      insertion = dtwDistanceMatrix.at<double>(j-1, i-1);
      deletion = dtwDistanceMatrix.at<double>(j-2, i-1);
      match = dtwDistanceMatrix.at<double>(j-1, i-2);

      if(slopeWeights){
        insertion += 2*distanceMatrix.at<double>(j-1,i-1);
        deletion +=  3*distanceMatrix.at<double>(j-1,i-1);
        match += 3*distanceMatrix.at<double>(j-1,i-1);

        dtwValue =  minimum(insertion, deletion, match);
      }else{
        dtwValue =  distanceMatrix.at<double>(j-1,i-1) + minimum(insertion, deletion, match);
      }

      dtwDistanceMatrix.at<double>(j,i) = dtwValue;
    }
  }
}

void Dtw::applyStepPatternType4(bool activeWindowSize, int windowSize, bool windowAdapted)
{
  double insertion = 0.0;
  double deletion = 0.0;
  double match = 0.0;
  double match2 = 0.0;

  double insertionDistance = 0.0;
  double deletionDistance = 0.0;
  double matchDistance = 0.0;
  double match2Distance = 0.0;

  double dtwValue = 0.0;

  int currentWindowSize = windowSize;
  if(activeWindowSize && windowAdapted){
    currentWindowSize = max(windowSize, abs(dtwDistanceMatrix.cols - dtwDistanceMatrix.rows));
  }

  int newI = 1;
  int newCondition = dtwDistanceMatrix.rows;

  for (int i = 1; i < dtwDistanceMatrix.cols; ++i) {

    if(activeWindowSize){
      newI =  max(1, i-currentWindowSize);
      newCondition = min(dtwDistanceMatrix.rows,i+currentWindowSize);
    }

    for (int j = newI; j < newCondition; ++j) {
      insertion = dtwDistanceMatrix.at<double>(j-1, i-2);
      deletion = dtwDistanceMatrix.at<double>(j-2, i-2);
      match = dtwDistanceMatrix.at<double>(j-1, i-1);
      match2 = dtwDistanceMatrix.at<double>(j-2, i-1);

      insertionDistance = distanceMatrix.at<double>(j-1,i-2) + distanceMatrix.at<double>(j-1,i-1);
      deletionDistance = distanceMatrix.at<double>(j-1,i-2) + distanceMatrix.at<double>(j-1,i-1);
      matchDistance =  distanceMatrix.at<double>(j-1,i-1);
      match2Distance = distanceMatrix.at<double>(j-1,i-1);

      insertion += insertionDistance;
      deletion += deletionDistance;
      match += matchDistance;
      match2 += match2Distance;

      dtwValue = minimum(insertion, deletion, match);
      dtwValue = minimum(dtwValue, match, match2);

      dtwDistanceMatrix.at<double>(j,i) = dtwValue;
    }
  }
}

void Dtw::applyStepPatternType5(bool activeWindowSize, int windowSize, bool windowAdapted)
{


  double insertion = 0.0;
  double deletion = 0.0;
  double match = 0.0;
  double insertion2 = 0.0;
  double deletion2 = 0.0;

  double dtwValue = 0.0;

  int currentWindowSize = windowSize;
  if(activeWindowSize && windowAdapted){
    currentWindowSize = max(windowSize, abs(dtwDistanceMatrix.cols - dtwDistanceMatrix.rows));
  }

  int newI = 1;
  int newCondition = dtwDistanceMatrix.rows;

  for (int i = 1; i < dtwDistanceMatrix.cols; ++i) {

    if(activeWindowSize){
      newI =  max(1, i-currentWindowSize);
      newCondition = min(dtwDistanceMatrix.rows,i+currentWindowSize);
    }

    for (int j = newI; j < newCondition; ++j) {
      insertion = dtwDistanceMatrix.at<double>(j-1, i-3);
      deletion = dtwDistanceMatrix.at<double>(j-1, i-2);
      match = dtwDistanceMatrix.at<double>(j-1, i-1);
      insertion2 = dtwDistanceMatrix.at<double>(j-2, i-1);
      deletion2 = dtwDistanceMatrix.at<double>(j-3, i-1);

      if(slopeWeights){
        insertion += 2*distanceMatrix.at<double>(j-1,i-3)+distanceMatrix.at<double>(j-1,i-2)+distanceMatrix.at<double>(j-1,i-1);
        deletion +=  2*distanceMatrix.at<double>(j-1,i-2)+distanceMatrix.at<double>(j-1,i-1);
        match += 2*distanceMatrix.at<double>(j-1,i-1);
        insertion2 += 2*distanceMatrix.at<double>(j-2,i-1)+distanceMatrix.at<double>(j-1,i-1);
        deletion2 += 2*distanceMatrix.at<double>(j-3,i-1)+distanceMatrix.at<double>(j-2,i-1)+distanceMatrix.at<double>(j-1,i-1);
      }else{
        insertion += distanceMatrix.at<double>(j-1,i-3)+distanceMatrix.at<double>(j-1,i-2)+distanceMatrix.at<double>(j-1,i-1);
        deletion +=  distanceMatrix.at<double>(j-1,i-2)+distanceMatrix.at<double>(j-1,i-1);
        match += distanceMatrix.at<double>(j-1,i-1);
        insertion2 += distanceMatrix.at<double>(j-2,i-1)+distanceMatrix.at<double>(j-1,i-1);
        deletion2 += distanceMatrix.at<double>(j-3,i-1)+distanceMatrix.at<double>(j-2,i-1)+distanceMatrix.at<double>(j-1,i-1);
      }

      dtwValue = minimum(insertion, deletion, match);
      dtwValue = minimum(dtwValue, insertion2, deletion2);

      dtwDistanceMatrix.at<double>(j,i) = dtwValue;
    }
  }
}

void Dtw::applyStepPatternItakura(bool activeWindowSize, int windowSize, bool windowAdapted)
{
  double insertion = 0.0;
  double deletion = 0.0;
  double match = 0.0;

  double dtwValue = 0.0;

  int currentWindowSize = windowSize;
  if(activeWindowSize && windowAdapted){
    currentWindowSize = max(windowSize, abs(dtwDistanceMatrix.cols - dtwDistanceMatrix.rows));
  }

  int newI = 1;
  int newCondition = dtwDistanceMatrix.rows;

  for (int i = 1; i < dtwDistanceMatrix.cols; ++i) {

    if(activeWindowSize){
      newI =  max(1, i-currentWindowSize);
      newCondition = min(dtwDistanceMatrix.rows,i+currentWindowSize);
    }

    for (int j = newI; j < newCondition; ++j) {
      insertion = dtwDistanceMatrix.at<double>(j, i-1);
      deletion = dtwDistanceMatrix.at<double>(j-1, i-1);
      match = dtwDistanceMatrix.at<double>(j-2, i-2);

      insertion += distanceMatrix.at<double>(j-1,i-1);;
      deletion += distanceMatrix.at<double>(j-1,i-1);;
      match += distanceMatrix.at<double>(j-1,i-1);;

      dtwValue = minimum(insertion, deletion, match);

      dtwDistanceMatrix.at<double>(j,i) = dtwValue;
    }
  }
}

double Dtw::getWarpingPathCost()
{
  double cost = 0.0;

//  for (int i = 0; i < this->warpingPath.size(); ++i) {
//    cost += dtwDistanceMatrix.at<double>(warpingPath.at(i).y,warpingPath.at(i).x);
//  }
  cost = dtwDistanceMatrix.at<double>(dtwDistanceMatrix.rows-1, dtwDistanceMatrix.cols-1);

  return cost;
}

void Dtw::printDistanceMatrix()
{
  QString str = "";
  ROS_INFO("########DistanceMatrix1########");
  for (int i = 0; i < distanceMatrix.rows; ++i) {
    str = "|";
    for (int j = 0; j < distanceMatrix.cols; ++j) {
      str += (QString::number(distanceMatrix.at<double>(i,j)) + "|");
    }
    ROS_INFO("%s", str.toStdString().c_str());
  }
  ROS_INFO("########DistanceMatrix2########");
}

void Dtw::printDtwDistanceMatric()
{
  QString str = "";
  ROS_INFO("########DtwDistanceMatrix1########");
  for (int i = 0; i < dtwDistanceMatrix.rows; ++i) {
    str = "|";
    for (int j = 0; j < dtwDistanceMatrix.cols; ++j) {
      str += (QString::number(dtwDistanceMatrix.at<double>(i,j)) + "|");
    }
    ROS_INFO("%s", str.toStdString().c_str());
  }
  ROS_INFO("########DtwDistanceMatrix2########");
}

double Dtw::calcWarpingCost(DistanceFunction df, bool activeWindow, int windowSize, bool windowAdapted)
{
  double warpingCost = 0.0;

  this->calculateDistanceMatrix(df);
  this->calculateDtwDistanceMatrix(activeWindow, windowSize, windowAdapted);
  this->calculateGreedyWarpingPath();

  warpingCost = this->getWarpingPathCost();

  return warpingCost;
}

Mat Dtw::getDtwDistanceMatrix() const
{
  return dtwDistanceMatrix;
}

Mat Dtw::getDistanceMatrix() const
{
  return distanceMatrix;
}

QList<Point> Dtw::getWarpingPath() const
{
  return warpingPath;
}


double Dtw::squareDistance(double val, double val2)
{
  return sqrt(val*val + val2*val2);
}

double Dtw::square2Distance(double val, double val2)
{
  return (val-val2) * (val-val2);
}

double Dtw::absDistance(double val, double val2)
{
  return fabs(val - val2);
}

double Dtw::minimum(double insertion, double deletion, double match)
{
  return min(min(insertion, deletion), match);
}
