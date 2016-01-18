#include "rqt_liprec/Dtw.h"


Dtw::Dtw()
{

}

Dtw::Dtw(QList<double> trajectory1, QList<double> trajectory2, DtwStepPattern stepPattern){

  this->trajectory1 = trajectory1;
  this->trajectory2 = trajectory2;
  this->stepPattern = stepPattern;
}

Dtw::~Dtw(){

}

void Dtw::seed(QList<double> trajectory1, QList<double> trajectory2, DtwStepPattern stepPattern)
{
  this->trajectory1 = trajectory1;
  this->trajectory2 = trajectory2;
  this->stepPattern = stepPattern;

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

Mat Dtw::calculateDtwDistanceMatrix(int windowSize, bool windowAdapted){
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

  for (int i = 0; i < dtwDistanceMatrix.cols; ++i) {
    for (int j = 0; j < dtwDistanceMatrix.rows; ++j) {
      dtwDistanceMatrix.at<double>(j,i) = INT_MAX;
    }
  }

  dtwDistanceMatrix.at<double>(0,0) = 0.0;
  dtwDistanceMatrix.at<double>(1,1) = distanceMatrix.at<double>(0,0);

  double insertion = 0.0;
  double deletion = 0.0;
  double match = 0.0;
  double insertion2 = 0.0;
  double deletion2 = 0.0;

  if(windowAdapted){
    windowSize = max(windowSize, abs(dtwDistanceMatrix.cols - dtwDistanceMatrix.rows));
  }

  if(stepPattern == BELLMANSTEP){
    for (int i = 1; i < dtwDistanceMatrix.cols; ++i) {
      for (int j = max(1, i-windowSize); j < min(dtwDistanceMatrix.rows,i+windowSize); ++j) {
        insertion = dtwDistanceMatrix.at<double>(j, i-1);
        deletion = dtwDistanceMatrix.at<double>(j-1, i);
        match = dtwDistanceMatrix.at<double>(j-1, i-1);

        dtwDistanceMatrix.at<double>(j,i) = distanceMatrix.at<double>(j-1,i-1) +
            minimum(insertion, deletion, match);
      }
    }
  }else if(stepPattern == DIAGONALSTEP){
    for (int i = 1; i < dtwDistanceMatrix.cols; ++i) {
      for (int j = max(1, i-windowSize); j < min(dtwDistanceMatrix.rows,i+windowSize); ++j) {
        insertion = dtwDistanceMatrix.at<double>(j-1, i-1);
        deletion = dtwDistanceMatrix.at<double>(j-2, i-1);
        match = dtwDistanceMatrix.at<double>(j-1, i-2);

        dtwDistanceMatrix.at<double>(j,i) = distanceMatrix.at<double>(j-1,i-1) +
            minimum(insertion, deletion, match);
      }
    }
  }else if(stepPattern == ITAKURASTEP){
    for (int i = 1; i < dtwDistanceMatrix.cols; ++i) {
      for (int j = max(1, i-windowSize); j < min(dtwDistanceMatrix.rows,i+windowSize); ++j) {
        insertion = dtwDistanceMatrix.at<double>(j-1, i-1);
        deletion = dtwDistanceMatrix.at<double>(j-2, i-3);
        match = dtwDistanceMatrix.at<double>(j-3, i-2);

        dtwDistanceMatrix.at<double>(j,i) = distanceMatrix.at<double>(j-1,i-1) +
            minimum(insertion, deletion, match);
      }
    }
  }else{
    for (int i = 1; i < dtwDistanceMatrix.cols; ++i) {
      for (int j = max(1, i-windowSize); j < min(dtwDistanceMatrix.rows,i+windowSize); ++j) {
        insertion = dtwDistanceMatrix.at<double>(j-1, i-1) + distanceMatrix.at<double>(j-1,i-1);
        deletion = dtwDistanceMatrix.at<double>(j-1, i-2) + distanceMatrix.at<double>(j-1,i-2) + distanceMatrix.at<double>(j-1,i-1);
        match = dtwDistanceMatrix.at<double>(j-2, i-1) + distanceMatrix.at<double>(j-2,i-1) + distanceMatrix.at<double>(j-1,i-1);
        insertion2 = dtwDistanceMatrix.at<double>(j-3, i-1) + distanceMatrix.at<double>(j-3,i-1) + distanceMatrix.at<double>(j-2,i-1) + distanceMatrix.at<double>(j-1,i-1);
        deletion2 = dtwDistanceMatrix.at<double>(j-1, i-3) + distanceMatrix.at<double>(j-1,i-3) + distanceMatrix.at<double>(j-1,i-2) + distanceMatrix.at<double>(j-1,i-1);

        double tmpMin = minimum(insertion, deletion, match);

        dtwDistanceMatrix.at<double>(j,i) = minimum(tmpMin, insertion2, deletion2);
      }
    }
  }

  return dtwDistanceMatrix;
}

Mat Dtw::calculateDtwDistanceMatrix(){
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

  dtwDistanceMatrix.at<double>(0,0) = 0.0;

  double insertion = 0.0;
  double deletion = 0.0;
  double match = 0.0;
  double insertion2 = 0.0;
  double deletion2 = 0.0;

  if(stepPattern == BELLMANSTEP){
    for (int i = 1; i < dtwDistanceMatrix.cols; ++i) {
      for (int j = 1; j < dtwDistanceMatrix.rows; ++j) {
        insertion = dtwDistanceMatrix.at<double>(j, i-1);
        deletion = dtwDistanceMatrix.at<double>(j-1, i);
        match = dtwDistanceMatrix.at<double>(j-1, i-1);

        dtwDistanceMatrix.at<double>(j,i) = distanceMatrix.at<double>(j-1,i-1) +
            minimum(insertion, deletion, match);
      }
    }
  }else if(stepPattern == DIAGONALSTEP){
    for (int i = 1; i < dtwDistanceMatrix.cols; ++i) {
      for (int j = 1; j < dtwDistanceMatrix.rows; ++j) {
        insertion = dtwDistanceMatrix.at<double>(j-1, i-1);
        deletion = dtwDistanceMatrix.at<double>(j-2, i-1);
        match = dtwDistanceMatrix.at<double>(j-1, i-2);

        dtwDistanceMatrix.at<double>(j,i) = distanceMatrix.at<double>(j-1,i-1) +
            minimum(insertion, deletion, match);
      }
    }
  }else if(stepPattern == ITAKURASTEP){
    for (int i = 1; i < dtwDistanceMatrix.cols; ++i) {
      for (int j = 1; j < dtwDistanceMatrix.rows; ++j) {
        insertion = dtwDistanceMatrix.at<double>(j-1, i-1);
        deletion = dtwDistanceMatrix.at<double>(j-2, i-3);
        match = dtwDistanceMatrix.at<double>(j-3, i-2);

        dtwDistanceMatrix.at<double>(j,i) = distanceMatrix.at<double>(j-1,i-1) +
            minimum(insertion, deletion, match);
      }
    }
  }else{
    for (int i = 1; i < dtwDistanceMatrix.cols; ++i) {
      for (int j = 1; j < dtwDistanceMatrix.rows; ++j) {
        insertion = dtwDistanceMatrix.at<double>(j-1, i-1) + distanceMatrix.at<double>(j-1,i-1);
        deletion = dtwDistanceMatrix.at<double>(j-2, i-1) + distanceMatrix.at<double>(j-2,i-1) + distanceMatrix.at<double>(j-1,i-1);
        match = dtwDistanceMatrix.at<double>(j-1, i-2) + distanceMatrix.at<double>(j-1,i-2) + distanceMatrix.at<double>(j-1,i-1);
        insertion2 = dtwDistanceMatrix.at<double>(j-3, i-1) + distanceMatrix.at<double>(j-3,i-1) + distanceMatrix.at<double>(j-2,i-1) + distanceMatrix.at<double>(j-1,i-1);
        deletion2 = dtwDistanceMatrix.at<double>(j-1, i-3) + distanceMatrix.at<double>(j-1,i-3) + distanceMatrix.at<double>(j-1,i-2) + distanceMatrix.at<double>(j-1,i-1);

        double tmpMin = minimum(insertion, deletion, match);

        dtwDistanceMatrix.at<double>(j,i) = minimum(tmpMin, insertion2, deletion2);
      }
    }
  }

  return dtwDistanceMatrix;
}

QList<Point> Dtw::calculateGreedyWarpingPath()
{
  QList<Point> warpingPath;
  int i = dtwDistanceMatrix.rows-1;
  int j = dtwDistanceMatrix.cols-1;

  double insertion = 0.0;
  double deletion = 0.0;
  double match = 0.0;
  double insertion2 = 0.0;
  double deletion2 = 0.0;

  if(stepPattern == BELLMANSTEP){

    while(i>1 && j>1){
      if(i==1){
        j--;
      }else if(j==1){
        i--;
      }else{
        insertion = dtwDistanceMatrix.at<double>(i, j-1);
        deletion = dtwDistanceMatrix.at<double>(i-1, j);
        match = dtwDistanceMatrix.at<double>(i-1, j-1);

        if(deletion == minimum(insertion, deletion, match)){
          i--;
        }else if(insertion == minimum(insertion, deletion, match)){
          j--;
        }else{
          i--;
          j--;
        }
        warpingPath.append(Point(j,i));
      }
    }

  }else if(stepPattern == DIAGONALSTEP){

    while(i>1 && j>1){
      if(i==2){
        j -= 2;
      }else if(j==2){
        i -= 2;
      }else{
        insertion = dtwDistanceMatrix.at<double>(i-1, j-1);
        deletion = dtwDistanceMatrix.at<double>(i-2, j-1);
        match = dtwDistanceMatrix.at<double>(i-1, j-2);

        if(insertion == minimum(insertion, deletion, match)){
          i--;
          j--;
        }else if(deletion == minimum(insertion, deletion, match)){
          i -= 2;
          j--;
        }else{
          i--;
          j -= 2;
        }
        warpingPath.append(Point(j,i));
      }
    }

  }else if(stepPattern == ITAKURASTEP){
    while(i>1 && j>1){
      if(i==1){
        j--;
      }else if(j==1){
        i --;
      }else{
        insertion = dtwDistanceMatrix.at<double>(i-1, j-1);
        deletion = dtwDistanceMatrix.at<double>(i-2, j-3);
        match = dtwDistanceMatrix.at<double>(i-3, j-2);

        if(insertion == minimum(insertion, deletion, match)){
          i--;
          j--;
        }else if(deletion == minimum(insertion, deletion, match)){
          i -= 2;
          j -= 3;
        }else{
          i -= 3;
          j -= 2;
        }
        warpingPath.append(Point(j,i));
      }
    }
  }else{

    while(i>1 && j>1){
      if(i==3){
        j -= 3;
      }else if(j==3){
        i -= 3;
      }else{
        insertion = dtwDistanceMatrix.at<double>(i-1, j-1) + distanceMatrix.at<double>(i-1,j-1);
        deletion = dtwDistanceMatrix.at<double>(i-2, j-1) + distanceMatrix.at<double>(i-2,j-1) + distanceMatrix.at<double>(i-1,j-1);
        match = dtwDistanceMatrix.at<double>(i-1, j-2) + distanceMatrix.at<double>(i-1,j-2) + distanceMatrix.at<double>(i-1,j-1);
        insertion2 = dtwDistanceMatrix.at<double>(i-3, j-1) + distanceMatrix.at<double>(i-3,j-1) + distanceMatrix.at<double>(i-2,j-1) + distanceMatrix.at<double>(i-1,j-1);
        deletion2 = dtwDistanceMatrix.at<double>(i-1, j-3) + distanceMatrix.at<double>(i-1,j-3) + distanceMatrix.at<double>(i-1,j-2) + distanceMatrix.at<double>(i-1,j-1);

        double tmpMin = minimum(insertion, deletion, match);

        if(insertion == minimum(tmpMin, insertion2, deletion2)){
          i--;
          j--;
        }else if(deletion == minimum(tmpMin, insertion2, deletion2)){
          i -= 2;
          j--;
        }else if(match == minimum(tmpMin, insertion2, deletion2)){
          i--;
          j -= 2;
        }else if(insertion2 == minimum(tmpMin, insertion2, deletion2)){
          i -= 3;
          j--;
        }else{
          i--;
          j -= 3;
        }
        warpingPath.append(Point(j,i));
      }
    }


  }


  this->warpingPath = warpingPath;

  return warpingPath;
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

double Dtw::calcWarpingCost(DistanceFunction df, int windowSize, bool windowAdapted)
{
  double warpingCost = 0.0;

  this->calculateDistanceMatrix(df);
  this->calculateDtwDistanceMatrix(windowSize, windowAdapted);
  this->calculateGreedyWarpingPath();

  warpingCost = this->getWarpingPathCost();

  return warpingCost;
}

double Dtw::calcWarpingCost(DistanceFunction df)
{
  double warpingCost = 0.0;

  this->calculateDistanceMatrix(df);
  this->calculateDtwDistanceMatrix();
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
