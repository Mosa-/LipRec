#include "rqt_liprec/Dtw.h"


Dtw::Dtw()
{

}

Dtw::Dtw(QList<double> trajectory1, QList<double> trajectory2){

    this->trajectory1 = trajectory1;
    this->trajectory2 = trajectory2;
}

Dtw::~Dtw(){

}

void Dtw::seed(QList<double> trajectory1, QList<double> trajectory2)
{
    double min1 = *std::min_element(trajectory1.begin(), trajectory1.end());
    double min2 = *std::min_element(trajectory2.begin(), trajectory2.end());
    double max1 = *std::max_element(trajectory1.begin(), trajectory1.end());
    double max2 = *std::max_element(trajectory2.begin(), trajectory2.end());

    this->trajectory1 = trajectory1;
    this->trajectory2 = trajectory2;

    double minFinal = qMin(min1, min2);
    double maxFinal = qMin(max1, max2);

    double newVal = 0.0;
    for (int i = 0; i < trajectory1.size(); ++i) {
        trajectory1[i] = ((trajectory1.at(i) - minFinal)/((maxFinal-minFinal)));
        newVal = (trajectory1[i] - minFinal) / (maxFinal - minFinal);
        ROS_INFO("%f", newVal);
    }

    for (int i = 0; i < trajectory2.size(); ++i) {
        trajectory2[i] = ((trajectory2.at(i) - minFinal)/((maxFinal-minFinal)));
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

    for (int i = 1; i < dtwDistanceMatrix.cols; ++i) {
        for (int j = 1; j < dtwDistanceMatrix.rows; ++j) {
            insertion = dtwDistanceMatrix.at<double>(j, i-1);
            deletion = dtwDistanceMatrix.at<double>(j-1, i);
            match = dtwDistanceMatrix.at<double>(j-1, i-1);

            dtwDistanceMatrix.at<double>(j,i) = distanceMatrix.at<double>(j-1,i-1) +
                    minimum(insertion, deletion, match);
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

    while(i>1 && j>1){
        if(i==1){
            j--;
        }else if(j==1){
            i--;
        }else{
            insertion = dtwDistanceMatrix.at<double>(i, j-1);
            deletion = dtwDistanceMatrix.at<double>(i-1, j);
            match = dtwDistanceMatrix.at<double>(i-1, j-1);

            if(dtwDistanceMatrix.at<double>(i-1, j) == minimum(insertion, deletion, match)){
                i--;
            }else if(dtwDistanceMatrix.at<double>(i, j-1) == minimum(insertion, deletion, match)){
                j--;
            }else{
                i--;
                j--;
            }
            warpingPath.append(Point(j,i));
        }
    }

    this->warpingPath = warpingPath;

    return warpingPath;
}

double Dtw::getWarpingPathCost()
{
    double cost = 0.0;

    for (int i = 0; i < this->warpingPath.size(); ++i) {
        cost += dtwDistanceMatrix.at<double>(warpingPath.at(i).y,warpingPath.at(i).x);
    }

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

double Dtw::calcWarpingCost(DistanceFunction df)
{
    double warpingCost = 0.0;

    this->calculateDistanceMatrix(df);
    this->calculateDtwDistanceMatrix();
    this->calculateGreedyWarpingPath();

    warpingCost = this->getWarpingPathCost();

    return warpingCost;
}

double Dtw::squareDistance(double val, double val2)
{
    return sqrt(val*val + val2*val2);
}

double Dtw::square2Distance(double val, double val2)
{
    return pow(val-val2, 2.0);
}

double Dtw::absDistance(double val, double val2)
{
    return fabs(val - val2);
}

double Dtw::minimum(double insertion, double deletion, double match)
{
    return min(min(insertion, deletion), match);
}
