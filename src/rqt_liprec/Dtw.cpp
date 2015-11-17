#include "rqt_liprec/Dtw.h"


Dtw::Dtw(QList<double> trajectory1, QList<double> trajectory2){

    this->trajectory1 = trajectory1;
    this->trajectory2 = trajectory2;
}

Dtw::~Dtw(){

}

void Dtw::seed(QList<double> trajectory1, QList<double> trajectory2)
{
    this->trajectory1 = trajectory1;
    this->trajectory2 = trajectory2;
}

Mat Dtw::calculateDistanceMatrix(Dtw::DistanceFunction distanceFunction){
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
}

Mat Dtw::calculateDtwDistanceMatrix(){
    dtwDistanceMatrix = distanceMatrix;

    vector<Mat> matrices;

    Mat infRow(dtwDistanceMatrix.rows, 1, CV_64F, Scalar(-1));
    Mat infCol(1, dtwDistanceMatrix.cols+1, CV_64F, Scalar(-1));

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
