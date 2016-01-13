#ifndef DTW_H_
#define DTW_H_

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <QtCore>
#include "CommonEnums.h"


using namespace cv;
using namespace std;

class Dtw {
public:

    Dtw();
    Dtw(QList<double> trajectory1, QList<double> trajectory2, DtwStepPattern stepPattern);
    virtual ~Dtw();

    void seed(QList<double> trajectory1, QList<double> trajectory2, DtwStepPattern stepPattern);

    Mat calculateDistanceMatrix(DistanceFunction distanceFunction);
    Mat calculateDtwDistanceMatrix();
    Mat calculateDtwDistanceMatrix(int windowSize, bool windowAdapted);


    QList<Point> calculateGreedyWarpingPath();
    double getWarpingPathCost();

    void printDistanceMatrix();
    void printDtwDistanceMatric();

    double calcWarpingCost(DistanceFunction df);
    double calcWarpingCost(DistanceFunction df, int windowSize, bool windowAdapted);


    Mat getDtwDistanceMatrix() const;
    Mat getDistanceMatrix() const;
    QList<Point> getWarpingPath() const;

private:
    QList<double> trajectory1;
    QList<double> trajectory2;
    Mat distanceMatrix;
    Mat dtwDistanceMatrix;
    QList<Point> warpingPath;
    DtwStepPattern stepPattern;

    double squareDistance(double val, double val2);
    double square2Distance(double val, double val2);
    double absDistance(double val, double val2);

    double minimum(double insertion, double deletion, double match);



};

#endif /* DTW_H_ */
