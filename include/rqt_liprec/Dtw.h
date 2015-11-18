#ifndef DTW_H_
#define DTW_H_

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <QtCore>

using namespace cv;
using namespace std;

class Dtw {
public:
    enum DistanceFunction{SQUARE, SQUARE2, ABS};

    Dtw();
    Dtw(QList<double> trajectory1, QList<double> trajectory2);
    virtual ~Dtw();

    void seed(QList<double> trajectory1, QList<double> trajectory2);

    Mat calculateDistanceMatrix(DistanceFunction distanceFunction);
    Mat calculateDtwDistanceMatrix();

    QList<Point> calculateGreedyWarpingPath();

    void printDistanceMatrix();
    void printDtwDistanceMatric();

private:
    QList<double> trajectory1;
    QList<double> trajectory2;
    Mat distanceMatrix;
    Mat dtwDistanceMatrix;

    double squareDistance(double val, double val2);
    double square2Distance(double val, double val2);
    double absDistance(double val, double val2);

    double minimum(double insertion, double deletion, double match);



};

#endif /* DTW_H_ */
