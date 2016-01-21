#ifndef DTW_H_
#define DTW_H_

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <QtCore>
#include "CommonData.h"


using namespace cv;
using namespace std;

class Dtw {
public:

    Dtw();
    Dtw(QList<double> trajectory1, QList<double> trajectory2, DtwStepPattern stepPattern, bool slopeWeights);
    virtual ~Dtw();

    void seed(QList<double> trajectory1, QList<double> trajectory2, DtwStepPattern stepPattern, bool slopeWeights);

    Mat calculateDistanceMatrix(DistanceFunction distanceFunction);
    Mat calculateDtwDistanceMatrix(bool activeWindowSize, int windowSize, bool windowAdapted);

    QList<Point> calculateGreedyWarpingPath();
    double getWarpingPathCost();

    void printDistanceMatrix();
    void printDtwDistanceMatric();

    double calcWarpingCost(DistanceFunction df, bool activeWindow, int windowSize, bool windowAdapted);

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
    bool slopeWeights;

    void applyStepPatternType1(bool activeWindowSize, int windowSize, bool windowAdapted);
    void applyStepPatternType2(bool activeWindowSize, int windowSize, bool windowAdapted);
    void applyStepPatternType3(bool activeWindowSize, int windowSize, bool windowAdapted);
    void applyStepPatternType4(bool activeWindowSize, int windowSize, bool windowAdapted);
    void applyStepPatternType5(bool activeWindowSize, int windowSize, bool windowAdapted);
    void applyStepPatternItakura(bool activeWindowSize, int windowSize, bool windowAdapted);

    double squareDistance(double val, double val2);
    double square2Distance(double val, double val2);
    double absDistance(double val, double val2);

    double minimum(double insertion, double deletion, double match);
};

#endif /* DTW_H_ */
