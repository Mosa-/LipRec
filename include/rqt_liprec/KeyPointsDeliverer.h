#ifndef KEYPOINTSDELIVERER_H_
#define KEYPOINTSDELIVERER_H_

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "ImageProcessing.h"

using namespace cv;
using namespace std;

class KeyPointsDeliverer {
public:
    KeyPointsDeliverer();
    virtual ~KeyPointsDeliverer();

    struct PossibleKeyPoint{
        int differenceToAvg;
        Point keyPoint;
    };

    void calcGradientImages(Mat &mouthImg);

    void extractMouthCornerKeyPoints(Mat &mouthImg, int thresholdDifferenceToAvg, int totalLineCheck);
    void extractCupidsBowKeyPoints(int thresholdDifferenceToAvg, int totalLineCheck);
    void extractLowerLipKeyPoint(int thresholdDifferenceToAvg, int totalLineCheck);


    Mat& getRTop();
    Mat& getRMid();
    Mat& getRLow();

    Point& getKeyPoint1();
    Point& getKeyPoint2();
    Point& getKeyPoint3();
    Point& getKeyPoint4();
    Point& getKeyPoint5();
    Point& getKeyPoint6();


private:
    Point keyPoint1, keyPoint2, keyPoint3, keyPoint4, keyPoint5, keyPoint6;
    ///        P2         P4
    ///       --X-- P3  --X--
    ///      /     \_X_/     \
    ///     /                 \
    /// P1 X-------------------X P5
    ///     \                 /
    ///      \               /
    ///       \______X______/
    ///              P6

    Mat rTopFinal;
    Mat rMidFinal;
    Mat rLowFinal;

    ImageProcessing imageProcessing;

};

#endif /* KEYPOINTSDELIVERER_H_ */
