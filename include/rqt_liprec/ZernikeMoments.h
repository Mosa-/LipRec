#ifndef ZERNIKEMOMENTS_H_
#define ZERNIKEMOMENTS_H_


#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <complex>


using namespace cv;
using namespace std;

// n = order, l = repetition

class ZernikeMoments {
public:

    ZernikeMoments(int imgSize);
    virtual ~ZernikeMoments();

private:
    int imgSize;

    double calcRadialPolynomial(int n, int l, double radiusDist);
    double calcZernikePolynomial(double radiusDist, int angle);
    double calcZernikeMoments();

    int fak(int n);

    double calcXCircularCoordinate(int imgSize, int xSquareCoordinate);
    double calcYCircularCoordinate(int imgSize, int ySquareCoordinate);
    double calcRadiusDist(int imgSize, int xSquareCoordinate, int ySquareCoordinate);
    double calcAngleCircular(int imgSize, int xSquareCoordinate, int ySquareCoordinate);

};


#endif /* ZERNIKEMOMENTS_H_ */
