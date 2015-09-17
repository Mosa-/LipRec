#ifndef ZERNIKEMOMENTS_H_
#define ZERNIKEMOMENTS_H_


#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "ImageProcessing.h"

using namespace cv;
using namespace std;

class ZernikeMoments {
public:

    ZernikeMoments();
    virtual ~ZernikeMoments();

private:
    ImageProcessing imageProcessing;

    double calcRadialPolynomial();
    double calcZernikeMoments();

};


#endif /* SWT_H_ */
