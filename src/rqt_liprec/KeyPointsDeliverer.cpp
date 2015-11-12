#include "rqt_liprec/KeyPointsDeliverer.h"


KeyPointsDeliverer::KeyPointsDeliverer(){

}

KeyPointsDeliverer::~KeyPointsDeliverer(){

}

void KeyPointsDeliverer::calcGradientImages(Mat& mouthImg)
{
    double pseudoHuePxl = 0.0;
    int luminancePxl = 0;

    Mat rTop(mouthImg.rows, mouthImg.cols, CV_8UC1);
    Mat rMid(mouthImg.rows, mouthImg.cols, CV_8UC1);
    Mat rLow(mouthImg.rows, mouthImg.cols, CV_8UC1);


    for (int i = 0; i < mouthImg.rows; ++i) {
        for (int j = 0; j < mouthImg.cols; ++j) {
            pseudoHuePxl = imageProcessing.pseudoHuePxl(mouthImg, j, i);
            luminancePxl = imageProcessing.luminancePxl(mouthImg, j, i);

            rTop.at<uchar>(i,j) = (int) (pseudoHuePxl - luminancePxl);
            rMid.at<uchar>(i,j) = (int) (pseudoHuePxl - luminancePxl);
            rLow.at<uchar>(i,j) = (int) (pseudoHuePxl + luminancePxl);
        }
    }

    double minVal, maxVal;

    Mat rTopTemp;
    //Sobel(rTop, rTopTemp, CV_32FC1, 0, 1);
    Scharr(rTop, rTopTemp, CV_32FC1, 0, 1);
    minMaxLoc(rTopTemp, &minVal, &maxVal);
    rTopTemp.convertTo(rTopFinal, CV_8UC1, 255.0/(maxVal - minVal), -minVal * 255.0/(maxVal - minVal));

    Mat rMidTemp;
    Sobel(rMid, rMidTemp, CV_32FC1, 0, 1);
    minMaxLoc(rMidTemp, &minVal, &maxVal);
    rMidTemp.convertTo(rMidFinal, CV_8UC1, 255.0/(maxVal - minVal), -minVal * 255.0/(maxVal - minVal));

    for (int i = 0; i < mouthImg.rows; ++i) {
        for (int j = 0; j < mouthImg.cols; ++j) {
            pseudoHuePxl = imageProcessing.pseudoHuePxl(mouthImg, j, i);

            rMidFinal.at<uchar>(i,j) = (int) (rMidFinal.at<uchar>(i,j) * pseudoHuePxl);
        }
    }

    Mat rLowTemp;
    //Sobel(rLow, rLowTemp, CV_32FC1, 0, 1);
    Scharr(rLow, rLowTemp, CV_32FC1, 0, 1);
    minMaxLoc(rLowTemp, &minVal, &maxVal);
    rLowTemp.convertTo(rLowFinal, CV_8UC1, 255.0/(maxVal - minVal), -minVal * 255.0/(maxVal - minVal));
}

void KeyPointsDeliverer::extractMouthCornerKeyPoints(Mat &mouthImg, int thresholdDifferenceToAvg, int totalLineCheck)
{
    QList<PossibleKeyPoint> possibleKeyPoints;
    PossibleKeyPoint possibleKeyPoint;

    for (int i = 0; i < rMidFinal.cols; ++i) {
        for (int j = rMidFinal.rows-totalLineCheck; j > totalLineCheck/2; --j) {

            int currentDiffToAvg = 0;

            for (int k = 1; k < totalLineCheck/2 + 1; ++k) {
                currentDiffToAvg += rMidFinal.at<uchar>(j-k,i) + rMidFinal.at<uchar>(j+k,i);

            }
            currentDiffToAvg = currentDiffToAvg / totalLineCheck;

            if(currentDiffToAvg > 0){
                currentDiffToAvg = 100 - (rMidFinal.at<uchar>(j,i) * 100 / currentDiffToAvg);
            }

            if(currentDiffToAvg > thresholdDifferenceToAvg){
                possibleKeyPoint.differenceToAvg = currentDiffToAvg;
                possibleKeyPoint.keyPoint.x  = i;
                possibleKeyPoint.keyPoint.y  = j;
                possibleKeyPoints.append(possibleKeyPoint);
            }
        }
    }

    Mat contourImg(rMidFinal.rows, rMidFinal.cols, CV_8UC1, Scalar(0,0,0));
    Point p;
    for (int i = 0; i < possibleKeyPoints.size(); ++i) {
        p = possibleKeyPoints.at(i).keyPoint;
        contourImg.at<uchar>(p.y, p.x) = 255;
    }
    Mat _img;
    double otsu_thresh_val = cv::threshold(
                contourImg, _img, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU
                );

    Canny(contourImg, contourImg, otsu_thresh_val*0.5, otsu_thresh_val, 3, true);

    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours( contourImg, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
    for( int i = 0; i< contours.size(); i++ )
    {
        drawContours( contourImg, contours, i, Scalar(255,255,255), 1, 8, hierarchy, 1, Point() );
    }

    double luminanceMean = 0.0;
    double pseudoHueMean = 0.0;
    double pseudoHuePxl = 0.0;
    int luminancePxl = 0;

    for (int i = 0; i < mouthImg.cols/2; ++i) {
        for (int j = 0; j < mouthImg.rows; ++j) {
            pseudoHuePxl = imageProcessing.pseudoHuePxl(mouthImg, j, i);
            luminancePxl = imageProcessing.luminancePxl(mouthImg, j, i);

            luminanceMean += luminancePxl;
            pseudoHueMean += pseudoHuePxl;
        }
    }
    luminanceMean /= (mouthImg.cols/2*mouthImg.rows);
    pseudoHueMean /= (mouthImg.cols/2*mouthImg.rows);

    QList<PossibleKeyPoint> pKPoints;
    PossibleKeyPoint pKPoint;

    for (int i = mouthImg.cols/2-(mouthImg.cols/2*0.4); i > 0; --i) {
        for (int j = mouthImg.rows; j > 0; --j) {
            pseudoHuePxl = imageProcessing.pseudoHuePxl(mouthImg, j, i);
            luminancePxl = imageProcessing.luminancePxl(mouthImg, j, i);


            if(contourImg.at<uchar>(j,i) == 255
                    ){
                pKPoint.keyPoint.x = i;
                pKPoint.keyPoint.y = j;
                pKPoints.append(pKPoint);
                break;
            }
        }
    }


    keyPoint1.x = 1000;
    for (int i = 0; i < pKPoints.size(); ++i) {
        int diffY = 0;

        if(i > 0){
            diffY = abs(pKPoints.at(i).keyPoint.y - pKPoints.at(i-1).keyPoint.y);
            //ROS_INFO("diff: %d", abs(pKPoints.at(i).keyPoint.y - pKPoints.at(i-1).keyPoint.y));
        }

        if(diffY > 3){
            break;
        }

        if(keyPoint1.x > pKPoints.at(i).keyPoint.x){
            keyPoint1.x = pKPoints.at(i).keyPoint.x;
            keyPoint1.y = pKPoints.at(i).keyPoint.y;
        }
        //circle(rMidFinal, pKPoints.at(i).keyPoint, 2, Scalar(255,255,255));
    }


    luminanceMean = 0.0;
    pseudoHueMean = 0.0;
    for (int i = mouthImg.cols/2; i < mouthImg.cols; ++i) {
        for (int j = 0; j < mouthImg.rows; ++j) {
            pseudoHuePxl = imageProcessing.pseudoHuePxl(mouthImg, j, i);
            luminancePxl = imageProcessing.luminancePxl(mouthImg, j, i);

            luminanceMean += luminancePxl;
            pseudoHueMean += pseudoHuePxl;
        }
    }
    luminanceMean /= (mouthImg.cols/2*mouthImg.rows);
    pseudoHueMean /= (mouthImg.cols/2*mouthImg.rows);

    pKPoints.clear();

    for (int i = mouthImg.cols/2+(mouthImg.cols/2*0.3); i < mouthImg.cols; ++i) {
        for (int j = mouthImg.rows; j > 0; --j) {
            pseudoHuePxl = imageProcessing.pseudoHuePxl(mouthImg, j, i);
            luminancePxl = imageProcessing.luminancePxl(mouthImg, j, i);

            if(contourImg.at<uchar>(j,i) == 255
                    ){
                pKPoint.keyPoint.x = i;
                pKPoint.keyPoint.y = j;
                pKPoints.append(pKPoint);
                break;
            }
        }
    }

//    for (int i = 0; i < pKPoints.size(); ++i) {
//        circle(rMidFinal, pKPoints.at(i).keyPoint, 1, Scalar(255,255,255));
//    }

//    for (int i = 0; i < contourImg.cols; ++i) {
//        for (int j = 0; j < contourImg.rows; ++j) {
//            if(contourImg.at<uchar>(j,i) == 255)
//                circle(rMidFinal, Point(i,j), 1, Scalar(255,255,255));
//        }
//    }


    keyPoint5.x = 0;
    for (int i = 0; i < pKPoints.size(); ++i) {
        int diffY = 0;

        if(i > 0){
            diffY = abs(pKPoints.at(i).keyPoint.y - pKPoints.at(i-1).keyPoint.y);
            //ROS_INFO("diff: %d", abs(pKPoints.at(i).keyPoint.y - pKPoints.at(i-1).keyPoint.y));
        }

        if(diffY > 3){
            break;
        }

        if(keyPoint5.x < pKPoints.at(i).keyPoint.x){
            keyPoint5.x = pKPoints.at(i).keyPoint.x;
            keyPoint5.y = pKPoints.at(i).keyPoint.y;
        }
        //circle(rMidFinal, pKPoints.at(i).keyPoint, 2, Scalar(255,255,255));
    }

}

void KeyPointsDeliverer::extractCupidsBowKeyPoints(int thresholdDifferenceToAvg, int totalLineCheck)
{
    QList<PossibleKeyPoint> possibleKeyPoints;
    PossibleKeyPoint possibleKeyPoint;

    for (int i = 0; i < rTopFinal.cols; ++i) {
        for (int j = rTopFinal.rows/2; j > totalLineCheck/2; --j) {

            int currentDiffToAvg = 0;

            for (int k = 1; k < totalLineCheck/2 + 1; ++k) {
                currentDiffToAvg += rTopFinal.at<uchar>(j-k,i) + rTopFinal.at<uchar>(j+k,i);

            }
            currentDiffToAvg = currentDiffToAvg / totalLineCheck;

            if(currentDiffToAvg > 0){
                currentDiffToAvg = 100 - (rTopFinal.at<uchar>(j,i) * 100 / currentDiffToAvg);
            }

            if(currentDiffToAvg > thresholdDifferenceToAvg){
                possibleKeyPoint.differenceToAvg = currentDiffToAvg;
                possibleKeyPoint.keyPoint.x  = i;
                possibleKeyPoint.keyPoint.y  = j;
                possibleKeyPoints.append(possibleKeyPoint);
            }
        }
    }

    Mat contourImg(rTopFinal.rows, rTopFinal.cols, CV_8UC1, Scalar(0,0,0));
    Point p;
    for (int i = 0; i < possibleKeyPoints.size(); ++i) {
        p = possibleKeyPoints.at(i).keyPoint;
        contourImg.at<uchar>(p.y, p.x) = 255;
    }
    Mat _img;
    double otsu_thresh_val = cv::threshold(
                contourImg, _img, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU
                );

    Canny(contourImg, contourImg, otsu_thresh_val*0.5, otsu_thresh_val);

    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours( contourImg, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
    for( int i = 0; i< contours.size(); i++ ){
        drawContours( contourImg, contours, i, Scalar(255,255,255), 1, 8, hierarchy, 1, Point() );
    }


    keyPoint2.y = 1000;
    for (int i = 0; i < rTopFinal.rows; ++i) {
        for (int j = rTopFinal.cols/2; j > 0; --j) {
            if(contourImg.at<uchar>(i,j) == 255){
                if(keyPoint2.y >= i){
                    keyPoint2.y = i;
                    keyPoint2.x = j;
                }
            }
        }
    }


    keyPoint4.y = 1000;
    for (int i = 0; i < rTopFinal.rows; ++i) {
        for (int j = rTopFinal.cols/2; j < rTopFinal.cols; ++j) {
            if(contourImg.at<uchar>(i,j) == 255){
                if(keyPoint4.y >= i){
                    keyPoint4.y = i;
                    keyPoint4.x = j;
                }
            }
        }
    }


    keyPoint3.y = 0;
    int kp2kp3Width = keyPoint4.x  - keyPoint2.x;
    kp2kp3Width = kp2kp3Width/2;

    for (int i = keyPoint2.x; i < keyPoint4.x; ++i) {
        for (int j = 0; j < keyPoint1.y-10; ++j) {
            if(contourImg.at<uchar>(j,i) == 255){
                if(keyPoint3.y <= j &&  i <= (keyPoint2.x + kp2kp3Width) ){
                    keyPoint3.y = j;
                    keyPoint3.x = i;
                }
            }
        }
    }
}

void KeyPointsDeliverer::extractLowerLipKeyPoint(int thresholdDifferenceToAvg, int totalLineCheck)
{
    QList<PossibleKeyPoint> possibleKeyPoints;
    PossibleKeyPoint possibleKeyPoint;

    for (int i = 0; i < rLowFinal.cols; ++i) {
        for (int j = rLowFinal.rows/2; j < rLowFinal.rows-totalLineCheck/2; ++j) {

            int currentDiffToAvg = 0;

            for (int k = 1; k < totalLineCheck/2 + 1; ++k) {
                currentDiffToAvg += rLowFinal.at<uchar>(j-k,i) + rLowFinal.at<uchar>(j+k,i);

            }
            currentDiffToAvg = currentDiffToAvg / totalLineCheck;

            if(currentDiffToAvg > 0){
                currentDiffToAvg = 100 - (rLowFinal.at<uchar>(j,i) * 100 / currentDiffToAvg);
            }

            if(currentDiffToAvg > thresholdDifferenceToAvg){
                possibleKeyPoint.differenceToAvg = currentDiffToAvg;
                possibleKeyPoint.keyPoint.x  = i;
                possibleKeyPoint.keyPoint.y  = j;
                possibleKeyPoints.append(possibleKeyPoint);
            }
        }
    }

    Mat contourImg(rLowFinal.rows, rLowFinal.cols, CV_8UC1, Scalar(0,0,0));
    Point p;
    for (int i = 0; i < possibleKeyPoints.size(); ++i) {
        p = possibleKeyPoints.at(i).keyPoint;
        contourImg.at<uchar>(p.y, p.x) = 255;
    }
    Mat _img;
    double otsu_thresh_val = cv::threshold(
                contourImg, _img, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU
                );
    Canny(contourImg, contourImg, otsu_thresh_val*0.5, otsu_thresh_val);

    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours( contourImg, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
    for( int i = 0; i< contours.size(); i++ ){
        drawContours( contourImg, contours, i, Scalar(255,255,255), 1, 8, hierarchy, 1, Point() );
    }


    keyPoint6.y = 0;
    int kp2kp3Width = keyPoint4.x - keyPoint2.x;
    kp2kp3Width = kp2kp3Width/2;

    for (int i = keyPoint2.x; i < keyPoint4.x; ++i) {
        for (int j = rLowFinal.rows-(rLowFinal.rows*0.2); j > keyPoint1.y; --j) {
            if(contourImg.at<uchar>(j, i) == 255){
                if(keyPoint6.y <= j && i <= (keyPoint2.x + kp2kp3Width)){
                    keyPoint6.y = j;
                    keyPoint6.x = i;
                    break;
                }
            }
        }
    }
}

Mat &KeyPointsDeliverer::getRTop()
{
    return this->rTopFinal;
}

Mat &KeyPointsDeliverer::getRMid()
{
    return this->rMidFinal;
}

Mat &KeyPointsDeliverer::getRLow()
{
    return this->rLowFinal;
}

Point &KeyPointsDeliverer::getKeyPoint1()
{
    return this->keyPoint1;
}

Point &KeyPointsDeliverer::getKeyPoint2()
{
    return this->keyPoint2;
}

Point &KeyPointsDeliverer::getKeyPoint3()
{
    return this->keyPoint3;
}

Point &KeyPointsDeliverer::getKeyPoint4()
{
    return this->keyPoint4;
}

Point &KeyPointsDeliverer::getKeyPoint5()
{
    return this->keyPoint5;
}

Point &KeyPointsDeliverer::getKeyPoint6()
{
    return this->keyPoint6;
}
