#include "rqt_liprec/ZernikeMoments.h"




ZernikeMoments::ZernikeMoments(int imgSize) : imgSize(imgSize)
{

}

double ZernikeMoments::calcRadialPolynomial(int n, int l, double radiusDist) {
    double radialPolynomialValue = 0.0;

    double denominator1;
    double denominator2;

    double numerator;

    for (int k = 0; k < (n-abs(l))/2; ++k) {
        numerator = fak(n-k);
        denominator1 = fak(((n+abs(l))/2)-k);
        denominator2 = fak(((n-abs(l))/2)-k);

        radialPolynomialValue+= pow(-1, k) * (fak(n-k) / (fak(k) * denominator1 * denominator2)) * pow(radiusDist, n-2*k);
    }

    return radialPolynomialValue;
}

double ZernikeMoments::calcZernikePolynomial(double radiusDist, int angle) {
    double zernikePolynomial = 0.0;


}

int ZernikeMoments::fak(int n)
{
    if(n==1){
        return 1;
    }else{
        return n * fak(n-1);
    }
}

double ZernikeMoments::calcXCircularCoordinate(int imgSize, int xSquareCoordinate) {
    return (sqrt(2)/imgSize-1)*xSquareCoordinate + (-1 / sqrt(2));
}

double ZernikeMoments::calcYCircularCoordinate(int imgSize, int ySquareCoordinate) {
    return (sqrt(2)/imgSize-1)*ySquareCoordinate + (-1 / sqrt(2));
}

double ZernikeMoments::calcRadiusDist(int imgSize, int xSquareCoordinate, int ySquareCoordinate) {
    double x = calcXCircularCoordinate(imgSize, xSquareCoordinate);
    double y = calcYCircularCoordinate(imgSize, ySquareCoordinate);

    return sqrt(pow(x,2) + pow(y,2));
}

double ZernikeMoments::calcAngleCircular(int imgSize, int xSquareCoordinate, int ySquareCoordinate) {
    double x = calcXCircularCoordinate(imgSize, xSquareCoordinate);
    double y = calcYCircularCoordinate(imgSize, ySquareCoordinate);

    return atan(y/x);
}
