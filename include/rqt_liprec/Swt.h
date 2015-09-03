#ifndef SWT_H_
#define SWT_H_

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

class Swt {
public:
    enum MotherWavelet
    {
      /* Haar Wavelet */
      Haar,

      /* Dmeyer */

      Dmey,

      /* Symmlets */
      Symm
    };

    enum ConvolutionType {   /* Return the full convolution, including border */
      CONVOLUTION_FULL,
      /* Return only the part that corresponds to the original image */
      CONVOLUTION_SAME,
      /* Return only the submatrix containing elements that were not influenced by the border */
      CONVOLUTION_VALID
    };

    Swt();
    virtual ~Swt();

    void applySwt(const Mat &src_Original, Mat &ca, Mat &ch, Mat &cd, Mat &cv, int Level, MotherWavelet Type);
    void extendPeriod(const Mat &b, Mat &c, int level);
    void conv2(const Mat &img, const Mat& kernel, ConvolutionType type, Mat& dest, int flipcode) ;
    void filterBank(Mat &kernel_High, Mat &kernel_Low, MotherWavelet type);
    void keepLoc(Mat &src, int extension, int originalSize);
    void dyadicUpsample(Mat &kernel);


};

#endif /* SWT_H_ */
