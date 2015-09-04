#include "rqt_liprec/Swt.h"


Swt::Swt(){

}

Swt::~Swt(){

}

void Swt::applySwt(const Mat &src_Original, Mat &ca, Mat &ch, Mat &cd, Mat &cv, int Level, MotherWavelet Type){
    if (Type == Haar) {

        /* Decalre and Intialize helper Matrices */

        Mat kernel_High = Mat::zeros(1, 2, CV_8U);
        Mat kernel_Low = Mat::zeros(1, 2, CV_8U);
        Mat swa, swh, swv, swd;
        Mat src = src_Original;


        /* Initiliaze Filter Banks for Haar Transform */


        this->filterBank(kernel_High, kernel_Low, Haar);


        /* The main loop for calculating Stationary 2D Wavelet Transform */


        for (int i = 0; i < Level; i++) {

            /* A temporary src Matrix for calculations */

            Mat extended_src = Mat::zeros(1,1, CV_8U);


            /* Extend source Matrix to deal with edge related issues */

            this->extendPeriod(src, extended_src, i);



            /* Helper Matrices */

            Mat y = Mat::zeros(1,1, CV_8U);
            Mat y1 = Mat::zeros(1,1, CV_8U);
            Mat z = Mat::zeros(1,1, CV_8U);

            /* Calculating Approximation coeffcients */

            this->conv2(extended_src, kernel_Low, CONVOLUTION_FULL, y, 1);

            transpose(y , y1);

            this->conv2(y1, kernel_Low, CONVOLUTION_FULL, z, 1);

            transpose(z , swa);

            this->keepLoc(swa, kernel_Low.cols + 1, src.cols);


            /* Calculating Horizontal coeffcients */

            this->conv2(y1, kernel_High, CONVOLUTION_FULL, z, 1);

            transpose(z , swh);

            this->keepLoc(swh, kernel_Low.cols + 1, src.cols);


            /* Calculating Vertical coeffcients */


            this->conv2(extended_src, kernel_High, CONVOLUTION_FULL, y, 1);

            transpose(y , y1);

            this->conv2(y1, kernel_Low, CONVOLUTION_FULL, z, 0);

            transpose(z , swv);

            this->keepLoc(swv, kernel_Low.cols + 1, src.cols);


            /* Calculating Diagonal coeffcients */


            this->conv2(y1, kernel_High, CONVOLUTION_FULL, z, 0);

            transpose(z , swd);

            this->keepLoc(swd, kernel_Low.cols + 1, src.cols);



            /* Upsamle Low and High Pass Filters */

            this->dyadicUpsample(kernel_High);
            this->dyadicUpsample(kernel_Low);

            /* Create a vector of Matrices to store two channels */

            vector <Mat> temp;

            /* Split Hor, Ver, Diag and App into respective channels, copy the latest */
            /* calculated co-efficients into the channels, and merge them */

            split(ca, temp); swa.copyTo(temp[i]); merge(temp, ca);    /* Approximation coeffcients */

            split(ch, temp); swh.copyTo(temp[i]); merge(temp, ch);	  /* Horizontal coefficients */

            split(cv, temp); swv.copyTo(temp[i]); merge(temp, cv);    /* Vertical coeffcients */

            split(cd, temp); swd.copyTo(temp[i]); merge(temp, cd);    /* Diagonal coeffcients */


            /* Copy the Approximation co-efficients into this stage's source Mat */
            /* for next stage decomposition */

            swa.copyTo(src);

        }
    }
}

void Swt::conv2(const Mat& img, const Mat& kernel, ConvolutionType type, Mat& dest, int flipcode) {
  Mat source = img;
  if(CONVOLUTION_FULL == type)
  {
    source = Mat();
    const int additionalRows = kernel.rows-1, additionalCols = kernel.cols-1;
    copyMakeBorder(img, source, (additionalRows+1)/2, additionalRows/2, (additionalCols+1)/2, additionalCols/2, BORDER_CONSTANT, Scalar(0));
  }

  Point anchor(kernel.cols - kernel.cols/2 - 1, kernel.rows - kernel.rows/2 - 1);
  int borderMode = BORDER_CONSTANT;
  Mat kernel_temp;
  flip(kernel,kernel_temp,flipcode);
  filter2D(source, dest, img.depth(), kernel_temp, anchor, 0, borderMode);

  if(CONVOLUTION_VALID == type) {
    dest = dest.colRange((kernel.cols-1)/2, dest.cols - kernel.cols/2)
               .rowRange((kernel.rows-1)/2, dest.rows - kernel.rows/2);
  }
}

void Swt::extendPeriod(const Mat& b, Mat& c, int level)
{
    /* Check for correct values of levels */

    if (level >= 0 && level < 6)
    {

        int Level_Mat[3] = {2,4,8};               /* This tells how much the source Matrix must be expanded at edges */
        int inc_Per = Level_Mat[level];			  /* Calculate the expansion at that particular level */
        c = Mat::zeros(b.rows + inc_Per , b.cols + inc_Per , CV_8U);	 /* Create Matrix with expanded edges */


        /* Copy original Matrix into new Matrix */

        b.rowRange(0,b.rows).copyTo(c.rowRange((int)(inc_Per/2), b.rows + inc_Per/2).colRange((int)(inc_Per/2), b.cols + inc_Per/2));

        /* Copy the columns from original matrix to new Matrix for edge periodization */


        b.rowRange(0, b.rows).colRange(b.cols - inc_Per/2, b.cols).copyTo(c.rowRange(inc_Per/2, b.rows + inc_Per/2).colRange(0, inc_Per/2));

        b.rowRange(0, b.rows).colRange(0, inc_Per/2).copyTo(c.rowRange(inc_Per/2, b.rows + inc_Per/2).colRange(b.cols + inc_Per/2, c.cols));


        /* Copy the rows from original matrix to new Matrix for edge periodization */

        c.rowRange(b.rows , b.rows + inc_Per/2).copyTo(c.rowRange(0, inc_Per/2));

        c.rowRange(inc_Per/2, inc_Per).copyTo(c.rowRange(b.rows + inc_Per/2, c.rows));



    }
}

void Swt::filterBank(Mat& kernel_High, Mat& kernel_Low, MotherWavelet type)
{
    if(type == Haar)
    {
        /* Initiliaze Haar's Filter Bank */
        Mat kernel_High1 = Mat::zeros(1, 2, CV_32F);
        Mat kernel_Low1 = Mat::zeros(1, 2, CV_32F);

        kernel_High1.at<float>(0,0) = (float) (-0.7071);
        kernel_High1.at<float>(0,1) = (float) (0.7071);
        kernel_Low1.at<float>(0,0) = (float) (0.7071);
        kernel_Low1.at<float>(0,1) = (float) (0.7071);

        kernel_High1.convertTo(kernel_High, CV_8U);
        kernel_Low1.convertTo(kernel_Low, CV_8U);
    }
}


void Swt::keepLoc(Mat& src, int extension, int originalSize)
{
          /* Get rid of the Edges, and get an image out which is of original dimensions */
          /* Note: This currently works for only Square matrices. Update needed. */

          int end = extension + originalSize - 1;
          Mat dst = Mat::zeros(originalSize, originalSize, CV_8U);
          src.rowRange(extension - 1, end).colRange(extension - 1, end).copyTo(dst);
          src = dst;
}

void Swt::dyadicUpsample(Mat& kernel)
{
    /* Create a new Matrix with two rows */

    Mat temp = Mat::zeros(kernel.rows + 1, kernel.cols, CV_8U);

    /* Copy Kernel into first row */

    kernel.row(0).colRange(0, kernel.cols).copyTo(temp.row(0));

    /* Now traverse the Matrix in Zig-Zag manner, and insert the column  */
    /* values into a new column major Matrix */
    /* Note: There must be a faster way to do this in OpenCV. I will be updating it. */

    Mat Ret = Mat::zeros(kernel.cols * 2, 1, CV_8U);
    int index = 0;
    for (int i = 0; i < (kernel.cols * 2)/2 ; i++)
    {
        temp.rowRange(0, temp.rows).col(i).copyTo(Ret.col(0).rowRange(index, index + 2));
        index = index + 2;
    }

    /* Take transpose converting column major matrix to row major */

    transpose(Ret, kernel);
}
