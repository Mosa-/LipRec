#ifndef CLUSTERING_H_
#define CLUSTERING_H_

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <QtCore>
#include <QtGlobal>

#include "Dtw.h"

using namespace cv;
using namespace std;

class Clustering {
public:

    Clustering();
    Clustering(int k);
    virtual ~Clustering();

    QList<QList<double> > kMedoidsClustering(DistanceFunction df, DtwStepPattern stepPattern, bool slopeWeights, bool windowSizeActive, int windowSize, bool windowAdapted);
    QList<QList<double> > simpleClustering(DistanceFunction df, int noExceptTrajectories);

    void addTrajectories(QList<QList<double> > trajectories);
    void addTrajectory(QList<double> trajectory);
    void setK(int k);
    void clearTrajectoriesSet();


private:

    Dtw dtw;

    int k;
    QList<QList<double> > trajectories;
    QMap<int, QList<int> > assignCluster;

    int randInt(int low, int high);
    double calcWarpingCost(DistanceFunction df, bool windowSizeActive, int windowSize, bool windowAdapted);


    void printTrajectory(QList<double> trajectory);


};

#endif /* CLUSTERING_H_ */
