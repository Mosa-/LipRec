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

    QList<QList<double> > kMedoidsClustering(DistanceFunction df);
    QList<QList<double> > ownClustering(DistanceFunction df, int noExceptTrajectories);

    void addTrajectory(QList<double> trajectory);
    void setK(int k);
    void clearTrajectoriesSet();

private:

    Dtw dtw;

    int k;
    QList<QList<double> > trajectories;
    QMap<int, QList<int> > assignCluster;

    int randInt(int low, int high);
    double calcWarpingCost(DistanceFunction df);


};

#endif /* CLUSTERING_H_ */
