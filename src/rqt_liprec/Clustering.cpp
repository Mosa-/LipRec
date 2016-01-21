#include "rqt_liprec/Clustering.h"

Clustering::Clustering()
{

}

Clustering::Clustering(int k){
    this->k = k;
}

Clustering::~Clustering(){

}

QList<QList<double> > Clustering::kMedoidsClustering(DistanceFunction df, DtwStepPattern stepPattern, bool slopeWeights, bool windowSizeActive, int windowSize, bool windowAdapted)
{
    //ROS_INFO("size in clustering %d k: %d", this->trajectories.size(), this->k);
    QList<int> kIndices;

    int kRandom = 0;
    for (int i = 0; i < this->k; ++i) {
        kRandom = randInt(0, this->trajectories.size()-1);


        if(kIndices.empty()){
            kIndices.append(kRandom);
            assignCluster[kRandom].append(kRandom);

            //ROS_INFO("kRandom %d", kRandom);

        }else{
            while(kIndices.contains(kRandom)){
                kRandom = randInt(0, this->trajectories.size()-1);
            }
            kIndices.append(kRandom);
            assignCluster[kRandom].append(kRandom);
            //ROS_INFO("kRandom %d",  kRandom);
        }
    }

    bool changes = true;
    int cntTempForDebugBreak = 0;

    while(changes){
        cntTempForDebugBreak++;
        if(cntTempForDebugBreak > 15){
            ROS_INFO("BREAK BRABS");
            break;
        }
        //ROS_INFO(">>>BEGIN assignEachTrajectoryToCluster");
        // assignEachTrajectoryToCluster
        int kIndex = 0;
        int lowCostTrajectoryIndex = -1;
        double previousWarpingCost = INT_MAX;
        double tmpWarpingCost = 0;
        for (int i = 0; i < trajectories.size(); ++i) {
            if(!kIndices.contains(i)){
                for (int j = 0; j < kIndices.size(); ++j) {
                    kIndex = kIndices.at(j);
                    dtw.seed(trajectories.at(kIndex), trajectories.at(i), stepPattern, slopeWeights);


                    tmpWarpingCost = calcWarpingCost(df, windowSizeActive, windowSize, windowAdapted);
                    if(tmpWarpingCost < previousWarpingCost){
                        previousWarpingCost = tmpWarpingCost;
                        lowCostTrajectoryIndex = kIndex;
                    }
                }
                assignCluster[lowCostTrajectoryIndex].append(i);
                //ROS_INFO("trajektory: %d assign to cluster %d", i, lowCostTrajectoryIndex);
                lowCostTrajectoryIndex = -1;
                previousWarpingCost = INT_MAX;
            }
        }

        //ROS_INFO(">>>END assignEachTrajectoryToCluster");


        //ROS_INFO(">>>BEGIN adjustClusterMedoidsCenter");
        // adjustClusterMedoidsCenter
        QMap<int, QList<int> > newAssignCluster;
        QList<int> cluster;
        kIndices.clear();
        foreach (int k, assignCluster.keys()) {
            cluster = assignCluster[k];

            double average = 0.0;
            double previousAverage = INT_MAX;
            int newK = -1;

            for (int i = 0; i < cluster.size(); ++i) {
                average = 0.0;
                for (int j = 0; j < cluster.size(); ++j) {
                    if(i != j){
                        dtw.seed(trajectories.at(cluster.at(i)), trajectories.at(cluster.at(j)), stepPattern, slopeWeights);
                        //this->printTrajectory(trajectories.at(cluster.at(j)));
                        average += calcWarpingCost(df, windowSizeActive, windowSize, windowAdapted);
                        //ROS_INFO("currentAvg: %f", average);
                    }
                }

                average = average/ (cluster.size());
                if(average < previousAverage){
                    if(cluster.size() > 1){
                        //ROS_INFO("average %f newK %d", average, cluster.at(i));

                        previousAverage = average;
                        newK = cluster.at(i);
                    }
                }
            }
            if(newK >= 0){
                //ROS_INFO("newAssignCluster k: %d", newK);
                newAssignCluster[newK].append(newK);
                kIndices.append(newK);
            }
        }
        //ROS_INFO(">>>END adjustClusterMedoidsCenter");


        changes = false;
        foreach (int k, assignCluster.keys()) {
            if(!newAssignCluster.keys().contains(k)){
                changes = true;
            }
        }
        assignCluster.clear();
        assignCluster = newAssignCluster;


    }

    QList<QList<double> > kMedoids;
    foreach (int k, assignCluster.keys()) {
        kMedoids.append(trajectories.at(k));
    }

    this->trajectories.clear();

    return kMedoids;
}

QList<QList<double> > Clustering::simpleClustering(DistanceFunction df, int noExceptTrajectories)
{
    QList<QList<double> > kClusterTrajectories;

//    QMap<int, double> trajectoryDistances;
//    double average = 0.0;
//    double warpingCost = 0.0;

//    QList<QList<double> > currentTrajectories = trajectories;
//    QList<QList<double> > remainTrajectories;
//    int lowTrajectoriesCostCounter = 0;
//    QMap<int, int> trajectoryLowCost;

//    while(currentTrajectories.size() > noExceptTrajectories) {
//        for (int i = 0; i < currentTrajectories.size(); ++i) {
//            lowTrajectoriesCostCounter = 0;
//            average = 0.0;
//            trajectoryDistances.clear();
//            remainTrajectories.clear();

//            for (int j = 0; j < currentTrajectories.size(); ++j) {
//                if(i != j){
//                    dtw.seed(currentTrajectories.at(i), currentTrajectories.at(j), stepPattern);
//                    warpingCost = this->calcWarpingCost(df);
//                    average += warpingCost;

//                    trajectoryDistances[j] = warpingCost;
//                }
//            }
//            average = average/(currentTrajectories.size());

//            foreach (int k, trajectoryDistances.keys()) {
//                if(trajectoryDistances[k] <= average){
//                    lowTrajectoriesCostCounter++;
//                }
//            }

//            trajectoryLowCost[i] = lowTrajectoriesCostCounter;
//        }

//        int highCounterIndex = 0;
//        int tempCnt = 0;
//        foreach (int trajectoryIndex, trajectoryLowCost.keys()) {
//            if(trajectoryLowCost[trajectoryIndex] > tempCnt){
//                tempCnt = trajectoryLowCost[trajectoryIndex];
//                highCounterIndex = trajectoryIndex;
//            }
//        }

//        kClusterTrajectories.append(trajectories.at(highCounterIndex));

//        // get remain trajectories
//        average = 0.0;
//        trajectoryDistances.clear();

//        for (int i = 0; i < currentTrajectories.size(); ++i) {
//            if(i != highCounterIndex){
//                dtw.seed(currentTrajectories.at(highCounterIndex), currentTrajectories.at(i), stepPattern);
//                warpingCost = this->calcWarpingCost(df);
//                average += warpingCost;
//                trajectoryDistances[i] = warpingCost;
//            }
//        }
//        average = average/(currentTrajectories.size());
//        foreach (int k, trajectoryDistances.keys()) {
//            if(trajectoryDistances[k] > average){
//                remainTrajectories.append(currentTrajectories.at(k));
//            }
//        }

//        currentTrajectories = remainTrajectories;
//    }

    return kClusterTrajectories;
}

void Clustering::addTrajectories(QList<QList<double> > trajectories)
{
    for (int i = 0; i < trajectories.size(); ++i) {
        this->addTrajectory(trajectories.at(i));
    }
}

void Clustering::addTrajectory(QList<double> trajectory)
{
    this->trajectories.append(trajectory);
}


double Clustering::calcWarpingCost(DistanceFunction df, bool windowSizeActive, int windowSize, bool windowAdapted)
{
    double warpingCost = 0.0;

    dtw.calculateDistanceMatrix(df);
    dtw.calculateDtwDistanceMatrix(windowSizeActive, windowSize, windowAdapted);
    dtw.calculateGreedyWarpingPath();

    warpingCost = dtw.getWarpingPathCost();

    return warpingCost;
}

void Clustering::printTrajectory(QList<double> trajectory)
{
    ROS_INFO(">>>Start printTrajectory");
    for (int i = 0; i < trajectory.size(); ++i) {
        ROS_INFO("%f", trajectory.at(i));
    }
    ROS_INFO(">>>End printTrajectory");
}

void Clustering::setK(int k)
{
    this->k = k;
}

void Clustering::clearTrajectoriesSet()
{
    this->trajectories.clear();
  this->assignCluster.clear();
}

int Clustering::randInt(int low, int high)
{
    return low + (rand() % (int)(high - low + 1));
}


