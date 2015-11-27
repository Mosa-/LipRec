#include "rqt_liprec/Clustering.h"

Clustering::Clustering()
{

}

Clustering::Clustering(int k){
    this->k = k;
}

Clustering::~Clustering(){

}

QList<QList<double> > Clustering::kMedoidsClustering(DistanceFunction df)
{
    QList<int> kIndices;

    int kRandom = 0;
    for (int i = 0; i < this->k; ++i) {
        kRandom = randInt(0, this->trajectories.size()-1);

        if(kIndices.empty()){
            kIndices.append(kRandom);
            assignCluster[kRandom].append(kRandom);
        }else{
            while(kIndices.contains(kRandom)){
                kRandom = randInt(0, this->trajectories.size()-1);
            }
            kIndices.append(kRandom);
            assignCluster[kRandom].append(kRandom);

        }
    }

    bool changes = true;

    while(changes){
        // assignEachTrajectoryToCluster
        int kIndex = 0;
        int lowCostTrajectoryIndex = -1;
        double previousWarpingCost = 1000;
        double tmpWarpingCost = 0;
        for (int i = 0; i < trajectories.size(); ++i) {
            if(!kIndices.contains(i)){
                for (int j = 0; j < kIndices.size(); ++j) {
                    kIndex = kIndices.at(j);
                    dtw.seed(trajectories.at(kIndex), trajectories.at(i));

                    tmpWarpingCost = calcWarpingCost(df);
                    if(tmpWarpingCost < previousWarpingCost){
                        previousWarpingCost = tmpWarpingCost;
                        lowCostTrajectoryIndex = kIndex;
                    }
                }
                assignCluster[lowCostTrajectoryIndex].append(i);
                lowCostTrajectoryIndex = -1;
                previousWarpingCost = 1000;
            }
        }


        // adjustClusterMedoidsCenter
        QMap<int, QList<int> > newAssignCluster;
        QList<int> cluster;
        foreach (int k, assignCluster.keys()) {
            cluster = assignCluster[k];

            double average = 0.0;
            double previousAverage = 1000.0;
            int newK = -1;
            for (int i = 0; i < cluster.size(); ++i) {
                average = 0.0;
                for (int j = 0; j < cluster.size(); ++j) {
                    if(i != j){
                        dtw.seed(trajectories.at(cluster.at(i)), trajectories.at(cluster.at(j)));
                        average += calcWarpingCost(df);
                    }
                }

                average = average/ (cluster.size()-1);
                if(average < previousAverage){
                    previousAverage = average;
                    newK = cluster.at(i);
                }
            }
            newAssignCluster[newK].append(newK);
        }

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

    return kMedoids;
}

QList<QList<double> > Clustering::ownClustering(DistanceFunction df, int noExceptTrajectories)
{
    QList<QList<double> > kClusterTrajectories;

    QMap<int, double> trajectoryDistances;
    double average = 0.0;
    double warpingCost = 0.0;

    QList<QList<double> > currentTrajectories = trajectories;
    QList<QList<double> > remainTrajectories;
    int lowTrajectoriesCostCounter = 0;
    QMap<int, int> trajectoryLowCost;

    while(currentTrajectories.size() > noExceptTrajectories) {
        for (int i = 0; i < currentTrajectories.size(); ++i) {
            lowTrajectoriesCostCounter = 0;
            average = 0.0;
            trajectoryDistances.clear();
            remainTrajectories.clear();

            for (int j = 0; j < currentTrajectories.size(); ++j) {
                if(i != j){
                    dtw.seed(currentTrajectories.at(i), currentTrajectories.at(j));
                    warpingCost = this->calcWarpingCost(df);
                    average += warpingCost;

                    trajectoryDistances[j] = warpingCost;
                }
            }
            average = average/(currentTrajectories.size()-1);

            foreach (int k, trajectoryDistances.keys()) {
                if(trajectoryDistances[k] <= average){
                    lowTrajectoriesCostCounter++;
                }
            }

            trajectoryLowCost[i] = lowTrajectoriesCostCounter;
        }

        int highCounterIndex = 0;
        int tempCnt = 0;
        foreach (int trajectoryIndex, trajectoryLowCost.keys()) {
            if(trajectoryLowCost[trajectoryIndex] > tempCnt){
                tempCnt = trajectoryLowCost[trajectoryIndex];
                highCounterIndex = trajectoryIndex;
            }
        }

        kClusterTrajectories.append(trajectories.at(highCounterIndex));

        // get remain trajectories
        average = 0.0;
        trajectoryDistances.clear();

        for (int i = 0; i < currentTrajectories.size(); ++i) {
            if(i != highCounterIndex){
                dtw.seed(currentTrajectories.at(highCounterIndex), currentTrajectories.at(i));
                warpingCost = this->calcWarpingCost(df);
                average += warpingCost;
                trajectoryDistances[i] = warpingCost;
            }
        }
        average = average/(currentTrajectories.size()-1);
        foreach (int k, trajectoryDistances.keys()) {
            if(trajectoryDistances[k] > average){
                remainTrajectories.append(currentTrajectories.at(k));
            }
        }

        currentTrajectories = remainTrajectories;
    }

    return kClusterTrajectories;
}

double Clustering::calcWarpingCost(DistanceFunction df)
{
    dtw.calculateDistanceMatrix(df);
    dtw.calculateDtwDistanceMatrix();
    dtw.calculateGreedyWarpingPath();

    return dtw.getWarpingPathCost();
}

void Clustering::addTrajectory(QList<double> trajectory)
{
    this->trajectories.append(trajectory);
}

void Clustering::setK(int k)
{
    this->k = k;
}

void Clustering::clearTrajectoriesSet()
{
    this->trajectories.clear();
}

int Clustering::randInt(int low, int high)
{
    return low + (rand() % (int)(high - low + 1));
}


