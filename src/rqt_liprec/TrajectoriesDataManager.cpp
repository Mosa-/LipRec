#include "rqt_liprec/TrajectoriesDataManager.h"

TrajectoriesDataManager::TrajectoriesDataManager(){

}

TrajectoriesDataManager::TrajectoriesDataManager(string host){
    try{
        mongoDB.connect(host);
    } catch(DBException &e) {
        ROS_INFO("caught %s",e.what());
    }
}

TrajectoriesDataManager::~TrajectoriesDataManager(){

}

void TrajectoriesDataManager::connectToDatabase(string host, QString database, QString collection)
{
    try{
        mongoDB.connect(host);
        this->database = database;
        this->collection = collection;
    } catch(DBException &e) {
        ROS_INFO("caught %s",e.what());
    }
}

void TrajectoriesDataManager::insertTrajectory(QList<double> trajectory, QString command, QString feature)
{
    BSONObjBuilder bsonBuilder;
    bsonBuilder.append("command", command.toStdString());
    bsonBuilder.append("feature", feature.toStdString());

    BSONArrayBuilder bab;
    BSONObj bo;

    for (int i = 0; i < trajectory.size(); ++i) {
        bo = BSON("id" << i << "value" << trajectory.at(i));
        bab.append(bo);
    }

    bsonBuilder.appendArray("values", bab.arr());

    mongoDB.insert(QString(database+"."+collection).toStdString(), bsonBuilder.obj());
}

QList<QList<double> > TrajectoriesDataManager::getTrajectory(QString command, QString feature)
{
    QList<QList<double> > trajectories;
    BSONObj p;

    auto_ptr<DBClientCursor> cursor =
            mongoDB.query(QString(database+"."+collection).toStdString(), QUERY("command" << command.toStdString() << "feature" << feature.toStdString()));

    QList<double> trajectory;
    while (cursor->more()) {
        p = cursor->next();
        trajectory.clear();

        //ROS_INFO("%s\n", p.toString().c_str());

        vector<BSONElement> array = p["values"].Array();
        for (vector<BSONElement>::iterator ar = array.begin(); ar != array.end(); ++ar){
            BSONObj bo = ar->embeddedObject();
            trajectory.append(bo.getField("value").Double());
            //ROS_INFO("%s %s",bo.getField("id").toString().c_str(), bo.getField("value").toString().c_str());
        }
        trajectories.append(trajectory);
    }

    return trajectories;
}


void TrajectoriesDataManager::insertClusterTrajectories(QList<QList<double> > trajectories, QString command, QString feature, QString clusterMethod)
{
    BSONObjBuilder bsonBuilder;
    bsonBuilder.append("command", command.toStdString());
    bsonBuilder.append("feature", feature.toStdString());
    bsonBuilder.append("clusterMethod", clusterMethod.toStdString());


    BSONArrayBuilder bab2;

    for (int i = 0; i < trajectories.size(); ++i) {
        BSONArrayBuilder bab;
        BSONObj bo;

        for (int k = 0; k < trajectories.at(i).size(); ++k) {
            bo = BSON("id" << k << "value" << trajectories.at(i).at(k));
            bab.append(bo);
        }
        bab2.append(bab.arr());
    }
    bsonBuilder.appendArray("values", bab2.arr());

    mongoDB.insert(QString(database+"."+collectionCluster).toStdString(), bsonBuilder.obj());
}

void TrajectoriesDataManager::updateClusterTrajectories(QList<QList<double> > trajectories, QString command, QString feature, QString clusterMethod)
{
    BSONObjBuilder bsonBuilder;

    BSONArrayBuilder bab2;

    for (int i = 0; i < trajectories.size(); ++i) {
        BSONArrayBuilder bab;
        BSONObj bo;

        for (int k = 0; k < trajectories.at(i).size(); ++k) {
            bo = BSON("id" << k << "value" << trajectories.at(i).at(k));
            bab.append(bo);
        }
        bab2.append(bab.arr());
    }
    bsonBuilder.appendArray("values", bab2.arr());
    BSONObj bsonData = BSON("$set" << bsonBuilder.obj());

    mongoDB.update(QString(database+"."+collectionCluster).toStdString(),
                   QUERY("command" << command.toStdString() << "feature" << feature.toStdString() << "clusterMethod" << clusterMethod.toStdString()), bsonData);

}

QList<QList<double> > TrajectoriesDataManager::getClusterTrajectories(QString command, QString feature, QString clusterMethod){
    QList<QList<double> > trajectories;
    BSONObj p;

    auto_ptr<DBClientCursor> cursor =
            mongoDB.query(QString(database+"."+collectionCluster).toStdString(),
                          QUERY("command" << command.toStdString() << "feature" << feature.toStdString() << "clusterMethod" << clusterMethod.toStdString()));

    QList<double> trajectory;
    while (cursor->more()) {
        p = cursor->next();

        vector<BSONElement> array = p["values"].Array();
        for (vector<BSONElement>::iterator ar = array.begin(); ar != array.end(); ++ar){
            vector<BSONElement> array2 = ar->Array();
            trajectory.clear();
            for (vector<BSONElement>::iterator ar2 = array2.begin(); ar2 != array2.end(); ++ar2) {
                BSONObj bo = ar2->embeddedObject();
                trajectory.append(bo.getField("value").Double());
            }
            trajectories.append(trajectory);
        }
    }

    return trajectories;

}

QMap<QString, int> TrajectoriesDataManager::getAllCommandsWithCount()
{
    QMap<QString, int> commandsWithCount;

    BSONObj p;

    auto_ptr<DBClientCursor> cursor =
            mongoDB.query((database+"."+collection).toStdString(), "");

    while (cursor->more()) {
        p = cursor->next();
        QString commandStr = QString::fromStdString(p.getField("command").str());
        commandsWithCount[commandStr]++;
    }

    return commandsWithCount;
}

QStringList TrajectoriesDataManager::getFeatures(QString command)
{
    QStringList features;
    BSONObj p;

    auto_ptr<DBClientCursor> cursor =
            mongoDB.query(QString(database+"."+collection).toStdString(), QUERY("command" << command.toStdString()));

    while (cursor->more()) {
        p = cursor->next();

        QString featureStr = QString::fromStdString(p.getField("feature").str());
        if(!features.contains(featureStr)){
            features.append(featureStr);
        }
    }
    return features;
}

void TrajectoriesDataManager::setCollection(QString collection)
{
    this->collection = collection;
}

void TrajectoriesDataManager::setCollectionCluster(QString collection)
{
    this->collectionCluster = collection;
}
