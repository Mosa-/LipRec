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

void TrajectoriesDataManager::connectToDatabase(string host)
{
    try{
        mongoDB.connect(host);
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

    mongoDB.insert("liprec.persons", bsonBuilder.obj());
}

QList<QList<double> > TrajectoriesDataManager::getTrajectory(QString command, QString feature)
{
    QList<QList<double> > trajectories;
    BSONObj p ;

    auto_ptr<DBClientCursor> cursor =
            mongoDB.query("liprec.persons", QUERY("command" << command.toStdString() << "feature" << feature.toStdString()));

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
