#ifndef TRAJECTORIESDATAMANAGER_H_
#define TRAJECTORIESDATAMANAGER_H_

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <QtCore>
#include <QtGlobal>
#include <mongo/client/dbclient.h>



using namespace cv;
using namespace std;
using namespace mongo;

class TrajectoriesDataManager {
public:

    TrajectoriesDataManager();
    TrajectoriesDataManager(string host);
    virtual ~TrajectoriesDataManager();

    void connectToDatabase(string host);

    void insertTrajectory(QList<double> trajectory, QString command, QString feature);
    QList<QList<double> > getTrajectory(QString command, QString feature);

    QMap<QString, int> getAllCommandsWithCount();
    QStringList getFeatures(QString command);

private:
    DBClientConnection mongoDB;


};

#endif /* TRAJECTORIESDATAMANAGER_H_ */
