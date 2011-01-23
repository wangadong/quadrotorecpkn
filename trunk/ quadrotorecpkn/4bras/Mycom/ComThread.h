#ifndef COMTHREAD_H
#define COMTHREAD_H

#include<QThread>
#include<QTextEdit>
#include<QMutex>
#include<QObject>
#include <QtGui/QLabel>
#include<QTimer>
#include<QFile>
#include<QTextStream>
#include "win_qextserialport.h"
#include <math.h>


class ComThread: public QWidget
{
    Q_OBJECT;

    public:
        Win_QextSerialPort *port;
        ComThread(Win_QextSerialPort *port);
        ~ComThread();
        void run();
        //QString port_name;
        QFile file;
        QTextStream out;

    private:

        int bytesRced;
        int datasource;
        bool PMstore;
        bool PMdisplay;
        bool PMdraw;
        QTimer *readportTimer;
        void checkDataPack(char* buff);

    signals:
        void readsignal();
        void setopenbtn(const char* openorclose);
        void threaderror(int error);
        void portisopen(bool state);

    public slots:
    char readport();
    void openport(QString port_name);
    void datasourcechanged(int ds);
    void PMstorechanged(bool st);
    void PMdisplaychanged(bool di);
    void PMdrawchanged(bool dr);
    void bytesRcedrct();
};

#endif // COMTHREAD_H
