#ifndef MACHINETHREAD_H
#define MACHINETHREAD_H

#include<QTextEdit>
#include<QThread>
#include<QMutex>
#include<QObject>
#include <QtGui/QLabel>
#include<QTimer>
#include<QFile>
#include<QTextStream>
#include "win_qextserialport.h"
#include "math.h"
#include "datafile\data_protocol.h"

class machineThread : public QThread
{
    Q_OBJECT;
    public:
        Win_QextSerialPort *port;
        machineThread(Win_QextSerialPort *port);
        ~machineThread();
        void run();
        //QString port_name;
        QFile file;
        QTextStream out;
    private:
        int byteRecevu;
        int byteEnvoie;
        bool ifsave;
        QTimer *readportTimer;
        void getdataDynamique(char * buff);
        void savedataSpace(QTextStream * dataSpace);
        void savedataDynamique(UN_DATA_DYNAMIQUE);
        void senddataMachine(QTextStream * dataMachine);
    signals:
        void readsignal();
        void showdataSpace();
        void setopenbutton(const char * string);
        void threadError(int error);
        void setinfoDynamique(UN_DATA_DYNAMIQUE dataD);
        void setCRotation(float C);
        void setPRotation(float P);
        void setRRotation(float R);
        void byteRecevuChange(int recevu);
        void byteEnvoieChange(int envoie);
        void portIsOpen(bool state);
    public slots:
        char readport();
        void openport(QString port_name);


};

#endif // MACHINETHREAD_H
