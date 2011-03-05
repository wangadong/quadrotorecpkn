#include "ComThread.h"

ComThread::ComThread(Win_QextSerialPort *port)
{
    this->port=port;
    readportTimer=new QTimer(this);
    QObject::connect(readportTimer, SIGNAL(timeout()), this, SLOT(readport()));
    QFile file("datatest.txt");
       if(!file.open(QIODevice::WriteOnly))
         emit threaderror(2);
    QTextStream out(&file);
    out << "Information from 2010-4-24" << endl << "***************************" << endl << endl;
    bytesRced = 0;
    datasource = 0;
    PMstore = false;
    PMdisplay = false;
    PMdraw = false;
}

ComThread::~ComThread()
{
    file.close();
}

void ComThread::run()
{
    port->setBaudRate(BAUD9600);
    port->setFlowControl(FLOW_OFF);
    port->setParity(PAR_NONE);
    port->setDataBits(DATA_8);
    port->setStopBits(STOP_1);
}

void ComThread::openport(QString port_name)
{

    if (!port->isOpen())
    {
        port->setPortName(port_name);
        port->open(QIODevice::ReadWrite);
            if (!port->isOpen())
                emit threaderror(1);
            else
            {
                readportTimer->start(10);
                emit setopenbtn("Close");//emit char open or close,for the pushButton_OpenCom
                emit portisopen(true);
                emit readsignal();
            }
    }
    else
    {
        readportTimer->stop();
        port->close();
        emit setopenbtn("Open");
        emit portisopen(false);
    }
}


//char ComThread::readport()
//{
//    char buff[1];
//    int numBytes;
//    numBytes = port->bytesAvailable();
//        if(numBytes > 0)
//        {
//            port->read(buff, 1);
//            bytesRced+=1;
//            readportTimer->stop();
////            if (datasource == 0)
////                checkDataPack(buff);
////            else if (datasource == 1)
////                writeDataPack(buff);
//            //else;
//        }
//    return buff[0];
//}

