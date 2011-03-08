#include"MachineThread.h"

//------------------------------------------------------------------------------------------------------------
machineThread::machineThread(Win_QextSerialPort *port)
{
    this->port=port;
    port->setTimeout(10);
    readportTimer=new QTimer(this);
    QObject::connect(readportTimer, SIGNAL(timeout()), this, SLOT(readport()));
    QFile file("datatest.txt");
       if(!file.open(QIODevice::WriteOnly))
         emit threadError(2);
    QTextStream out(&file);
    out << "Information from 2011-03-02" << endl << "***************************" << endl << endl;
    byteRecevu = 0;
    byteEnvoie=0;
}
//---------------------------------------------------------------------------------------------------------------
machineThread::~machineThread()
{
    file.close();
}
//-----------------------------------------------------------------------------------------------------------------
void machineThread::run()
{
    port->setBaudRate(BAUD9600);
    port->setFlowControl(FLOW_OFF);
    port->setParity(PAR_NONE);
    port->setDataBits(DATA_8);
    port->setStopBits(STOP_1);
}
//-------------------------------------------------------------------------------------------------------------------
void machineThread::openport(QString port_name)
{

    if (!port->isOpen())
    {
        port->setPortName(port_name);
        port->open(QIODevice::ReadWrite);
            if (!port->isOpen())
                emit threadError(1);
            else
            {
                readportTimer->start(10);
                emit setopenbutton("Close");
                emit portIsOpen(true);
                emit readsignal();
            }
    }
    else
    {
        readportTimer->stop();
        port->close();
        emit setopenbutton("Open");
        emit portIsOpen(false);
    }
}
//---------------------------------------------------------------------------------------------------------
char machineThread::readport()
{
    char buff[14];
    int numBytes;
    numBytes = port->bytesAvailable();
        if(numBytes >= 0)
        {
            port->read(buff,1);
            if(buff[0]=='B')
            {
                byteRecevu+=1;
                emit byteRecevuChange(byteRecevu);
                readportTimer->stop();
                getdataDynamique(buff);
            }
        }
    return buff[0];
}

//---------------------------------------------------------------------------------------------------------
void machineThread::getdataDynamique(char * buff)
{
    UN_DATA_DYNAMIQUE dataD;
    int avail=port->bytesAvailable();
    if (avail>=0)
    {
        unsigned int temp=0;
        port->read(buff,13);
//        Accx
        qDebug("%c",buff[0]);
        temp=((buff[0]<<8)+buff[1]);
        dataD.datadynamique.Accx=float(temp);
//        Accy
        temp=0;
        temp=((buff[2]<<8)+buff[3]);
        dataD.datadynamique.Accy=float(temp);
//        Accz
        temp=0;
        temp=((buff[4]<<8)+buff[5]);
        dataD.datadynamique.Accz=float(temp);
//        Pulx
        temp=0;
        temp=((buff[6]<<8)+buff[7]);
        dataD.datadynamique.Vrotx=float(temp);
//        Puly
        temp=0;
        temp=((buff[8]<<8)+buff[9]);
        dataD.datadynamique.Vroty=float(temp);
//        Pulz
        temp=0;
        temp=((buff[10]<<8)+buff[11]);
        dataD.datadynamique.Vrotz=float(temp);
    }
//    savedataDynamique(dataD);
    emit setinfoDynamique(dataD);
    readportTimer->start();
}
