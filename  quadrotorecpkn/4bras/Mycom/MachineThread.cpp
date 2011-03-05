#include"MachineThread.h"

//------------------------------------------------------------------------------------------------------------
machineThread::machineThread(Win_QextSerialPort *port)
{
    this->port=port;
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
    char * buff;
    int numBytes;
    numBytes = port->bytesAvailable();
        if(numBytes > 0)
        {
            port->read(buff, 1);
            byteRecevu+=1;
            emit byteRecevuChange(byteRecevu);
            readportTimer->stop(); 
            getdataSpace(buff);
        }
    return buff[0];
}

//---------------------------------------------------------------------------------------------------------
void machineThread::getdataSpace(char* buff)
{
    UN_DATA_SPACE dataS;
    port->read(buff,1);
    dataS.dataspace.RotC = float(* buff);
    emit setCRotation(dataS.dataspace.RotC);
    readportTimer->start();
}
