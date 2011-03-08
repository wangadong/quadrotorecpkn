#include "Mycom.h"

Mycom::Mycom()
{
    setWindowTitle(tr("Serial Port"));
//Creat bases
//Creat label------------------------------------------------------------------------
    label_acce_x=new QLabel(tr("Acceleration d'axe X"));
    label_acce_y=new QLabel(tr("Acceleration d'axe Y"));
    label_acce_z=new QLabel(tr("Acceleration d'axe Z"));
    label_ang_x=new QLabel(tr("Pulsation d'axe X"));
    label_ang_y=new QLabel(tr("Pulsation d'axe Y"));
    label_ang_z=new QLabel(tr("Pulsation d'axe Z"));
    label_barometre=new QLabel(tr("Barometre"));

    label_ComNum=new QLabel (tr("ComNum")) ;
    label_Baudrate=new QLabel(tr("Baud rate"));
    label_Parity_bit=new QLabel(tr("Parity bit"));
    label_Data_bit=new QLabel(tr("Data rit"));
    label_Stop_bit=new QLabel(tr("Stop bit"));
    label_Control_flow=new QLabel(tr("Control flow"));
    label_Send_out=new QLabel(tr("Send out"));
    label_Received=new QLabel(tr("Recived"));
    label_Interval=new QLabel(tr("Interval"));
    label_Numrecevu=new QLabel(tr("Data recevu number"));
    label_Numenvoie=new QLabel(tr("Data envoie number"));

    label_Datas_envoyes=new QLabel(tr("Datas envoyes"));
    label_Datas_recevus=new QLabel(tr("Datas Recevus"));
 //Creat lineedir------------------------------------------------------------------------
    lineEdit_acce_x = new QLineEdit();
    lineEdit_acce_y = new QLineEdit();
    lineEdit_acce_z = new QLineEdit();
    lineEdit_ang_x = new QLineEdit();
    lineEdit_ang_y = new QLineEdit();
    lineEdit_ang_z = new QLineEdit();
    lineEdit_barometre=new QLineEdit();
    lineEdit_Numrecevu = new QLineEdit();
    lineEdit_Numenvoie = new QLineEdit();
 //Creat Combobox---------------------------------------------------------------------------------------------
    comboBox_ComNum = new QComboBox();  //Comname
    comboBox_Baudrate = new QComboBox();  //Baudrate
    comboBox_Parity_bit = new QComboBox();  //Paritybit
    comboBox_Data_bit = new QComboBox();  //Databit
    comboBox_Stop_bit = new QComboBox();  //Stopbit
    comboBox_Control_flow = new QComboBox();  //Controlflow
 //Creat Textedit---------------------------------------------------------------------------------------------
    textEdit_Send_out = new QTextEdit();
    textEdit_Recevied = new QTextEdit;
    textEdit_Recevied->setReadOnly(true);
//Creat Pushbutton---------------------------------------------------------------------------------------------
    pushButton_OpenCom = new QPushButton();
    pushButton_OpenCom->setText(tr("OpenCom"));
    pushButton_ClearReceptionArea = new QPushButton();
    pushButton_ClearReceptionArea->setText(tr("ClearReceptionArea"));
    pushButton_Recount = new QPushButton();
    pushButton_Recount->setText(tr("Recount"));
    pushButton_Send = new QPushButton();
    pushButton_Send->setText(tr("Send"));
    pushButton_LoopSend = new QPushButton();
    pushButton_LoopSend->setText(tr("LoopSend"));
//Layout
//Uplayout:gird
    QGridLayout * UpLayout = new QGridLayout();
    UpLayout->addWidget(label_acce_x,0,0);
    UpLayout->addWidget(label_acce_y,1,0);
    UpLayout->addWidget(label_acce_z,2,0);
    UpLayout->addWidget(label_ang_x,3,0);
    UpLayout->addWidget(label_ang_y,4,0);
    UpLayout->addWidget(label_ang_z,5,0);
    UpLayout->addWidget(label_barometre,6,0);

    UpLayout->addWidget(lineEdit_acce_x,0,1);
    UpLayout->addWidget(lineEdit_acce_y,1,1);
    UpLayout->addWidget(lineEdit_acce_z,2,1);
    UpLayout->addWidget(lineEdit_ang_x,3,1);
    UpLayout->addWidget(lineEdit_ang_y,4,1);
    UpLayout->addWidget(lineEdit_ang_z,5,1);
    UpLayout->addWidget(lineEdit_barometre,6,1);
//Midlayout:gird
    QGridLayout * MidLayout=new QGridLayout();
    MidLayout->addWidget(label_ComNum,0,0);
    MidLayout->addWidget(label_Baudrate,1,0);
    MidLayout->addWidget(label_Parity_bit,2,0);
    MidLayout->addWidget(label_Data_bit,3,0);
    MidLayout->addWidget(label_Stop_bit,4,0);
    MidLayout->addWidget(label_Control_flow,0,2);
    MidLayout->addWidget(label_Numrecevu,1,2);
    MidLayout->addWidget(label_Numenvoie,2,2);

    MidLayout->addWidget(comboBox_ComNum,0,1);
    MidLayout->addWidget(comboBox_Baudrate,1,1);
    MidLayout->addWidget(comboBox_Parity_bit,2,1);
    MidLayout->addWidget(comboBox_Data_bit,3,1);
    MidLayout->addWidget(comboBox_Stop_bit,4,1);
    MidLayout->addWidget(comboBox_Control_flow,0,3);
    MidLayout->addWidget(lineEdit_Numrecevu,1,3);
    MidLayout->addWidget(lineEdit_Numenvoie,2,3);
//Baselayout:vertical
    QVBoxLayout * BaseLayout=new QVBoxLayout();
    BaseLayout->addWidget(label_Datas_envoyes);
    BaseLayout->addWidget(textEdit_Send_out);
    BaseLayout->addWidget(label_Datas_recevus);
    BaseLayout->addWidget(textEdit_Recevied);
    BaseLayout->addWidget(pushButton_OpenCom);
    BaseLayout->addWidget(pushButton_ClearReceptionArea);
    BaseLayout->addWidget(pushButton_Recount);
    BaseLayout->addWidget(pushButton_Send);
    BaseLayout->addWidget(pushButton_LoopSend);
//Mainlayout:vertical
    QVBoxLayout * mainLayout = new QVBoxLayout(this);
    this->setLayout(mainLayout);
    mainLayout->addLayout(UpLayout);
    mainLayout->addLayout(MidLayout);
    mainLayout->addLayout(BaseLayout);
//Init
    init_com();
//Creat port
    port = new Win_QextSerialPort();
    byteRecevu = byteEnvoie = 0;
    thread = new machineThread(port);
    thread->start();
//comboBox相关
    QObject::connect(comboBox_Baudrate,SIGNAL(currentIndexChanged(const QString &)),
                     this,SLOT(comboBox_Baudrate_currentIndexChanged(const QString &)));
    QObject::connect(comboBox_Parity_bit,SIGNAL(currentIndexChanged(const QString &)),
                     this,SLOT(comboBox_Paritybit_currentIndexChanged(const QString &)));
    QObject::connect(comboBox_Data_bit,SIGNAL(currentIndexChanged(const QString &)),
                     this,SLOT(comboBox_Databit_currentIndexChanged(const QString &)));
    QObject::connect(comboBox_Stop_bit,SIGNAL(currentIndexChanged(const QString &)),
                     this,SLOT(comboBox_Stopbit_currentIndexChanged(const QString &)));
    QObject::connect(comboBox_Control_flow,SIGNAL(currentIndexChanged(const QString &)),
                     this,SLOT(comboBox_Controlflow_currentIndexChanged(const QString &)));
//button相关
    QObject::connect(pushButton_OpenCom, SIGNAL(clicked()), this, SLOT(opencom_port()));
    QObject::connect(thread,SIGNAL(setopenbutton(const char*)),this,SLOT(setopenbutton(const char*)));
    QObject::connect(pushButton_ClearReceptionArea,SIGNAL(clicked()),this,SLOT(cleartextArea()));
    QObject::connect(pushButton_Recount,SIGNAL(clicked()),this,SLOT(recountNum()));
//串口相关
    QObject::connect(this,SIGNAL(comopen(QString)),thread,SLOT(openport(QString)));
    QObject::connect(thread,SIGNAL(threadError(int)),this,SLOT(threaderror(int)));
    QObject::connect(thread,SIGNAL(portIsOpen(bool)),this,SLOT(portisopen(bool)));
    QObject::connect(thread,SIGNAL(setinfoDynamique(UN_DATA_DYNAMIQUE)),this,SLOT(setinfoDynamique(UN_DATA_DYNAMIQUE)));
 }
//-----------------------------------------------------------------------------------------------------------
void Mycom::init_com()
{

    QString path="HKEY_LOCAL_MACHINE\\HARDWARE\\DEVICEMAP\\SERIALCOMM";
    QSettings *settings=new QSettings(path,QSettings::NativeFormat);
    QStringList key=settings->allKeys();
    QStringList comlist ;   //comX 表
    QStringList Baudlist ;//可选比特率列表
    QStringList Paritylist ;//可选奇偶效验位
    QStringList DataBitslist;//可选数据位
    QStringList StopBitslist;//可选终止位
    QStringList ControlFlowlist;//是否使用流量控制
    msg="";

    int kk = key.size();
    int i;
    comlist.clear();
    for(i=0;i<kk;i++)
    {
      comlist << getcomm(i,"value");
    }
    comboBox_ComNum->addItems(comlist);
    Baudlist.clear();
    Baudlist<< "300" << "600" << "1200" << "2400" << "4800" << "9600"
            << "19200" << "38400" << "56000" << "57600" << "115200";
    comboBox_Baudrate->addItems(Baudlist);

    Paritylist.clear();
    Paritylist<< "NONE" << "ODD" << "EVEN";//ODD为奇效验，EVEN为偶效验
    comboBox_Parity_bit->addItems(Paritylist);

    DataBitslist.clear();
    DataBitslist<< "8" << "7" << "6";
    comboBox_Data_bit->addItems(DataBitslist);

    StopBitslist.clear();
    StopBitslist<< "1" << "2";
    comboBox_Stop_bit->addItems(StopBitslist);

    ControlFlowlist.clear();
    ControlFlowlist<< "NONE" << "XON/XOFF" << "Hard";
    comboBox_Control_flow->addItems(ControlFlowlist);

//        qDebug("com list init!\n");
}
//----------------------------------------------------------------------------------------------------
QString Mycom::getcomm(int index,QString keyorvalue)
{
    QString commresult="";
    QString strkey="HARDWARE\\DEVICEMAP\\SERIALCOMM";//子键路径
    int a=strkey.toWCharArray(subkey);//类型转换string to array
     subkey[a]=L'\0';
    if(RegOpenKeyEx(HKEY_LOCAL_MACHINE,subkey,0,KEY_READ|KEY_QUERY_VALUE,&hKey)!=0)
     {
         QString error="Cannot open regedit!";//无法打开注册表时返回error
       }

    QString keymessage="";//键名
    QString message="";
    QString valuemessage="";//键值
    indexnum=index;//要读取键值的索引号

    keysize=sizeof(keyname);
    valuesize=sizeof(keyvalue);

     if(::RegEnumValue(hKey,indexnum,keyname,&keysize,0,&type,(BYTE*)keyvalue,&valuesize)==0)//列举键名和值，RegEnumValue枚举指定项的值
      {
         for(int i=0;i<keysize;i++)
             {
                 message=QString::fromStdWString(keyname);
                 keymessage.append(message);
               }// for(int i=0;i<=keysize;i++)    读取键名
           for(int j=0;j<valuesize;j++)
               {
                 if(keyvalue[j]!=0x00)//0x00是16进制的0
                 { valuemessage.append(keyvalue[j]);}
               }//for(int j=0;j<valuesize;j++) 读取键值
           if(keyorvalue=="key")
             {
               commresult=keymessage;
             }
           if(keyorvalue=="value")
             {
               commresult=valuemessage;
             }
     }
     else
     {
         commresult="nokey";
     }
::RegCloseKey(hKey);//关闭注册表
return commresult;
}
//Combox的相关槽函数
//串口选择comboBox
void Mycom::opencom_port()
{
    QString port_name = comboBox_ComNum->currentText();
    emit comopen(port_name);
}
//-----------------------------------------------------------------------------------------
//比特率comboBox
void Mycom::comboBox_Baudrate_currentIndexChanged(const QString &baud)
{
          qDebug("BaudRate is change!\n");
    int i = baud.toInt();
        switch(i)
        {
        case 300:
                port->setBaudRate(BAUD300);
                break;
        case 600:
                port->setBaudRate(BAUD600);
                break;
        case 1200:
                port->setBaudRate(BAUD1200);
                break;
        case 2400:
                port->setBaudRate(BAUD2400);
                break;
        case 4800:
                port->setBaudRate(BAUD4800);
                break;
        case 9600:
                port->setBaudRate(BAUD9600);
                break;
        case 19200:
                port->setBaudRate(BAUD19200);
                break;
        case 38400:
                port->setBaudRate(BAUD38400);
                break;
        case 56000:
                port->setBaudRate(BAUD56000);
                break;
        case 57600:
                port->setBaudRate(BAUD57600);
                break;
        case 115200:
                port->setBaudRate(BAUD115200);
                break;
        }
}
//-----------------------------------------------------------------------------------------
//奇偶效验comboBox
void Mycom::comboBox_Paritybit_currentIndexChanged(const QString &Paritybit)
{
    qDebug("Parity bit is change!\n");
    if(Paritybit==tr("NONE")) port->setParity(PAR_NONE);
    else if(Paritybit==tr("ODD")) port->setParity(PAR_ODD);
    else if(Paritybit==tr("EVEN")) port->setParity(PAR_EVEN);

}
//-----------------------------------------------------------------------------------------
//数据位comboBox
void Mycom::comboBox_Databit_currentIndexChanged(const QString &Databit)
{
    qDebug("Databit bit is change!\n");
    int i=Databit.toInt();
    switch(i)
    {
    case 6:
        port->setDataBits(DATA_6);
        break;
    case 7:
        port->setDataBits(DATA_7);
        break;
    case 8:
        port->setDataBits(DATA_8);
        break;
    }
}
//-----------------------------------------------------------------------------------------
//终止位comboBox
void Mycom::comboBox_Stopbit_currentIndexChanged(const QString &Stopbit)
{
    if(Stopbit==tr("1")) port->setStopBits(STOP_1);
    else if(Stopbit==tr("2")) port->setStopBits(STOP_2);
}
//-----------------------------------------------------------------------------------------
//流量控制comboBox
void Mycom::comboBox_Controlflow_currentIndexChanged(const QString &Controlflow)
{
    if(Controlflow==tr("NONE")) port->setFlowControl(FLOW_OFF);
    else if(Controlflow==tr("XON/XOFF")) port->setFlowControl(FLOW_XONXOFF);
    else if(Controlflow==tr("Hard")) port->setFlowControl(FLOW_HARDWARE);
}
//-----------------------------------------------------------------------------------------
//checkBox相关
void Mycom::verifier_check()
{

}

void Mycom::portisopen(bool state)
{
if (state)
    {
        comboBox_ComNum->setEnabled(false);
        comboBox_Baudrate->setEnabled(false);
        comboBox_Parity_bit->setEnabled(false);
        comboBox_Data_bit->setEnabled(false);
        comboBox_Stop_bit->setEnabled(false);
        comboBox_Control_flow->setEnabled(false);
    }
else
    {
        comboBox_ComNum->setEnabled(true);
        comboBox_Baudrate->setEnabled(true);
        comboBox_Parity_bit->setEnabled(true);
        comboBox_Data_bit->setEnabled(true);
        comboBox_Stop_bit->setEnabled(true);
        comboBox_Control_flow->setEnabled(true);
    }
}
//--------------------------------------------------------------------------------------------------
void Mycom::writedataSpace(char * datatemp)
{
    textEdit_Recevied->append("dataSpace_general information:\n");
//    textEdit_Recevied->insertPlainText(";  \n PosX: ");
//    textEdit_Recevied->insertPlainText(msg.number(dataS.PosX,'g',16));
//    textEdit_Recevied->insertPlainText(";  \n PosY: ");
//    textEdit_Recevied->insertPlainText(msg.number(dataS.PosY,'g',16));
//    textEdit_Recevied->insertPlainText(";  \n PosZ: ");
//    textEdit_Recevied->insertPlainText(msg.number(dataS.PosZ,'g',16));
//    textEdit_Recevied->insertPlainText(";  \n RotC: ");
//    textEdit_Recevied->insertPlainText(msg.number(dataS.RotC,'g',16));
//    textEdit_Recevied->insertPlainText(";  \n RotP: ");
//    textEdit_Recevied->insertPlainText(msg.number(dataS.RotP,'g',16));
//    textEdit_Recevied->insertPlainText(";  \n RotR: ");
//    textEdit_Recevied->insertPlainText(msg.number(dataS.RotR,'g',16));
//    textEdit_Recevied->insertPlainText("\n\n");
    for(int i=0;i<=9;i++)
    {
       msg.setNum(double(float(int(datatemp[i]))-48.0));
       textEdit_Recevied->insertPlainText(msg);
       textEdit_Recevied->insertPlainText("\n");
    }
//    textEdit_Recevied->insertPlainText((datatemp[0]));
//    textEdit_Recevied->insertPlainText("\n");
//    textEdit_Recevied->insertPlainText((datatemp[1]));
//    textEdit_Recevied->insertPlainText("\n");
//    textEdit_Recevied->insertPlainText((datatemp[2]));
//    textEdit_Recevied->insertPlainText("\n");
//    textEdit_Recevied->insertPlainText((datatemp[3]));
//    textEdit_Recevied->insertPlainText("\n");
//    textEdit_Recevied->insertPlainText((datatemp[4]));
//    textEdit_Recevied->insertPlainText("\n");
//    textEdit_Recevied->insertPlainText((datatemp[5]));
//    textEdit_Recevied->insertPlainText("\n");
//    textEdit_Recevied->insertPlainText((datatemp[6]));
//    textEdit_Recevied->insertPlainText("\n");
//    textEdit_Recevied->insertPlainText((datatemp[7]));
//    textEdit_Recevied->insertPlainText("\n");
//    textEdit_Recevied->insertPlainText((datatemp[8]));
//    textEdit_Recevied->insertPlainText("\n");
//    textEdit_Recevied->insertPlainText((datatemp[9]));
//    textEdit_Recevied->insertPlainText("\n");
}
//--------------------------------------------------------------------------------------------------------
void Mycom::setopenbutton(const char* openorclose)
{
    pushButton_OpenCom->setText(tr(openorclose));
}
//------------------------------------------------------------------------------------------------------------
void Mycom::threaderror(int error)
{

}
//------------------------------------------------------------------------------------------------------------
void Mycom::cleartextArea()
{
    textEdit_Send_out->clear();
    textEdit_Recevied->clear();
}
//-----------------------------------------------------------------------------------------------------------
void Mycom::recountNum()
{
    lineEdit_Numrecevu->clear();
    lineEdit_Numenvoie->clear();
}
//------------------------------------------------------------------------------------------------------------
void Mycom::setinfoDynamique(UN_DATA_DYNAMIQUE dataD)
{
    lineEdit_acce_x->clear();
    lineEdit_acce_x->insert(msg.number(dataD.datadynamique.Accx));
    lineEdit_acce_y->clear();
    lineEdit_acce_y->insert(msg.number(dataD.datadynamique.Accy));
    lineEdit_acce_z->clear();
    lineEdit_acce_z->insert(msg.number(dataD.datadynamique.Accz));
    lineEdit_ang_x->clear();
    lineEdit_ang_x->insert(msg.number(dataD.datadynamique.Vrotx));
    lineEdit_ang_y->clear();
    lineEdit_ang_y->insert(msg.number(dataD.datadynamique.Vroty));
    lineEdit_ang_z->clear();
    lineEdit_ang_z->insert(msg.number(dataD.datadynamique.Vrotz));
}
//-------------------------------------------------------------------------------------------------------------
