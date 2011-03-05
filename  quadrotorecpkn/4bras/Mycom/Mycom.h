#ifndef MYCOM_H
#define MYCOM_H


#include <QtGui>
#include <QSettings>
#include <qt_windows.h>
#include <QMutex>
#include <QObject>
#include <QThread>
#include "MachineThread.h"
#include "win_qextserialport.h"
#include "datafile\data_protocol.h"

class Mycom : public QWidget
{
    Q_OBJECT;
    public:
        Mycom();
        Win_QextSerialPort *port;

//Label声明-----------------------------------------------------------------------------------------
        QLabel * label_acce_x;//x轴加速度计
        QLabel * label_acce_y;//y轴加速度计
        QLabel * label_acce_z;//z轴加速度计
        QLabel * label_ang_x;//x轴角速度计
        QLabel * label_ang_y;//y周角速度计
        QLabel * label_ang_z;//z周角速度计
        QLabel * label_barometre;//气压计

        QLabel * label_ComNum;//ComMum
        QLabel * label_Baudrate;//比特率
        QLabel * label_Parity_bit;//Parity bit
        QLabel * label_Data_bit;//Data bit
        QLabel * label_Stop_bit;//Stop bit
        QLabel * label_Control_flow;//Control flow
        QLabel * label_Send_out;//Send out
        QLabel * label_Received;//Received
        QLabel * label_Interval;//Interval
        QLabel * label_DataSource;//Data source
        QLabel * label_ProcessMode;//Process mode

        QLabel * label_Datas_envoyes;//Data qu'on veut envoyer
        QLabel * label_Datas_recevus;//Data qu'on recevoit;
//LineEdit声明---------------------------------------------------------------------------------------------
        QLineEdit * lineEdit_acce_x;
        QLineEdit * lineEdit_acce_y;
        QLineEdit * lineEdit_acce_z;
        QLineEdit * lineEdit_ang_x;
        QLineEdit * lineEdit_ang_y;
        QLineEdit * lineEdit_ang_z;
        QLineEdit * lineEdit_barometre;
//Combobox声明---------------------------------------------------------------------------------------------
        QComboBox * comboBox_ComNum;//ComMum;
        QComboBox * comboBox_Baudrate;//比特率
        QComboBox * comboBox_Parity_bit;//奇偶效验位
        QComboBox * comboBox_Data_bit;//数据位
        QComboBox * comboBox_Stop_bit;//终止位
        QComboBox * comboBox_Control_flow;//Control flow
        QComboBox * comboBox_DataSource;//Data source
//TextEdit声明---------------------------------------------------------------------------------------------
        QTextEdit * textEdit_Send_out;//Send out
        QTextEdit * textEdit_Recevied;//Recevied
//PushButton声明---------------------------------------------------------------------------------------------
        QPushButton * pushButton_OpenCom;
        QPushButton * pushButton_ClearReceptionArea;
        QPushButton * pushButton_Recount;
        QPushButton * pushButton_Send;
        QPushButton * pushButton_LoopSend;
//CheckBox声明---------------------------------------------------------------------------------------------
        QCheckBox * checkBox_Storage;
        QCheckBox * checkBox_Display;
        QCheckBox * checkBox_Draw;

        machineThread * thread;

    private:
        void init_com();

        HKEY hKey;
        wchar_t subkey[80];
        wchar_t keyname[256]; //键名数组
        char keyvalue[256];  //键值数组
        DWORD keysize,type,valuesize;
        int indexnum;
        int byteRecevu;
        int byteEnvoie;


        QString getcomm(int index,QString keyorvalue);
        QString msg;


    public slots:
        void opencom_port();//打开串口
        void portisopen(bool state);
        void setopenbutton(const char *);
//comboBox相关
        void comboBox_Baudrate_currentIndexChanged(const QString &);
        void comboBox_Paritybit_currentIndexChanged(const QString &);
        void comboBox_Databit_currentIndexChanged(const QString &);
        void comboBox_Stopbit_currentIndexChanged(const QString &);
        void comboBox_Controlflow_currentIndexChanged(const QString &);
//checkBox相关
        void verifier_check();

        void writeDataSpace(dataSpace dataS);

        void threaderror(int error);
    signals:
        void comopen(QString port_name);
    };

#endif // MYCOM_H
