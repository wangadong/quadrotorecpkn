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

//Label����-----------------------------------------------------------------------------------------
        QLabel * label_acce_x;//x����ٶȼ�
        QLabel * label_acce_y;//y����ٶȼ�
        QLabel * label_acce_z;//z����ٶȼ�
        QLabel * label_ang_x;//x����ٶȼ�
        QLabel * label_ang_y;//y�ܽ��ٶȼ�
        QLabel * label_ang_z;//z�ܽ��ٶȼ�
        QLabel * label_barometre;//��ѹ��

        QLabel * label_ComNum;//ComMum
        QLabel * label_Baudrate;//������
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
//LineEdit����---------------------------------------------------------------------------------------------
        QLineEdit * lineEdit_acce_x;
        QLineEdit * lineEdit_acce_y;
        QLineEdit * lineEdit_acce_z;
        QLineEdit * lineEdit_ang_x;
        QLineEdit * lineEdit_ang_y;
        QLineEdit * lineEdit_ang_z;
        QLineEdit * lineEdit_barometre;
//Combobox����---------------------------------------------------------------------------------------------
        QComboBox * comboBox_ComNum;//ComMum;
        QComboBox * comboBox_Baudrate;//������
        QComboBox * comboBox_Parity_bit;//��żЧ��λ
        QComboBox * comboBox_Data_bit;//����λ
        QComboBox * comboBox_Stop_bit;//��ֹλ
        QComboBox * comboBox_Control_flow;//Control flow
        QComboBox * comboBox_DataSource;//Data source
//TextEdit����---------------------------------------------------------------------------------------------
        QTextEdit * textEdit_Send_out;//Send out
        QTextEdit * textEdit_Recevied;//Recevied
//PushButton����---------------------------------------------------------------------------------------------
        QPushButton * pushButton_OpenCom;
        QPushButton * pushButton_ClearReceptionArea;
        QPushButton * pushButton_Recount;
        QPushButton * pushButton_Send;
        QPushButton * pushButton_LoopSend;
//CheckBox����---------------------------------------------------------------------------------------------
        QCheckBox * checkBox_Storage;
        QCheckBox * checkBox_Display;
        QCheckBox * checkBox_Draw;

        machineThread * thread;

    private:
        void init_com();

        HKEY hKey;
        wchar_t subkey[80];
        wchar_t keyname[256]; //��������
        char keyvalue[256];  //��ֵ����
        DWORD keysize,type,valuesize;
        int indexnum;
        int byteRecevu;
        int byteEnvoie;


        QString getcomm(int index,QString keyorvalue);
        QString msg;


    public slots:
        void opencom_port();//�򿪴���
        void portisopen(bool state);
        void setopenbutton(const char *);
//comboBox���
        void comboBox_Baudrate_currentIndexChanged(const QString &);
        void comboBox_Paritybit_currentIndexChanged(const QString &);
        void comboBox_Databit_currentIndexChanged(const QString &);
        void comboBox_Stopbit_currentIndexChanged(const QString &);
        void comboBox_Controlflow_currentIndexChanged(const QString &);
//checkBox���
        void verifier_check();

        void writeDataSpace(dataSpace dataS);

        void threaderror(int error);
    signals:
        void comopen(QString port_name);
    };

#endif // MYCOM_H
