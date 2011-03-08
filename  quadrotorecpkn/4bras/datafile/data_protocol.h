#ifndef DATA_PROTOCOL_H
#define DATA_PROTOCOL_H

#pragma pack(1)
//�ռ�λ����Ϣ
struct dataSpace
{
//    ��ת��ԣ�λ�ƾ���(�������ݽṹ����x,y,z��д˵����������ϵ��Сд˵���������ϵ)
    float RotC;
    float RotP;
    float RotR;
    float PosX;
    float PosY;
    float PosZ;
};

union UN_DATA_SPACE
{
    struct dataSpace dataspace;
    char temp[10];
};

//------------------------
//�˶�ѧ��Ϣ
struct dataDynamique
{
    float Accx;
    float Accy;
    float Accz;
    float Vrotx;
    float Vroty;
    float Vrotz;
};

union UN_DATA_DYNAMIQUE
{
    struct dataDynamique datadynamique;
    char temp[10];
};

#pragma pack()
#endif // DATA_PROTOCOL_H
