#ifndef DATA_PROTOCOL_H
#define DATA_PROTOCOL_H

#pragma pack(1)
//空间位置信息
struct dataSpace
{
//    旋转相对，位移绝对(所有数据结构体中x,y,z大写说明绝对坐标系，小写说明相对坐标系)
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
//运动学信息
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
