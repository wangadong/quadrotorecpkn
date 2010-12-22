Change Log
2010-6-6
修正说明：
nwk_ioctl.c

1 nwk_rawSend_withACK函数，把Tx的模式，由CCA模式，改为Forced模式
2 nwk_rawSend函数，把Tx的模式，由CCA模式，改为Forced模式

--------------------------------------------------------------------------------------------------
2010-5-14
裁减说明：

1 nwk_app.h 取消：

//#include "nwk_link.h"， 
//#include "nwk_join.h"
//#include "nwk_ping.h"

2 nwk.c 修改：

   smplStatus_t nwk_nwkInit(uint8_t (*f)(linkID_t)) 
  取消函数中有关nwk_link, nwk_ping, 以及nwk_join部分的初始化

3 nwk_frame.c 修改定义：
static  fhStatus_t (* const func[])(mrfiPacket_t *) = { //nwk_processPing,
                                                        //nwk_processLink, //LISHI - 5 - 14
                                                        //nwk_processJoin, //LISHI - 5 - 14
                                                        nwk_processMgmt
                                                      };

4 nwk_api.h 

//smplStatus_t SMPL_Link(linkID_t *);
//smplStatus_t SMPL_LinkListen(linkID_t *);
//smplStatus_t SMPL_Unlink(linkID_t);
//smplStatus_t SMPL_Send(linkID_t lid, uint8_t *msg, uint8_t len);
//smplStatus_t SMPL_SendOpt(linkID_t lid, uint8_t *msg, uint8_t len, txOpt_t);
//smplStatus_t SMPL_Receive(linkID_t lid, uint8_t *msg, uint8_t *len);
SMPL_Init函数当中的
//rc = nwk_join();

5 nwk_api.c 
取消相应函数说明; 
------------------------------------------------------------------------

2010年4月30日
修改说明：
1 取消nwk_types.h中IO_RADIO_WOROFF的定义。因为这个状态就不存在。
2 nwk.c 中增加定义：uint8_t     ackTID;
3 nwk.c 中，nwk_isConnectionValid函数，增加：ackTID = GET_FROM_FRAME(MRFI_P_PAYLOAD(frame), F_TRACTID_OS);
4 nwk.c 中，nwk_isConnectionValid函数，取消是否广播端口判断;
5 ioctl.h 增加函数定义：
  smplStatus_t nwk_rawSend_withACK(ioctlRawSend_t *);
6 ioctl.c中，增加nwk_rawSend_withACK函数.
7 nwk_api.c中，smplStatus_t SMPL_Ioctl函数，增加

       #if defined(APP_AUTO_ACK)
          rc = nwk_rawSend_withACK((ioctlRawSend_t *)val);
       #else
          rc = nwk_rawSend((ioctlRawSend_t *)val);
       #endif


**带ACK，IOCTL Read/Write通讯的测试代码，在lishi_example中Listen-ioctl-ACK中，以及
Send-ioctl-ACK.c中
需要在smpl_config.dat中配置相应的地址。
发生端： -DTHIS_DEVICE_ADDRESS="{0xA0, 0x00, 0xF1, 0x01}"
接收端： -DTHIS_DEVICE_ADDRESS="{0xA0, 0x00, 0xF1, 0x02}"
------------------------------------------------------------------------

2010年4月29日
END板最终版本
版本说明：

1 END代码在cc2500-LinkListen.c 中;
2 END地址编码，按照最后字节连续编码方式。第三字节设定为0xF1, 固定表示END节点。前两个字节与其父节点SUPEREND地址的前两个字节保持一致;
3 缺省的SUPEREND地址，为{0xA0, 0x00, 0x00, 0x00};
4 Flash C段前4个字节存储END的父节点地址，也就是其父：SUPEREND地址
5 Flash D段，前8个字节，存储END下挂8个传感器的I2C地址。位置为空的传感器，地址为0xFF;
6 END与父节点，也就是SUPEREND通讯协议说明：
   1) 每个SUPEREND -> END 通讯包，包含5个字节，其中第一个字节，为命令字节，后面四个，为数据字节
   2) 命令字节0x41，表示更新SUPEREND地址
   3) 命令字节0x42，表示要更新温湿度传感器地址信息
   4) 命令字节0x43，表示要温湿度数据
   5) 命令字节0x44，表示要电量数据



2010年4月28日
主要测试多对一，也就是一个接收端，同时接收多个发送端发来的信息。
结论如下：

1 通过SMPL_Ioctl( IOCTL_OBJ_RADIO, IOCTL_ACT_RADIO_WORON, 0);设置完WOR状态之后，
至少要等待200ms，以保证硬件状态转换；
2 通过轮询等待，轮流监听，可以实现接收多个发送端数据的流程。
3 接收端如果正在处理收到的某个发射端的数据包，同时另外的发射端传来数据，可能会丢失。也就是说接收端无暇接收和处理。
4 发射端，20次连续发射，接收端WOR状态，被唤醒的概率，可以认为是100%。

-------------------------------------------------------------------------------
2010年4月26日

1 主函数，测试数据互发。
2 测试流程如下：
   1) 发射端发送0x41, 0x41
   2) 接收端受到之后，等待1.5s，然后向发送端发送0x5F, 0x5F，连续发送20次
   3) 发送端发送之后，就处于WOR等待状态，直到接收到0x5F,0x5F之后，再重新开始新的流程。

----------------------------------------------------------------------------
2010年4月24日

1 nwk_types.h中，增加  IOCTL_ACT_RADIO_WORON,  IOCTL_ACT_RADIO_WOROFF 定义
2 nwk_ioctl.c中，增加 
	  else if (IOCTL_ACT_RADIO_WORON == action)
  	  {
    		MRFI_WorOn();
  	  }
3 mrfi.h中，增加WOR相关的定义
4 mrfi_radio.c中，增加MRFI_WorOn，以及MRFI_WorOff函数体
5 mrfi_radio.c中，Mrfi_SyncPinRxIsr函数中，增加
  if(mrfiRadioState == MRFI_RADIO_STATE_WOR)
  {
  	MRFI_WorOff();
  	MRFI_RxOn();
  	mrfiRadioState = MRFI_RADIO_STATE_RX;	
  }
说明：设置WOR模式，只需要：SMPL_Ioctl( IOCTL_OBJ_RADIO, IOCTL_ACT_RADIO_WORON, 0);
其实顶层应用不需要手动退出WOR模式。当CC1101收到唤醒信号，进入同步字中断，开始收包时，
会自动设置状态恢复到RxOn状态。

-----------------------------------------------------------------------
本套代码，增加End通讯测试功能，测试说明与结论如下：

1 本系统测试主要测试SMPL_Ioctl( IOCTL_OBJ_RAW_IO, IOCTL_ACT_WRITE/READ, &Info)来进行数据通讯，通讯测试成功。
2 几点说明：
  1) Info结构中，Port必须设置成0x3F。目前测试设置其他的，都不对；
  2) SMPL_Ioctl( IOCTL_OBJ_RAW_IO, IOCTL_ACT_READ, ..)之前，必须手工设置RXON状态：
	 SMPL_Ioctl( IOCTL_OBJ_RADIO, IOCTL_ACT_RADIO_RXON, 0);
  3) 地址赋值，可以用此形式：addr_t add = {0xA0, 0x00, 0x00, 0x01};
  4) SMPL_Ioctl与中断响应不冲突。所以可以在中断响应函数中调用。测试使用TimerB进行定时，发送     数据，发送成功;
  5) AutoACK不灵，需要自己做数据发送成功判断。目前发送方，没有阻断，即使返回SMPL_SUCCESS，也      是指数据被成功发出，不保证是否被成功接收。所以需要接收端进行判断，如果没有收到，还需要     再次请求。


------------------------------------------------------------
二级裁减，裁减内容包括：

1 BSP目录
2 MRFI目录

基本没有减少，因为编译器会优化代码，把那些ifdef的东西去掉。
不过这次裁减，可以让代码清晰很多。

---------------------------------------------------------------------------------------
初步裁减，裁减内容包括：

1 取消Nwk_pplicaion中，有关Freq，以及Security两个模块
2 将SMPL_Unlink函数，从Extend API中，移植到普通API中。这样就不用打开Extend API调用开关，不用加载其他API函数。
3 取消Button调用。

当前容量：
LinkTo： 6600
LinkListen: 6326
