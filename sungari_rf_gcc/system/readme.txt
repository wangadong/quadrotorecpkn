Change Log
2010-6-6
����˵����
nwk_ioctl.c

1 nwk_rawSend_withACK��������Tx��ģʽ����CCAģʽ����ΪForcedģʽ
2 nwk_rawSend��������Tx��ģʽ����CCAģʽ����ΪForcedģʽ

--------------------------------------------------------------------------------------------------
2010-5-14
�ü�˵����

1 nwk_app.h ȡ����

//#include "nwk_link.h"�� 
//#include "nwk_join.h"
//#include "nwk_ping.h"

2 nwk.c �޸ģ�

   smplStatus_t nwk_nwkInit(uint8_t (*f)(linkID_t)) 
  ȡ���������й�nwk_link, nwk_ping, �Լ�nwk_join���ֵĳ�ʼ��

3 nwk_frame.c �޸Ķ��壺
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
SMPL_Init�������е�
//rc = nwk_join();

5 nwk_api.c 
ȡ����Ӧ����˵��; 
------------------------------------------------------------------------

2010��4��30��
�޸�˵����
1 ȡ��nwk_types.h��IO_RADIO_WOROFF�Ķ��塣��Ϊ���״̬�Ͳ����ڡ�
2 nwk.c �����Ӷ��壺uint8_t     ackTID;
3 nwk.c �У�nwk_isConnectionValid���������ӣ�ackTID = GET_FROM_FRAME(MRFI_P_PAYLOAD(frame), F_TRACTID_OS);
4 nwk.c �У�nwk_isConnectionValid������ȡ���Ƿ�㲥�˿��ж�;
5 ioctl.h ���Ӻ������壺
  smplStatus_t nwk_rawSend_withACK(ioctlRawSend_t *);
6 ioctl.c�У�����nwk_rawSend_withACK����.
7 nwk_api.c�У�smplStatus_t SMPL_Ioctl����������

       #if defined(APP_AUTO_ACK)
          rc = nwk_rawSend_withACK((ioctlRawSend_t *)val);
       #else
          rc = nwk_rawSend((ioctlRawSend_t *)val);
       #endif


**��ACK��IOCTL Read/WriteͨѶ�Ĳ��Դ��룬��lishi_example��Listen-ioctl-ACK�У��Լ�
Send-ioctl-ACK.c��
��Ҫ��smpl_config.dat��������Ӧ�ĵ�ַ��
�����ˣ� -DTHIS_DEVICE_ADDRESS="{0xA0, 0x00, 0xF1, 0x01}"
���նˣ� -DTHIS_DEVICE_ADDRESS="{0xA0, 0x00, 0xF1, 0x02}"
------------------------------------------------------------------------

2010��4��29��
END�����հ汾
�汾˵����

1 END������cc2500-LinkListen.c ��;
2 END��ַ���룬��������ֽ��������뷽ʽ�������ֽ��趨Ϊ0xF1, �̶���ʾEND�ڵ㡣ǰ�����ֽ����丸�ڵ�SUPEREND��ַ��ǰ�����ֽڱ���һ��;
3 ȱʡ��SUPEREND��ַ��Ϊ{0xA0, 0x00, 0x00, 0x00};
4 Flash C��ǰ4���ֽڴ洢END�ĸ��ڵ��ַ��Ҳ�����丸��SUPEREND��ַ
5 Flash D�Σ�ǰ8���ֽڣ��洢END�¹�8����������I2C��ַ��λ��Ϊ�յĴ���������ַΪ0xFF;
6 END�븸�ڵ㣬Ҳ����SUPERENDͨѶЭ��˵����
   1) ÿ��SUPEREND -> END ͨѶ��������5���ֽڣ����е�һ���ֽڣ�Ϊ�����ֽڣ������ĸ���Ϊ�����ֽ�
   2) �����ֽ�0x41����ʾ����SUPEREND��ַ
   3) �����ֽ�0x42����ʾҪ������ʪ�ȴ�������ַ��Ϣ
   4) �����ֽ�0x43����ʾҪ��ʪ������
   5) �����ֽ�0x44����ʾҪ��������



2010��4��28��
��Ҫ���Զ��һ��Ҳ����һ�����նˣ�ͬʱ���ն�����Ͷ˷�������Ϣ��
�������£�

1 ͨ��SMPL_Ioctl( IOCTL_OBJ_RADIO, IOCTL_ACT_RADIO_WORON, 0);������WOR״̬֮��
����Ҫ�ȴ�200ms���Ա�֤Ӳ��״̬ת����
2 ͨ����ѯ�ȴ�����������������ʵ�ֽ��ն�����Ͷ����ݵ����̡�
3 ���ն�������ڴ����յ���ĳ������˵����ݰ���ͬʱ����ķ���˴������ݣ����ܻᶪʧ��Ҳ����˵���ն���Ͼ���պʹ���
4 ����ˣ�20���������䣬���ն�WOR״̬�������ѵĸ��ʣ�������Ϊ��100%��

-------------------------------------------------------------------------------
2010��4��26��

1 ���������������ݻ�����
2 �����������£�
   1) ����˷���0x41, 0x41
   2) ���ն��ܵ�֮�󣬵ȴ�1.5s��Ȼ�����Ͷ˷���0x5F, 0x5F����������20��
   3) ���Ͷ˷���֮�󣬾ʹ���WOR�ȴ�״̬��ֱ�����յ�0x5F,0x5F֮�������¿�ʼ�µ����̡�

----------------------------------------------------------------------------
2010��4��24��

1 nwk_types.h�У�����  IOCTL_ACT_RADIO_WORON,  IOCTL_ACT_RADIO_WOROFF ����
2 nwk_ioctl.c�У����� 
	  else if (IOCTL_ACT_RADIO_WORON == action)
  	  {
    		MRFI_WorOn();
  	  }
3 mrfi.h�У�����WOR��صĶ���
4 mrfi_radio.c�У�����MRFI_WorOn���Լ�MRFI_WorOff������
5 mrfi_radio.c�У�Mrfi_SyncPinRxIsr�����У�����
  if(mrfiRadioState == MRFI_RADIO_STATE_WOR)
  {
  	MRFI_WorOff();
  	MRFI_RxOn();
  	mrfiRadioState = MRFI_RADIO_STATE_RX;	
  }
˵��������WORģʽ��ֻ��Ҫ��SMPL_Ioctl( IOCTL_OBJ_RADIO, IOCTL_ACT_RADIO_WORON, 0);
��ʵ����Ӧ�ò���Ҫ�ֶ��˳�WORģʽ����CC1101�յ������źţ�����ͬ�����жϣ���ʼ�հ�ʱ��
���Զ�����״̬�ָ���RxOn״̬��

-----------------------------------------------------------------------
���״��룬����EndͨѶ���Թ��ܣ�����˵����������£�

1 ��ϵͳ������Ҫ����SMPL_Ioctl( IOCTL_OBJ_RAW_IO, IOCTL_ACT_WRITE/READ, &Info)����������ͨѶ��ͨѶ���Գɹ���
2 ����˵����
  1) Info�ṹ�У�Port�������ó�0x3F��Ŀǰ�������������ģ������ԣ�
  2) SMPL_Ioctl( IOCTL_OBJ_RAW_IO, IOCTL_ACT_READ, ..)֮ǰ�������ֹ�����RXON״̬��
	 SMPL_Ioctl( IOCTL_OBJ_RADIO, IOCTL_ACT_RADIO_RXON, 0);
  3) ��ַ��ֵ�������ô���ʽ��addr_t add = {0xA0, 0x00, 0x00, 0x01};
  4) SMPL_Ioctl���ж���Ӧ����ͻ�����Կ������ж���Ӧ�����е��á�����ʹ��TimerB���ж�ʱ������     ���ݣ����ͳɹ�;
  5) AutoACK���飬��Ҫ�Լ������ݷ��ͳɹ��жϡ�Ŀǰ���ͷ���û����ϣ���ʹ����SMPL_SUCCESS��Ҳ      ��ָ���ݱ��ɹ�����������֤�Ƿ񱻳ɹ����ա�������Ҫ���ն˽����жϣ����û���յ�������Ҫ     �ٴ�����


------------------------------------------------------------
�����ü����ü����ݰ�����

1 BSPĿ¼
2 MRFIĿ¼

����û�м��٣���Ϊ���������Ż����룬����Щifdef�Ķ���ȥ����
������βü��������ô��������ܶࡣ

---------------------------------------------------------------------------------------
�����ü����ü����ݰ�����

1 ȡ��Nwk_pplicaion�У��й�Freq���Լ�Security����ģ��
2 ��SMPL_Unlink��������Extend API�У���ֲ����ͨAPI�С������Ͳ��ô�Extend API���ÿ��أ����ü�������API������
3 ȡ��Button���á�

��ǰ������
LinkTo�� 6600
LinkListen: 6326
