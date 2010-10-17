#ifndef APP_H
#define APP_H

enum ctrlIds {
	ID_NOTEBOOK,
	ID_BT_START,
	ID_BT_STOP,
	ID_CHOICE_COMID,
	ID_TxtCtl_NODEB,
	ID_GRID_INFO_END,
	ID_GRID_INFO_SENSOR,
	ID_TEXTLOG,
	ID_TOOLBAR,
	ID_TIMER,
	Field_Clock
};

#define MESSAGE_LENGTH 30
#define MESSAGE_ID_COMMAND 1
#define MESSAGE_ID 3

#define MESSAGE_ADDR_NODE 26
#define MESSAGE_ADDR_RELAY 28
#define ADDRESS_LENGTH 2
#define ADDRESS_NUMBER 100

#define COM_BUF_SIZE        2048
#define MAX_COM_NUM         (int)8
#define READV_TIMEOUT		5

#define MAX_RELAY_ID 1000

struct command_t{
	int num;
	int packet[ADDRESS_NUMBER];
	int total;
	int all;
};

struct relay_t {
	int num;
	command_t command[MAX_RELAY_ID];
	int total;
	int all;
};

#endif

