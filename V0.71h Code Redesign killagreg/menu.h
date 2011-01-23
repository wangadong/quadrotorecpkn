#ifndef _MENU_H
#define _MENU_H

#include <inttypes.h>

#define DISPLAYBUFFSIZE 80

extern void LCD_PrintMenu(void);
extern void LCD_Clear(void);
extern int8_t DisplayBuff[DISPLAYBUFFSIZE];
extern uint8_t DispPtr;
extern uint8_t MenuItem;
extern uint8_t MaxMenuItem;
extern uint8_t RemoteKeys;
#endif //_MENU_H


