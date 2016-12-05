#ifndef __DELAY_H
#define __DELAY_H 			   
#include "sys.h"  
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供學習使用，未經作者許可，不得用於其它任何用途
//ALIENTEK STM32開發板
//使用SysTick的普通計數模式對延遲進行管理（適合STM32F10x系列）
//包括delay_us,delay_ms
//正點原子@ALIENTEK
//技術論壇:www.openedv.com
//創建日期:2010/1/1
//版本：V1.8
//版權所有，盜版必究。
//Copyright(C) 廣州市星翼電子科技有限公司 2009-2019
//All rights reserved
//********************************************************************************
//V1.2修改說明
//修正了中斷中調用出現死循環的錯誤
//防止延時不準確,採用do while結構!
//V1.3修改說明
//增加了對UCOSII延時的支持.
//如果使用ucosII,delay_init會自動設置SYSTICK的值,使之與ucos的TICKS_PER_SEC對應.
//delay_ms和delay_us也進行了針對ucos的改造.
//delay_us可以在ucos下使用,而且準確度很高,更重要的是沒有占用額外的定時器.
//delay_ms在ucos下,可以當成OSTimeDly來用,在未啟動ucos時,它採用delay_us實現,從而準確延時
//可以用來初始化外設,在啟動了ucos之後delay_ms根據延時的長短,選擇OSTimeDly實現或者delay_us實現.
//V1.4修改說明 20110929
//修改了使用ucos,但是ucos未啟動的時候,delay_ms中中斷無法響應的bug.
//V1.5修改說明 20120902
//在delay_us加入ucos上鎖，防止由於ucos打斷delay_us的執行，可能導致的延時不準。
//V1.6修改說明 20150109
//在delay_ms加入OSLockNesting判斷。
//V1.7修改說明 20150319
//修改OS支持方式,以支持任意OS(不限於UCOSII和UCOSIII,理論上任意OS都可以支持)
//添加:delay_osrunning/delay_ostickspersec/delay_osintnesting三個宏定義
//添加:delay_osschedlock/delay_osschedunlock/delay_ostimedly三個函數
//V1.8修改說明 20150519
//修正UCOSIII支持時的2個bug：
//delay_tickspersec改為：delay_ostickspersec
//delay_intnesting改為：delay_osintnesting
////////////////////////////////////////////////////////////////////////////////// 	

void delay_init(u8 SYSCLK);
void delay_ms(u16 nms);
void delay_us(u32 nus);

#endif





























