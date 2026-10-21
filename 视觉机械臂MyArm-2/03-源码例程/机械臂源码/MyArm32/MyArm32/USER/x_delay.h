/*
	*	@笔者	：	星甲
	*	@日期	：	2023年02月13日
	*	@所属	：	杭州星呗科技
	*	@淘宝	：	https://xbrobot.taobao.com
*/
#ifndef __TB_DELAY_H__
#define __TB_DELAY_H__

#include "x_type.h"

#define mdelay(x) tb_delay_ms(x)

void tb_delay_init(void);
void tb_delay(u16 t);
void tb_delay_ns(u16 t);
void tb_delay_us(u16 t);
void tb_delay_ms(u16 t);




#endif

