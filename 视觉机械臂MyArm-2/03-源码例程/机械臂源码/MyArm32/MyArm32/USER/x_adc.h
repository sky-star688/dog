/*
	*	@笔者	：	星甲
	*	@日期	：	2023年02月13日
	*	@所属	：	杭州星呗科技
	*	@淘宝	：	https://xbrobot.taobao.com
*/
#ifndef __ADC_H__
#define __ADC_H__

#include "x_main.h"
#include "x_type.h"
#include "stm32f10x_conf.h"

#define ADC_NUM 1


void ADC_init(void);
void ADC1_GPIO_Configuration(void);
void ADC1_Configuration(void);
void DMA_Configuration(void);

extern u16 ADC_ConvertedValue[ADC_NUM];

#endif
