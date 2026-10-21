#include "x_rcc.h"		//配置时钟文件
#include "x_gpio.h"		//配置IO口文件
#include "x_global.h"	//存放全局变量
#include "x_delay.h"	//存放延时函数
#include "x_type.h"		//存放类型定义
#include "x_usart.h"	//存放串口功能文件
#include "x_timer.h"	//存放定时器功能文件
#include "x_ps2.h"		//存放PS2手柄
#include "x_w25q64.h"	//存储芯片的操作
#include "x_sensor.h"	//传感器初始化
#include "x_adc.h"		//ADC初始化
#include <stdio.h>		//标准库文件
#include <string.h>		//标准库文件
#include <math.h>		//标准库文件
#include "x_kinematics.h"	//逆运动学算法
#define pi 3.1415926

/*
	设置四个关节的长度
	单位1mm
*/

void setup_kinematics(float L0, float L1, float L2, float L3, kinematics_t *kinematics) {
	//放大10倍
	kinematics->L0 = L0*10;
	kinematics->L1 = L1*10;
	kinematics->L2 = L2*10;
	kinematics->L3 = L3*10;
}

/*
	x,y 为映射到平面的坐标
	z为距离地面的距离
	Alpha 为爪子和平面的夹角 -25~-65范围比较好
*/

int kinematics_analysis(float x, float y, float z, float Alpha, kinematics_t *kinematics) {
// alpha = 0   # 第四个舵机的仰角
// theta3 = 0  # ID3舵机角度值
// theta4 = 0  # ID2舵机角度值
// theta5 = 0  # ID14舵机角度值
// theta6 = 0  # ID0舵机角度值
	float theta3, theta4, theta5, theta6;
	float l0, l1, l2, l3;//L0 L1  L2  L3
	float aaa, bbb,ccc, zf_flag;
	
	//放大10倍
	x = x*10;
	y = y*10;
	z = z*10;
	
	
	l0 = kinematics->L0;
	l1 = kinematics->L1;
	l2 = kinematics->L2;
	l3 = kinematics->L3;
	
//	if(x == 0) {
//		theta6 = 0.0;
//		
//	} else {
//		theta6 = atan(x/y)*180.0/pi;
//	}

//优化后机械臂可以在-y轴活动，即活动范围大于180度
	 if(x == 0){
        theta6 = 0.0;
	 }
    else if(x>0 && y<0){
        theta6 = atan(x / y);
        theta6 = 180 + (theta6 * 180.0/pi);
    }else{
			  if(y>=0){
				   theta6 = atan(x / y);
           theta6 = theta6 * 180.0 / pi;
				}else{
					
			  	theta6 = atan(x / y);
          theta6 = theta6 * 180.0 / pi;
				  theta6=theta6-180;
				}
        
		}
	  y = sqrt(x*x + y*y);//#  x,y坐标的斜边 
    y = y-l3 * cos(Alpha*pi/180.0);  //求出 y总 - y3 = y2 + y1
    z = z-l0-l3*sin(Alpha*pi/180.0); //求出z1 + z2
    if(z < -l0) {
        return 1;
	}
    if(sqrt(y*y + z*z) > (l1+l2)) {
        return 2;
	}
	
  	ccc = acos(y / sqrt(y * y + z * z));
    bbb = (y*y+z*z+l1*l1-l2*l2)/(2*l1*sqrt(y*y+z*z));//余弦定理
    if(bbb > 1 || bbb < -1) {
        return 5;
	}
    if (z < 0) {
        zf_flag = -1;
	} else {
        zf_flag = 1;
	}
    theta5 = ccc * zf_flag + acos(bbb);
    theta5 = theta5 * 180.0 / pi;
    if(theta5 > 180.0 || theta5 < 0.0) {
        return 6;
	}
	
    aaa = -(y*y+z*z-l1*l1-l2*l2)/(2*l1*l2);
    if (aaa > 1 || aaa < -1) {
        return 3;
	}
    theta4 = acos(aaa); 
    theta4 = 180.0 - theta4 * 180.0 / pi ;  
    if (theta4 > 135.0 || theta4 < -135.0) {
        return 4;
	}

    theta3 = Alpha - theta5 + theta4;
    if(theta3 > 90.0 || theta3 < -90.0) {
        return 7;
	}

//  sprintf((char *)cmd_return,"%04d",(int)theta6);
//	zx_uart_send_str(cmd_return);


  kinematics->servo_angle[0] = theta6;
	kinematics->servo_angle[1] = theta5-90;
	kinematics->servo_angle[2] = theta4;
	kinematics->servo_angle[3] = theta3;    
	
	
	//换算舵机角度
	kinematics->servo_pwm[0] = (int)(1500-2000.0 * kinematics->servo_angle[0] / 270.0);
	kinematics->servo_pwm[1] = (int)(1500+2000.0 * kinematics->servo_angle[1] / 270.0);
	kinematics->servo_pwm[2] = (int)(1500+2000.0 * kinematics->servo_angle[2] / 270.0);
	kinematics->servo_pwm[3] = (int)(1500+2000.0 * kinematics->servo_angle[3] / 270.0);

	return 0;
}

