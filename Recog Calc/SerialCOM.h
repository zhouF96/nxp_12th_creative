#include "cv.hpp"

#pragma once


#pragma pack(1)

struct Point2s
{
	unsigned short x;
	unsigned short y;
};




typedef struct flight_data_pack
{
	float x = 0;				//��ŷɻ�x����ƫ��
	float y = 0;				//��ŷɻ�Y����ƫ��
	float x_speed;
	float y_speed;

	float relocate = 0;		//��ŷɻ������Ƕ�
	unsigned short car_x;
	unsigned short car_y;
	unsigned short dest_x;
	unsigned short dest_y;
	float angle = 0;			//���С�����
	float distance = 0;	//���С������Ŀ�����

	unsigned char beacon_num = 0;
	Point2s beacon[4];
}Data_t;

extern Data_t uart_data_struct;

int Serial_COM_init();
void send_once();
void wait_for_send_end();
