#pragma once


#pragma pack(1)
typedef struct
{
	float x = 0;				//存放飞机x方向偏移
	float y = 0;				//存放飞机Y方向偏移
	float x_speed;
	float y_speed;

	float relocate = 0;		//存放飞机自旋角度
	unsigned short car_x;
	unsigned short car_y;
	unsigned short dest_x;
	unsigned short dest_y;
	float angle = 0;			//存放小车打角
	float distance = 0;	//存放小车距离目标距离
}Data_t;

extern Data_t uart_data_struct;

int Serial_COM_init();
void send_once();
void wait_for_send_end();
