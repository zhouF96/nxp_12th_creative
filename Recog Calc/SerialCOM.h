#pragma once


#pragma pack(1)
typedef struct
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
}Data_t;

extern Data_t uart_data_struct;

int Serial_COM_init();
void send_once();
void wait_for_send_end();
