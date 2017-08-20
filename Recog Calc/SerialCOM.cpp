#include "Windows.h"
#include "iostream"
#include"SerialCOM.h"
#define SERIAL_NAME		"COM3"
#define BAUD_RATE		115200



using namespace std;

uint8_t t;

HANDLE hCom;

Data_t uart_data_struct;
int Serial_COM_init()
{
	
	//初始化串口
	hCom = CreateFileA(SERIAL_NAME, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING,
		FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED, //0
		NULL);
	if (hCom == INVALID_HANDLE_VALUE) {
		cout << "打开COM失败!" << endl;
		//return FALSE;
		//exit(0);
	}
	DCB dcb;
	//配置串口相关参数
	{
		GetCommState(hCom, &dcb);
		dcb.BaudRate = BAUD_RATE;					//波特率的设置
		dcb.ByteSize = 8;
		dcb.fParity = 0;
		dcb.Parity = NOPARITY;
		dcb.StopBits = 0;

		SetupComm(hCom, 1024, 1024); //输入缓冲区和输出缓冲区的大小都是1024
		COMMTIMEOUTS TimeOuts; //设定读超时 
		TimeOuts.ReadIntervalTimeout = 0;
		TimeOuts.ReadTotalTimeoutMultiplier = 0;
		TimeOuts.ReadTotalTimeoutConstant = 0; //设定写超时 
		TimeOuts.WriteTotalTimeoutMultiplier = 0;
		TimeOuts.WriteTotalTimeoutConstant = 0;
		SetCommTimeouts(hCom, &TimeOuts); //设置超时

		SetCommState(hCom, &dcb);
		PurgeComm(hCom, PURGE_TXCLEAR | PURGE_RXCLEAR);


	}
	return TRUE;
}

BOOL bWriteStat=0;
OVERLAPPED m_osWrite;

/*
0:0xff
1:0xab
2-5:(float)plane_x
6-9:(float)plane_y
10-13:(float)plane_x_speed
14-17:(float)plane_y_speed
18-21:(float)relocate
22-23:(uint16_t)car_x
24-25:(uint16_t)car_y
26-27:(uint16_t)beacon_x
28-29:(uint16_t)beacon_y
30-33:(float)distance
34:(uint8_t)beacon_num
35-36: (uint16_t) beacon[0].x
37-38: (uint16_t) beacon[0].y
39-40: (uint16_t) beacon[1].x
41-42: (uint16_t) beacon[1].y
43-44: (uint16_t) beacon[2].x
45-46:(uint16_t) beacon[2].y
47-48: (uint16_t) beacon[3].x
49-50: (uint16_t) beacon[3].y
51:0xbb
52:0xaa
*/
void memory_copy(uint8_t from[], uint8_t to[], uint16_t len)
{
	while (len--) to[len] = from[len];
}
void send_once()
{
	uint8_t tx_buff[53];
	DWORD dwBytesWritten = sizeof(tx_buff);
	DWORD dwErrorFlags;
	COMSTAT ComStat;

	tx_buff[0] = 0xff;
	tx_buff[1] = 0xab;
	/*
	memcpy(&tx_buff[2], (uint8_t*)&uart_data_struct.x, 4);
	memcpy(&tx_buff[6], (uint8_t*)&uart_data_struct.y, 4);
	memcpy((uint8_t*)&uart_data_struct.x_speed, &tx_buff[10], 4);
	memcpy((uint8_t*)&uart_data_struct.y_speed, &tx_buff[14], 4);
	memcpy((uint8_t*)&uart_data_struct.relocate, &tx_buff[18], 4);
	memcpy((uint8_t*)&uart_data_struct.car_x, &tx_buff[22], 2);
	memcpy((uint8_t*)&uart_data_struct.car_y, &tx_buff[24], 2);
	memcpy((uint8_t*)&uart_data_struct.dest_x, &tx_buff[26], 2);
	memcpy((uint8_t*)&uart_data_struct.dest_y, &tx_buff[28], 2);
	memcpy((uint8_t*)&uart_data_struct.distance, &tx_buff[30], 4);
	memcpy((uint8_t*)&uart_data_struct.beacon_num, &tx_buff[34], 2);
	memcpy((uint8_t*)&uart_data_struct.beacon[0].x, &tx_buff[36], 4);
	memcpy((uint8_t*)&uart_data_struct.beacon[0].y, &tx_buff[40], 4);
	memcpy((uint8_t*)&uart_data_struct.beacon[1].x, &tx_buff[44], 4);
	memcpy((uint8_t*)&uart_data_struct.beacon[1].y, &tx_buff[48], 4);
	memcpy((uint8_t*)&uart_data_struct.beacon[2].x, &tx_buff[52], 4);
	memcpy((uint8_t*)&uart_data_struct.beacon[2].y, &tx_buff[56], 4);
	memcpy((uint8_t*)&uart_data_struct.beacon[3].x, &tx_buff[60], 4);
	memcpy((uint8_t*)&uart_data_struct.beacon[3].y, &tx_buff[64], 4);
	*/
	memcpy( &tx_buff[2],(uint8_t*)&uart_data_struct.x, 4);
	memcpy( &tx_buff[6],(uint8_t*)&uart_data_struct.y, 4);
	memcpy( &tx_buff[10],(uint8_t*)&uart_data_struct.x_speed, 4);
	memcpy( &tx_buff[14],(uint8_t*)&uart_data_struct.y_speed, 4);
	memcpy( &tx_buff[18],(uint8_t*)&uart_data_struct.relocate, 4);
	memcpy( &tx_buff[22],(uint8_t*)&uart_data_struct.car_x, 2);
	memcpy( &tx_buff[24],(uint8_t*)&uart_data_struct.car_y, 2);
	memcpy( &tx_buff[26],(uint8_t*)&uart_data_struct.dest_x, 2);
	memcpy( &tx_buff[28],(uint8_t*)&uart_data_struct.dest_y, 2);
	memcpy( &tx_buff[30],(uint8_t*)&uart_data_struct.distance, 4);
	memcpy( &tx_buff[34],(uint8_t*)&uart_data_struct.beacon_num, 1);
	memcpy( &tx_buff[35],(uint8_t*)&uart_data_struct.beacon[0].x, 2);
	memcpy( &tx_buff[37],(uint8_t*)&uart_data_struct.beacon[0].y, 2);
	memcpy( &tx_buff[39],(uint8_t*)&uart_data_struct.beacon[1].x, 2);
	memcpy( &tx_buff[41],(uint8_t*)&uart_data_struct.beacon[1].y, 2);
	memcpy( &tx_buff[43],(uint8_t*)&uart_data_struct.beacon[2].x, 2);
	memcpy( &tx_buff[45],(uint8_t*)&uart_data_struct.beacon[2].y, 2);
	memcpy( &tx_buff[47],(uint8_t*)&uart_data_struct.beacon[3].x, 2);
	memcpy( &tx_buff[49],(uint8_t*)&uart_data_struct.beacon[3].y, 2);

	tx_buff[51] = 0xbb;
	tx_buff[52] = 0xaa;
	ZeroMemory(&m_osWrite, sizeof(m_osWrite));
	if (m_osWrite.hEvent != NULL)
	{
		ResetEvent(m_osWrite.hEvent);
		m_osWrite.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
	}

	bWriteStat = WriteFile(hCom,tx_buff, dwBytesWritten, &dwBytesWritten, &m_osWrite);
}

void wait_for_send_end()
{
	if (!bWriteStat) {
		if (GetLastError() == ERROR_IO_PENDING) {
			WaitForSingleObject(m_osWrite.hEvent, INFINITE);

		}
	}
}