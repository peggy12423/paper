#ifndef HEADER_FILE_H  // �קK���ƥ]�t
#define HEADER_FILE_H

#include <iostream>
#include <time.h>
#include <math.h>
#include <fstream>
#include <queue>     

#define roundnumber 20
#define MAX_energy 6480//j (1.5V*600mA*3600sec*2 = 6480j) 2*3���q��
#define SINK_BUFFER_SIZE 10000000
#define SINKID 2000
#define SINK_X 400
#define SINK_Y 0
#define NODE_BUFFER1 100 //0~49 �@��CH����CM�� node_buffer 40Kbytes (200��) ��F�o�ӰѼ� �U����bomb�]�n��
#define NODE_BUFFER2 200 //50~100 �S�O���ǿ��

#define ProbeEnergy 0.03 //j (8*200bit*1.5V*25mA*0.5mS = 0.00375j*8 = 0.03j) !/*(�d�쪺�פ�:bit*50nj+bit*distance(m)����*100nj)*/
#define TransmitEnergy 0.00008 //0.000000001*50*200*8 j (50nj/bit by�a��) bit * 8 = byte !(�O�o�˺��)
#define AmplifierEnergy 0.00016 //100*200*0.000000001*8 j/distance^2
#define ReceiveEnergy 0.00008 //0.000000001*50*200j*8 (50nj/bit by�a��)
#define Package_size 200 //bytes 
#define node_buffer 20 //Kbytes (100��)
#define trans_dis 60 //m 80�X�G�i�H�T�w�L�Ǫ���sink
#define round_number 20

using namespace std;
struct Package{
	int src;
	int dst;
	int data;
	int time;
};

struct Sink{
	int id;//node information
	Package buffer[SINK_BUFFER_SIZE];//buffer
};

struct RREQ{
	queue<int>route;
	int hop_count;
};

double avg_t(0);
double drop(0);
double macdrop(0);
double total(0);

#endif