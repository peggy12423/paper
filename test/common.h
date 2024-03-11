#ifndef HEADER_FILE_H  // �קK���ƥ]�t
#define HEADER_FILE_H

#include <iostream>
#include <time.h>
#include <math.h>
#include <fstream>
#include <queue>
#include <list>
#include <iterator>

/*�ܰʹ���ѼƳ]�w*/
#define S_NUM 100 //�P�����`��
#define compression_rate 0.25 //���Y�v �]1�h�S�����Y
#define CH_transmit 120 //CH trans frequency
#define SensingRate_type1f 360
#define SensingRate_type2f 480
#define SensingRate_type3f 720

/*�T�w����Ѽ�*/
#define roundnumber 5
#define MAX_energy 6480//1.5(V)*0.6(A)*3600(sec)*2 = 6480�J�� 2*3���q��
#define SINK_BUFFER_SIZE 10000000
#define SINKID 2000
#define SINK_X 400
#define SINK_Y 0
#define NODE_BUFFER1 100 //0~49 �@��CH����CM�� node_buffer 40Kbytes (200��) ��F�o�ӰѼ� �U����bomb�]�n��
#define NODE_BUFFER2 200 //50~100 �S�O���ǿ��

#define ProbeEnergy 0.03 //8*200*1.5(V)*0.025(A)*0.0005(sec) = 0.00375*8 = 0.03�J�� 
// Etrans= 200*8*(50*0.000000001 + 100*d^2*0.000000000001)
#define TransmitEnergy 0.00008 //200*8*(50*0.000000001)�J�� (���t��j���һݪ���q)
#define AmplifierEnergy 0.00000016 //200*8*100*0.000000000001 �J��/����^2
#define ReceiveEnergy 0.00008 //200*8*50*0.000000001�J�� 
#define Package_size 200 //bytes 
#define node_buffer 20 //Kbytes (100��)
#define trans_dis 60 //���� 80�X�G�i�H�T�w�L�Ǫ���sink
#define round_number 5

using namespace std;
struct Package{
	int src;
	int dst;
	int data;
	int time;
};

struct Sink{
	int id = SINKID;//node information
	Package buffer[SINK_BUFFER_SIZE];//buffer
};

struct Node{
	int id, x, y, CH, region, type;  //
    double energy = 0.0;//node information
	double dist_to_sink = 0.0;//dist = distance to sink
	Package receive;
	Package sense;
	Package buffer[NODE_BUFFER2];//buffer in sensor node
};

double avg_time = 0;
double drop = 0;
double macdrop = 0;
double total = 0;


#endif