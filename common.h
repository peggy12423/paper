#ifndef HEADER_FILE_H  // 避免重複包含
#define HEADER_FILE_H

#include <iostream>
#include <time.h>
#include <math.h>
#include <fstream>
#include <queue>     

#define roundnumber 20
#define MAX_energy 6480//j (1.5V*600mA*3600sec*2 = 6480j) 2*3號電池
#define SINKBUFFER 10000000
#define SINKID 2000
#define SINK_X 400
#define SINK_Y 0

#define ProbeEnergy 0.03 //j (8*200bit*1.5V*25mA*0.5mS = 0.00375j*8 = 0.03j) !/*(查到的論文:bit*50nj+bit*distance(m)平方*100nj)*/
#define TransmitEnergy 0.00008 //0.000000001*50*200*8 j (50nj/bit by冠中) bit * 8 = byte !(是這樣算嗎)
#define AmplifierEnergy 0.00016 //100*200*0.000000001*8 j/distance^2
#define ReceiveEnergy 0.00008 //0.000000001*50*200j*8 (50nj/bit by冠中)
#define Package_size 200 //bytes 
#define node_buffer 20 //Kbytes (100格)
#define trans_dis 60 //m 80幾乎可以確定他傳的到sink

using namespace std;
struct Package{
	int src;
	int dst;
	int data;
	int time;
};

struct Node{
	int id, x, y, CH, type, region1;//region1 for 第一層grid , region2 for 第二層grid
	double random_num;
	double energy;//node information
	Package receive;
	Package sense;
	Package buffer[100];//buffer in sensor node
	double dtc;//dist = distance to sink
	int non; //nomber of negihbor
	int neighbor[1000]; //存取節點的鄰居
    int visited;
	queue<int>route;
};

struct Sink{
	int id;//node information
	Package buffer[SINKBUFFER];//buffer
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