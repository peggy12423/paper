#ifndef HEADER_FILE_H  // 避免重複包含
#define HEADER_FILE_H

#include <iostream>
#include <time.h>
#include <math.h>
#include <fstream>
#include <queue>
#include <list>   

#define MAX_energy 6480//1.5(V)*0.6(A)*3600(sec)*2 = 6480焦耳 2*3號電池
#define SINK_BUFFER_SIZE 10000000
#define SINKID 2000
#define SINK_X 400
#define SINK_Y 0
#define NODE_BUFFER1 100 //0~49 一般CH接收CM用 node_buffer 40Kbytes (200格) 改了這個參數 下面的bomb也要改
#define NODE_BUFFER2 200 //50~100 特別的傳輸用

#define ProbeEnergy 0.03 //8*200*1.5(V)*0.025(A)*0.0005(sec) = 0.00375*8 = 0.03焦耳 
// Etrans= 200*8*(50*0.000000001 + 100*d^2*0.000000000001) 
#define TransmitEnergy 0.00008 //200*8*(50*0.000000001)焦耳 (不含放大器所需的能量)
#define AmplifierEnergy 0.00000016 //200*8*100*0.000000000001 焦耳/公尺^2
#define ReceiveEnergy 0.00008 //200*8*50*0.000000001焦耳 
#define Package_size 200 //bytes 
#define node_buffer 20 //Kbytes (100格)
#define trans_dis 60 //公尺 80幾乎可以確定他傳的到sink
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

double avg_t(0);
double drop(0);
double macdrop(0);
double total(0);


#endif