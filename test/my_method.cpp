#include <iostream>
#include <time.h>
#include <math.h>
#include <fstream>
#include <queue>

#define MAX_energy 6480//j (1.5V*600mA*3600sec*2 = 6480j) 2*3號電池
#define SINKID 2000
#define SINK_X 400
#define SINK_Y 0
#define SINK_BUFFER_SIZE 5000000
#define NODE_BUFFER1 100 //0~49 一般CH接收CM用 node_buffer 40Kbytes (200格) 改了這個參數 下面的bomb也要改
#define NODE_BUFFER2 200 //50~100 特別的傳輸用

#define R 0.25 //壓縮率 設1則沒有壓縮
#define type3f 360 //常規sensing frequency
#define type4f 480
#define type5f 600
#define CHf 120 //CH trans frequency
#define freq_change_switch 0 //0關 1開 是否要使資料量突然暴增的開關
#define b_t 10800 //大T 每多少秒開一次 小T 每一次開多少秒
#define s_t 1800
#define bomb_f3 45 //爆炸sensing frequency
#define bomb_f4 60
#define bomb_f5 90

#define ProbeEnergy 0.03 //j (8*200bit*1.5V*25mA*0.5mS = 0.00375j*8 = 0.03j) !
/*(查到的論文:bit*50nj+bit*distance(m)平方*100nj)*/
#define TransmitEnergy 0.00008 //0.000000001*50*200*8 j (50nj/bit by冠中) bit * 8 = byte !(是這樣算嗎)
#define AmplifierEnergy 0.00016 //100*200*0.000000001*8 j/distance^2
#define ReceiveEnergy 0.00008 //0.000000001*50*200j*8 (50nj/bit by冠中)
#define Package_size 200 //bytes 
#define node_buffer 20 //Kbytes (100格)

/*變動實驗參數設定*/
#define round_number 10
#define E_NUM 1000

using namespace std;

int S_NUM = 200;

struct Package{
	int src;
	int dst;
	int data;
	int time;
};

struct Node{
	int id, x, y, CH, region, type;  //
    double energy;//node information
	double dist_to_sink;//dist = distance to sink
	Package receive;
	Package sense;
	Package buffer[NODE_BUFFER2];//buffer in sensor node
};

struct Sink{
	int id;//node information
	Package buffer[SINK_BUFFER_SIZE];//buffer
};


/*變動實驗參數設定*/
int S_NUM = 200; //感測器總數

double avg_time = 0;
double macdrop = 0;
double drop = 0;
double total = 0;

Node node[2000];
Sink sink;
int R2, R3, R4;

double distance(int a, int b)
{
	if (b != SINKID)
	{
		return sqrt(pow(abs(ns[a].x - ns[b].x), 2) + pow(abs(ns[a].y - ns[b].y), 2));
	}
	else //SINK
	{
		return sqrt(pow(abs(ns[a].x - SINK_X), 2) + pow(abs(ns[a].y - SINK_Y), 2));
	}
}

void node_deployed(){
	R2 = S_NUM * 0.25;
	R3 = S_NUM * 0.5;
	R4 = S_NUM * 0.75;
	
	int i = 0;
	for (i; i < R2; i++)
	{
		ns[i].id = i;
		ns[i].x = rand() % 200 + 1;
		ns[i].y = rand() % 200 + 1;
		ns[i].CH = i;
		ns[i].type = rand() % 3 + 3;//3 4 5
		ns[i].energy = MAX_energy;
		ns[i].dtc = distance(i, SINKID);/*距離區SINK*/
		ns[i].region1 = 1;
	}
	for (i; i < R3; i++)
	{
		ns[i].id = i;
		ns[i].x = rand() % 200 + 201;
		ns[i].y = rand() % 200 + 1;
		ns[i].CH = i;
		ns[i].type = rand() % 3 + 3;
		ns[i].energy = MAX_energy;
		ns[i].dtc = distance(i, SINKID); /*距離sink*/
		ns[i].region1 = 2;
	}
	for (i; i < R4; i++)
	{
		ns[i].id = i;
		ns[i].x = rand() % 200 + 1;
		ns[i].y = rand() % 200 + 201;
		ns[i].CH = i;
		ns[i].type = rand() % 3 + 3;
		ns[i].energy = MAX_energy;
		ns[i].dtc = distance(i, SINKID);/*距離區SINKID*/
		ns[i].region1 = 3;
	}
	for (i; i < S_NUM; i++)
	{
		ns[i].id = i;
		ns[i].x = rand() % 200 + 201;
		ns[i].y = rand() % 200 + 201;
		ns[i].CH = i;
		ns[i].type = rand() % 3 + 3;
		ns[i].energy = MAX_energy;
		ns[i].dtc = distance(i, SINKID);/*距離區SINKID*/
		ns[i].region1 = 4;
	}
}

void special_node_deployed(){
	R2 = S_NUM * 0.2;  //region 1
	R3 = S_NUM * 0.5;  //region 2
	R4 = S_NUM * 0.6;  //region 3 
	int i = 0;
	for (i; i < R2; i++)
	{
		ns[i].id = i;
		ns[i].x = rand() % 200 + 1;
		ns[i].y = rand() % 200 + 1;
		ns[i].CH = i;
		ns[i].type = rand() % 3 + 3;//3 4 5
		ns[i].energy = MAX_energy;
		ns[i].dtc = distance(i, SINKID);/*距離區SINK*/
		ns[i].region1 = 1;
	}
	for (i; i < R3; i++)
	{
		ns[i].id = i;
		ns[i].x = rand() % 200 + 201;
		ns[i].y = rand() % 200 + 1;
		ns[i].CH = i;
		ns[i].type = rand() % 3 + 3;
		ns[i].energy = MAX_energy;
		ns[i].dtc = distance(i, SINKID); /*距離sink*/
		ns[i].region1 = 2;
	}
	for (i; i < R4; i++)
	{
		ns[i].id = i;
		ns[i].x = rand() % 200 + 1;
		ns[i].y = rand() % 200 + 201;
		ns[i].CH = i;
		ns[i].type = rand() % 3 + 3;
		ns[i].energy = MAX_energy;
		ns[i].dtc = distance(i, SINKID);/*距離區SINKID*/
		ns[i].region1 = 3;
	}
	for (i; i < S_NUM; i++)
	{
		ns[i].id = i;
		ns[i].x = rand() % 200 + 201;
		ns[i].y = rand() % 200 + 201;
		ns[i].CH = i;
		ns[i].type = rand() % 3 + 3;
		ns[i].energy = MAX_energy;
		ns[i].dtc = distance(i, SINKID);/*距離區SINKID*/
		ns[i].region1 = 4;
	}
}

void packet_init()
{
	for (int a = 0; a < S_NUM; a++)
	{
		ns[a].receive.data = -1;
		ns[a].receive.dst = -1;
		ns[a].receive.src = -1;
		ns[a].receive.time = -1;
		ns[a].sense.data = -1;
		ns[a].sense.dst = -1;
		ns[a].sense.src = -1;
		ns[a].sense.time = -1;
		for (int b = 0; b < NODE_BUFFER2; b++)
		{
			ns[a].buffer[b].data = -1;
			ns[a].buffer[b].dst = -1;
			ns[a].buffer[b].src = -1;
			ns[a].buffer[b].time = -1;
		}
	}
}

void sink_buffer_init(){
	sink.id = SINKID;
	for (int i = 0; i < SINK_BUFFER_SIZE; i++){
		sink.buffer[i].data = -1;
		sink.buffer[i].dst = -1;
		sink.buffer[i].src = -1;
		sink.buffer[i].time = -1;
	}
}

double avg_energy(int start_index, int end_index){
    double total_energy = 0;
	int region_S_NUM = 0;
    for( int i = start_index; i <= end_index; i++){
		total_energy += node[i].energy;
		region_S_NUM++;
    }
    double avg_energy = total / region_S_NUM;
    return avg_energy;
}

double node_density(int start_index, int end_index, int area){
    int region_S_NUM = 0;
    for(int i = start_index; i <= end_index; i++){
        region_S_NUM++;
    }
    double node_density = region_S_NUM / area;
    return node_density;
}

int main(){
	ofstream fout("my_normal.txt");
	streambuf *coutbuf = cout.rdbuf();
	cout.rdbuf(fout.rdbuf());
	cout << "My method" << endl;

	srand((unsigned)time(NULL)); //random seed
	for (int round = 0; round < round_number; round++){
		cout << "----------------ROUND " << round +1 << "-----------------" <<endl;
		round_init();
		node_deployed();
		// special_node_deployed();
		
		/*initialization*/
		packet_init();
		sink_buffer_init();

		double D1 = node_density(0, R2-1, 40000);
		double D2 = node_density(R2, R3-1, 40000);
		double D3 = node_density(R3, R4-1, 40000);
		double D4 = node_density(R4, S_NUM-1, 40000);

    }
	total /= round_number;
	macdrop /= round_number;
	drop /= round_number;
	avg_time /= round_number;
	fout << "avg_lifetime : " << avg_time << endl;
	fout << "avg_total : " << total << endl;
	fout << "avg_macdrop : " << macdrop << endl;
	fout << "avg_drop : " << drop << endl;
	fout << "avg_PLR : " << (drop + macdrop) / total << endl;

	cout.rdbuf(coutbuf);
    fout.close();
	return 0;
}