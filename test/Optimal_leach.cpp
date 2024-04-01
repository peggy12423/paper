#include <iostream>
#include <time.h>
#include <math.h>
#include <fstream>
#define MAX_energy 6480//j (1.5V*600mA*3600sec*2 = 6480j) 2*3號電池
#define SINKID 2000
#define SINK_X 400
#define SINK_Y 0
#define SINK_BUFFER_SIZE 5000000
#define NODE_BUFFER1 400 //0~49 一般CH接收CM用 node_buffer 40Kbytes (200格) 改了這個參數 下面的bomb也要改
#define NODE_BUFFER2 800 //50~100 特別的傳輸用

#define R 1 //壓縮率 設1則沒有壓縮
#define type3f 90//常規sensing frequency
#define type4f 120
#define type5f 150
#define reservation_energy_time 10000
#define CHf 100 //CH trans frequency

#define ProbeEnergy 0.03 //j (8*200bit*1.5V*25mA*0.5mS = 0.00375j*8 = 0.03j) !
/*(查到的論文:bit*50nj+bit*distance(m)平方*100nj)*/
#define TransmitEnergy 0.00008 //0.000000001*50*200*8 j (50nj/bit by冠中) bit * 8 = byte !
#define AmplifierEnergy 0.00016 //100*200*0.000000001*8 j/distance^2
#define ReceiveEnergy 0.00008 //0.000000001*50*200j*8 (50nj/bit by冠中)
#define Package_size 200 //bytes 
#define successful_rate 5 //設x 成功率就是100-x%

/*變動實驗參數設定*/
#define round_number 100
#define E_NUM 1000

using namespace std;

int S_NUM = 200; //感測器總數
struct P
{
	int src;
	int dst;
	int data;
	int time;
};
struct N
{
	int id, x, y, CH, type, region1;//region1 for 第一層grid , region2 for 第二層grid
	double random_num;
	double energy;//node information
	P receive;
	P sense;
	P buffer[NODE_BUFFER2];//buffer in sensor node
	double dtc;//dist = distance to sink
};
struct S
{
	int id;//node information
	P buffer[SINK_BUFFER_SIZE];//buffer
};
ofstream fout("100.400.800Optimal-LEACH_normal.txt");
N ns[2000];
S sink;
double avg_t, buffer_drop, mac_drop, total;
int R2, R3, R4;

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
        ns[i].type = rand() % 3 + 3;//3 4 5
        ns[i].energy = MAX_energy;
        ns[i].region1 = 1;
    }
    for (i; i < R3; i++)
    {
        ns[i].id = i;
        ns[i].x = rand() % 200 + 201;
        ns[i].y = rand() % 200 + 1;
        ns[i].type = rand() % 3 + 3;
        ns[i].energy = MAX_energy;
        ns[i].region1 = 2;
    }
    for (i; i < R4; i++)
    {
        ns[i].id = i;
        ns[i].x = rand() % 200 + 1;
        ns[i].y = rand() % 200 + 201;
        ns[i].type = rand() % 3 + 3;
        ns[i].energy = MAX_energy;
        ns[i].region1 = 3;
    }
    for (i; i < S_NUM; i++)
    {
        ns[i].id = i;
        ns[i].x = rand() % 200 + 201;
        ns[i].y = rand() % 200 + 201;
        ns[i].type = rand() % 3 + 3;
        ns[i].energy = MAX_energy;
        ns[i].region1 = 4;
    }
}

void special_node_deployed(){
	R2 = S_NUM * 0.2;
	R3 = S_NUM * 0.5;
	R4 = S_NUM * 0.6;
	int i = 0;
	  for (i; i < R2; i++)
    {
        ns[i].id = i;
        ns[i].x = rand() % 200 + 1;
        ns[i].y = rand() % 200 + 1;
        ns[i].type = rand() % 3 + 3;//3 4 5
        ns[i].energy = MAX_energy;
        ns[i].region1 = 1;
    }
    for (i; i < R3; i++)
    {
        ns[i].id = i;
        ns[i].x = rand() % 200 + 201;
        ns[i].y = rand() % 200 + 1;
        ns[i].type = rand() % 3 + 3;
        ns[i].energy = MAX_energy;
        ns[i].region1 = 2;
    }
    for (i; i < R4; i++)
    {
        ns[i].id = i;
        ns[i].x = rand() % 200 + 1;
        ns[i].y = rand() % 200 + 201;
        ns[i].type = rand() % 3 + 3;
        ns[i].energy = MAX_energy;
        ns[i].region1 = 3;
    }
    for (i; i < S_NUM; i++)
    {
        ns[i].id = i;
        ns[i].x = rand() % 200 + 201;
        ns[i].y = rand() % 200 + 201;
        ns[i].type = rand() % 3 + 3;
        ns[i].energy = MAX_energy;
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

void sink_init(){
	sink.id = SINKID;
	for (int b = 0; b < SINK_BUFFER_SIZE; b++){
		sink.buffer[b].data = -1;
		sink.buffer[b].dst = -1;
		sink.buffer[b].src = -1;
		sink.buffer[b].time = -1;
	}
}


int main()
{
	/*sensor initialization*/
	srand((unsigned)time(NULL)); //random seed
	fout << "my" << endl;
	for( S_NUM ; S_NUM <= E_NUM ; S_NUM += 100){
		avg_t = 0;
		buffer_drop = 0;
		mac_drop = 0;
		total = 0;
		cout << "sensors: " << S_NUM << endl;
		fout << endl << "------------ Sensors " << S_NUM << " ------------" << endl;
		/*sensor initialization*/
		for (int round = 0; round < round_number; round++)
		{
			cout << round+1 << endl;
			node_deployed();
			// special_node_deployed();
			packet_init();

			/*sink initialization*/
			sink_init();

            int CH_record[4][3] = { {-1,-1,-1}, {-1,-1,-1}, {-1,-1,-1}, {-1,-1,-1} };
            int start[4] = { 0, R2, R3, R4 };
            int end[4] = { R2-1, R3-1, R4-1, S_NUM-1 };
            int CH_num[4];
            for(int i = 0; i <= 3; i++){
                CH_num[i] = region_CH_num( start[i], end[i] );
                CH_selection( start[i], end[i], i, CH_record );
            }
			/*traffic start*/
			int die(0);
			int t(1);
			while (!die){
				int c = CheckEnergy();/*有一個節點沒電則等於死亡*/
				if (c < SINKID)
				{
					// fout << "dead node: " << c << endl;
					// fout << "energy: " << ns[c].energy << endl;
					// fout << "CH: " << ns[c].CH << endl;
					// fout << "region: " << ns[c].region1 << endl;
					// fout << "------------------------" << endl;
					avg_t += t;
					die = 1;
					break;
				}
				/*Data genetate , trans , receive*/
                if (t % type3f == 0){
                    for (int j = 0; j < S_NUM; j++){
                        if (ns[j].type == 3){ //CH need to sense
                            transaction(j, t, end);                          
                        }
                    }
                }
                if (t % type4f == 0){
                    for (int j = 0; j < S_NUM; j++){
                        if (ns[j].type == 4){//CH need to sense
                            transaction(j, t, end);                          
                        }
                    }
                }
                if (t % type5f == 0){
                    for (int j = 0; j < S_NUM; j++){
                        if (ns[j].type == 5){ //CH need to sense
                            transaction(j, t, end);                          
                        }
                    }
                }
				

				if (t % CHf == 0) //每一分鐘傳到sink 1次
				{
                    for(int i = 0; i <= 2; i++){
                        if( CH_record[1][i] != -1 ){
                            CH2Sink(CH_record[1][i]);
                        }
                    }
                    for(int j = 0; j <= 2; j++){
                        if( CH_record[0][j] != -1 ){
                            CHtoRegion2(CH_record[0][j]);
                        }
                    }
                    for(int j = 0; j <= 2; j++){
                        if( CH_record[2][j] != -1 ){
                            CHtoRegion2(CH_record[2][j]);
                        }
                    }
                    for(int j = 0; j <= 2; j++){
                        if( CH_record[3][j] != -1 ){
                            CHtoRegion2(CH_record[3][j]);
                        }
                    }
            		CH_Reselection(CH_record);
				}
				t++;
			}
		}
		total /= round_number;
		mac_drop /= round_number;
		buffer_drop /= round_number;
		avg_t /= round_number;
		fout << "avg_lifetime : " << avg_t << endl;
		fout << "avg_total : " << total << endl;
		fout << "avg_macdrop : " << mac_drop << endl;
		fout << "avg_drop : " << buffer_drop << endl;
		fout << "avg_PLR : " << (buffer_drop + mac_drop) / total << endl;
	}
	return 0;
}

