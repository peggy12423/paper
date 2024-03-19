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
#define type5f 720
#define CHf 120 //CH trans frequency
#define reservation_energy_time 10000

#define ProbeEnergy 0.03 //j (8*200bit*1.5V*25mA*0.5mS = 0.00375j*8 = 0.03j) !
/*(查到的論文:bit*50nj+bit*distance(m)平方*100nj)*/
#define TransmitEnergy 0.00008 //0.000000001*50*200*8 j (50nj/bit by冠中) bit * 8 = byte !(是這樣算嗎)
#define AmplifierEnergy 0.00016 //100*200*0.000000001*8 j/distance^2
#define ReceiveEnergy 0.00008 //0.000000001*50*200j*8 (50nj/bit by冠中)
#define Package_size 200 //bytes 
#define node_buffer 20 //Kbytes (100格)

/*學長做出最好的權重*/
#define Alpha 0.2
#define Beta 0.8

/*變動實驗參數設定*/
#define round_number 10
#define E_NUM 1000

//實驗一可更改
#define density_th1 1.75
#define density_th2 2.5


using namespace std;

int S_NUM = 200;

struct Center{
	double x, y;
};

struct Package{
	int src;
	int dst;
	int data;
	int time;
};

struct Node{
	int id, x, y, CH, type, rate, region1, region2;
    double energy;//ns information
	double dtc;    //節點到中心的距離
	Package receive;
	Package sense;
	Package buffer[NODE_BUFFER2];//buffer in sensor ns
};

struct Sink{
	int id;//ns information
	Package buffer[SINK_BUFFER_SIZE];//buffer
};

double avg_time = 0;
double macdrop = 0;
double drop = 0;
double total = 0;

Node ns[2000];
Sink sink;
int R2, R3, R4;
double high_density_th1 = (S_NUM / 160000) * density_th1;
double high_density_th2 = (S_NUM / 160000) * density_th2;

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

Center find_center(int sIndex, int eIndex){
	Center region_center;
	region_center.x = 0.0;
	region_center.y = 0.0;
	for( int i = sIndex ; i <= eIndex ; i++){
		region_center.x += node[i].x;
		region_center.y += node[i].y;
	}
	int cluster_S_NUM = eIndex - sIndex + 1;
	region_center.x /= cluster_S_NUM;
	region_center.y /= cluster_S_NUM;
	return region_center;   // 返回中心點的座標
}

void set_distance_to_center( Center region_center, int sIndex, int eIndex){
	int dis_x = abs(node[i].x - region_center.x);
	int dis_y = abs(node[i].y - region_center.y);
	for( int i = sIndex; i <= eIndex; i++){
		node[i].dtc = sqrt( pow(dis_x, 2) + pow(dis_y, 2) );
	}
}

void set_data_rate(){
	int i = 0;
	for( i; i < S_NUM; i++){
		if(ns[i].type == 1){
			ns[i].rate = type3f;
			continue;
		}
		else if(ns[i].type == 2){
			ns[i].rate = type4f;
			continue;
		}
		else{
			ns[i].rate = type5f;
			continue;
		}
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
		ns[i].CH2 = -1;
		ns[i].CH3 = -1;
		ns[i].CH4 = -1;
		ns[i].type = rand() % 3 + 3;
		ns[i].energy = MAX_energy;
		ns[i].dtc = distance(i, SINKID);/*距離區SINK*/
		ns[i].region1 = 1;
		if( i == R2-1 ){
			Center center1 = find_center(0, R2-1);
			set_distance_to_center(center1, 0, R2-1);
		}
	}
	for (i; i < R3; i++)
	{
		ns[i].id = i;
		ns[i].x = rand() % 200 + 201;
		ns[i].y = rand() % 200 + 1;
		ns[i].CH = i;
		ns[i].CH2 = -1;
		ns[i].CH3 = -1;
		ns[i].CH4 = -1;
		ns[i].type = rand() % 3 + 3;
		ns[i].energy = MAX_energy;
		ns[i].region1 = 2;
		if( i == R3-1 ){
			Center center2 = find_center(R2, R3-1);
			set_distance_to_center(center2, R2, R3-1);
		}
	}
	for (i; i < R4; i++)
	{
		ns[i].id = i;
		ns[i].x = rand() % 200 + 1;
		ns[i].y = rand() % 200 + 201;
		ns[i].CH = i;
		ns[i].CH2 = -1;
		ns[i].CH3 = -1;
		ns[i].CH4 = -1;
		ns[i].type = rand() % 3 + 3;
		ns[i].energy = MAX_energy;
		ns[i].region1 = 3;
		if( i == R4-1 ){
			Center center3 = find_center(R3, R4-1);
			set_distance_to_center(center3, R3, R4-1);
		}
	}
	for (i; i < S_NUM; i++)
	{
		ns[i].id = i;
		ns[i].x = rand() % 200 + 201;
		ns[i].y = rand() % 200 + 201;
		ns[i].CH = i;
		ns[i].CH2 = -1;
		ns[i].CH3 = -1;
		ns[i].CH4 = -1;
		ns[i].type = rand() % 3 + 3;
		ns[i].energy = MAX_energy;
		ns[i].region1 = 4;
		if( i == S_NUM-1 ){
			Center center4 = find_center(R4, S_NUM-1);
			set_distance_to_center(center4, R4, S_NUM-1);
		}
	}
	set_data_rate();
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
	for (int i = 0; i < SINK_BUFFER_SIZE; i++){
		sink.buffer[i].data = -1;
		sink.buffer[i].dst = -1;
		sink.buffer[i].src = -1;
		sink.buffer[i].time = -1;
	}
}

double Max_energy(int sIndex, int eIndex){
	double max_E = 0;
	for(int i = sIndex; i <= eIndex; i++){
		if( max_E < ns[i].energy ){
			max_E = ns[i].energy;
		}
	}
	return max_E;
}

double Max_distance(int sIndex, int eIndex){
	double max_D = 0;
	for( int i = sIndex; i <= eIndex; i++){
		if( max_D < ns[i].dtc ){
			max_D = ns[i].dtc;
		}
	}
	return max_D;
}

double standard(double j_dtc, double j_RE, double energy_standard, double distance_standard){    //考慮i->j->Sink的距離/i->鄰近區域CH->Sink的距離 和 j的剩餘能量/鄰近區域節點最大剩餘能量
	double standard = Alpha*(1 - j_dtc / distance_standard) + Beta*(j_RE / energy_standard); //larger is better
	return standard;
}

double node_density(int sIndex, int eIndex, int area){
    int region_S_NUM = eIndex - sIndex + 1;
    double node_density = region_S_NUM / area;
    return node_density;
}

int region_CH_num(int sIndex, int eIndex){
	double region_density = node_density(sIndex, eIndex, 40000);
	int CH_num = 1;
	if( region_density >= high_density_th2 ){
		CH_num = 3;
	}
	else if( region_density >= high_density_th1 ){
		CH_num = 2;
	}
	return CH_num;
}

void CH_selection(int sIndex, int eIndex){
	int CH_num = region_CH_num(sIndex, eIndex);
	double Energy = Max_energy(sIndex, eIndex);
	double Distance = Max_distance(sIndex, eIndex);
	int start = sIndex;
	int end = eIndex;
	int CH = sIndex;
	int sCH = -1, tCH = -1;
	double d = distance(sIndex, ns[sIndex].CH);
	double MAX_re_energy = ns[sIndex].energy - (floor(reservation_energy_time / ns[sIndex].rate )*(ProbeEnergy + (TransmitEnergy + d*d*AmplifierEnergy)));
	double MAX_standard = standard(ns[sIndex].dtc, MAX_re_energy, Energy, Distance);
	sIndex += 1;
	double standard2, standard3;
	if( CH_num == 2 ){
		sCH = sIndex+1;
		double d2 = distance(sIndex+1, ns[sIndex+1].CH);
		double reserve_energy2 = ns[sIndex+1].energy - (floor(reservation_energy_time / ns[sIndex+1].rate )*(ProbeEnergy + (TransmitEnergy + d2*d2*AmplifierEnergy)));
		standard2 = standard(ns[sIndex+1].dtc, reserve_energy2, Energy, Distance);
		sIndex += 1;
	}
	else if( CH_num == 3 ){
		double d2 = distance(sIndex+1, ns[sIndex+1].CH);
		double reserve_energy2 = ns[sIndex+1].energy - (floor(reservation_energy_time / ns[sIndex+1].rate )*(ProbeEnergy + (TransmitEnergy + d2*d2*AmplifierEnergy)));
		standard2 = standard(ns[sIndex+1].dtc, reserve_energy2, Energy, Distance);
		
		double d3 = distance(sIndex+2, ns[sIndex+2].CH);
		double reserve_energy3 = ns[sIndex+2].energy - (floor(reservation_energy_time / ns[sIndex+2].rate )*(ProbeEnergy + (TransmitEnergy + d3*d3*AmplifierEnergy)));
		standard3 = standard(ns[sIndex+2].dtc, reserve_energy3, Energy, Distance);
		sIndex += 2;
	}

	for( int i = sIndex; i <= eIndex; i++){
		d = distance(i, ns[i].CH);
		double current_re_energy = ns[i].energy - (floor(reservation_energy_time / ns[i].rate )*(ProbeEnergy + (TransmitEnergy + d*d*AmplifierEnergy)));
		double current_standard = standard(ns[i].dtc, current_re_energy, Energy, Distance);
		if( MAX_standard < current_standard ){
			MAX_standard = current_standard;
			CH = ns[i].id;
			continue;
		}
		else if( CH_num > 1 && standard2 < current_standard){  //有兩個或三個CH時，且當前標準介於最大與第二標準之間
			standard2 = current_standard;
			sCH = ns[i].id;
			continue;
		}
		else if( CH_num > 2 && standard3 < current_standard){   //有三個CH時，且當前標準介於第二與第三標準之間
			standard3 = current_standard;
			tCH = ns[i].id;
			continue;
		}
	}

	for( int j = start; j <= end; j++){   //更改區域內所有節點的CH為選出的CH
		if( sCH != -1 && CH_num == 2){
			double dist1 = distance(j, CH);
			double dist2 = distance(j, sCH);
			if( dist1 <= dist2){
				ns[j].CH = CH;
				continue;
			}
			else{
				ns[j].CH = sCH;
				continue;
			}
		}
		else if( sCH != -1 && tCH != -1 && CH_num == 3){
			double dist1 = distance(j, CH);
			double dist2 = distance(j, sCH);
			double dist3 = distance(j, tCH);
			if( dist1 <= dist2 && dist1 <= dist3 ){
				ns[j].CH = CH;
				continue;
			}
			else if( dist2 <= dist1 && dist2 <= dist3){
				ns[j].CH = sCH;
				continue;
			}
			else{
				ns[j].CH = tCH;
				continue;
			}
		}
		else{
			ns[j].CH = CH;
		}
	}
}

double avg_energy(int sIndex, int eIndex){
    double total_energy = 0;
	int region_S_NUM = 0;
    for( int i = sIndex; i <= eIndex; i++){
		total_energy += ns[i].energy;
		region_S_NUM++;
    }
    double avg_energy = total / region_S_NUM;
    return avg_energy;
}

int CheckEnergy()
{
	for (int b = 0; b < S_NUM; b++)
	{
		//fout << "node " << b << "'s energy = " << ns[b].energy << endl;
		if (ns[b].energy <= 0)
		{
			return b;
			break;
		}
	}
	return SINKID;
}

int main(){
	ofstream fout("my_normal.txt");
	streambuf *coutbuf = cout.rdbuf();
	cout.rdbuf(fout.rdbuf());
	cout << "My method" << endl;

	srand((unsigned)time(NULL)); //random seed
	for (int round = 0; round < round_number; round++){
		cout << "----------------ROUND " << round +1 << "-----------------" <<endl;
		node_deployed();
		// special_node_deployed();
		
		/*initialization*/
		packet_init();
		sink_init();

   		/*firts CH selection*/
		CH_selection(0, R2-1);
		CH_selection(R2, R3-1);
		CH_selection(R3, R4-1);
		CH_selection(R4, S_NUM-1);

		/*traffic start*/
		int die = 0;
		int t = 1;
		while(!die){
			int dead_node = CheckEnergy();
			if( dead_node < SINKID ){    //有node死掉
				avg_time += t;
				die = 1;
				break;
			}
			else{   //沒有node死掉

			}
		}

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