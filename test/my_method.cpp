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
#define successful_rate 5 //設x 成功率就是100-x%

/*學長做出最好的權重*/
#define Alpha 0.2
#define Beta 0.8

/*變動實驗參數設定*/
#define round_number 10
#define E_NUM 1000

//實驗一可更改
#define density_th1 1.75
#define density_th2 2.5

//實驗二可更改
#define ch_energy_th 5


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
	int isCH_switch;
    double energy;//ns information
	double dtc;    //節點到中心的距離
	Package sense;
	Package receive;
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
int CH_record[4][3] = { {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0} };  //紀錄每個region的CH狀況
int CHtoSink_count[4][3] = { {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0} };  //紀錄當前CH的剩餘能量

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

double find_avg_energy(int s, int e, int rnum)
{
	double avg_energy(0.0);
	for (int i = s; i <= e; i++)
	{
		avg_energy += ns[i].energy;
	}
	avg_energy /= rnum;
	return avg_energy;
}

Center find_center(int sIndex, int eIndex){
	Center region_center;
	region_center.x = 0.0;
	region_center.y = 0.0;
	for( int i = sIndex ; i <= eIndex ; i++){
		region_center.x += ns[i].x;
		region_center.y += ns[i].y;
	}
	int cluster_S_NUM = eIndex - sIndex + 1;
	region_center.x /= cluster_S_NUM;
	region_center.y /= cluster_S_NUM;
	return region_center;   // 返回中心點的座標
}

void set_distance_to_center( Center region_center, int sIndex, int eIndex){
	int dis_x, dis_y;
	for( int i = sIndex; i <= eIndex; i++){
		dis_x = abs(ns[i].x - region_center.x);
		dis_y = abs(ns[i].y - region_center.y);
		ns[i].dtc = sqrt( pow(dis_x, 2) + pow(dis_y, 2) );
	}
}

void set_data_rate(){
	for( int i = 0; i < S_NUM; i++){
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
	for (i; i < R2; i++)   //Region 1
	{
		ns[i].id = i;
		ns[i].x = rand() % 200 + 1;
		ns[i].y = rand() % 200 + 201;
		ns[i].CH = i;
		ns[i].isCH_switch = 0;
		ns[i].type = rand() % 3 + 3;
		ns[i].energy = MAX_energy;
		ns[i].region1 = 1;
		if( i == R2-1 ){
			Center center1 = find_center(0, R2-1);
			set_distance_to_center(center1, 0, R2-1);
		}
	}
	for (i; i < R3; i++)   //Region 2
	{
		ns[i].id = i;
		ns[i].x = rand() % 200 + 201;
		ns[i].y = rand() % 200 + 201;
		ns[i].CH = i;
		ns[i].isCH_switch = 0;
		ns[i].type = rand() % 3 + 3;
		ns[i].energy = MAX_energy;
		ns[i].region1 = 2;
		if( i == R3-1 ){
			Center center2 = find_center(R2, R3-1);
			set_distance_to_center(center2, R2, R3-1);
		}
	}
	for (i; i < R4; i++)   //Region 3
	{
		ns[i].id = i;
		ns[i].x = rand() % 200 + 1;
		ns[i].y = rand() % 200 + 1;
		ns[i].CH = i;
		ns[i].isCH_switch = 0;
		ns[i].type = rand() % 3 + 3;
		ns[i].energy = MAX_energy;
		ns[i].region1 = 3;
		if( i == R4-1 ){
			Center center3 = find_center(R3, R4-1);
			set_distance_to_center(center3, R3, R4-1);
		}
	}
	for (i; i < S_NUM; i++)   //Region 4
	{
		ns[i].id = i;
		ns[i].x = rand() % 200 + 201;
		ns[i].y = rand() % 200 + 1;
		ns[i].CH = i;
		ns[i].isCH_switch = 0;
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

void CH_selection(int sIndex, int eIndex, int region){
	int CH_num = region_CH_num(sIndex, eIndex);
	double Energy = Max_energy(sIndex, eIndex);
	double Distance = Max_distance(sIndex, eIndex);
	int start = sIndex;
	int end = eIndex;
	int CH = sIndex;
	ns[CH].isCH_switch = 1;   //開關-1代表索引CH為CH
	int sCH = -1, tCH = -1;
	double d = distance(sIndex, ns[sIndex].CH);
	double reserve_energy = ns[sIndex].energy - (floor(reservation_energy_time / ns[sIndex].rate )*(ProbeEnergy + (TransmitEnergy + d*d*AmplifierEnergy)));
	double Max_standard = Alpha*(1 - ns[sIndex].dtc / Distance) + Beta*(reserve_energy / Energy);
	sIndex += 1;
	double standard2, standard3;
	if( CH_num == 2 ){
		sCH = sIndex;
		ns[sCH].isCH_switch = 1;
		double d2 = distance(sIndex, ns[sIndex].CH);
		reserve_energy = ns[sIndex].energy - (floor(reservation_energy_time / ns[sIndex].rate )*(ProbeEnergy + (TransmitEnergy + d2*d2*AmplifierEnergy)));
		standard2 = Alpha*(1 - ns[sIndex].dtc / Distance) + Beta*(reserve_energy / Energy);
		sIndex += 1;
	}
	else if( CH_num == 3 ){
		sCH = sIndex;
		ns[sCH].isCH_switch = 1;
		double d2 = distance(sIndex, ns[sIndex].CH);
		reserve_energy = ns[sIndex].energy - (floor(reservation_energy_time / ns[sIndex].rate )*(ProbeEnergy + (TransmitEnergy + d2*d2*AmplifierEnergy)));
		standard2 = Alpha*(1 - ns[sIndex].dtc / Distance) + Beta*(reserve_energy / Energy);
		sIndex += 1;
		
		tCH = sIndex;
		ns[tCH].isCH_switch = 1;
		double d3 = distance(sIndex, ns[sIndex].CH);
		reserve_energy = ns[sIndex].energy - (floor(reservation_energy_time / ns[sIndex].rate )*(ProbeEnergy + (TransmitEnergy + d3*d3*AmplifierEnergy)));
		standard3 = Alpha*(1 - ns[sIndex].dtc / Distance) + Beta*(reserve_energy / Energy);
		sIndex += 1;
	}

	for( int i = sIndex; i <= eIndex; i++){
		d = distance(i, ns[i].CH);
		reserve_energy = ns[i].energy - (floor(reservation_energy_time / ns[i].rate )*(ProbeEnergy + (TransmitEnergy + d*d*AmplifierEnergy)));
		double current_standard = standard(ns[i].dtc, reserve_energy, Energy, Distance);
		if( Max_standard < current_standard ){
			ns[i].isCH_switch = 1;
			if( sCH != -1 && tCH != -1){    // CH_num = 3
				ns[tCH].isCH_switch = 0;    //原本的tCH變成非CH
				standard3 = standard2;
				standard2 = Max_standard;
				tCH = sCH;
				sCH = CH;
			}
			else if( sCH != -1 && tCH == -1){    // CH_num = 2
				ns[sCH].isCH_switch = 0;
				standard2 = Max_standard;
				sCH = CH;
			}
			else{         //CH_num = 1
				ns[CH].isCH_switch = 0;
			}
			Max_standard = current_standard;
			CH = ns[i].id;
			continue;
		}
		else if( CH_num > 1 && standard2 < current_standard){  //有兩個或三個CH時，且當前標準介於最大與第二標準之間
			ns[i].isCH_switch = 1;
			if( sCH != -1 && tCH != -1 ){   // CH_num =3
				ns[tCH].isCH_switch = 0;
				standard3 = standard2;
				tCH = sCH;
			}
			else if( sCH != -1 && tCH == -1){   //CH_num = 2
				ns[sCH].isCH_switch = 0;
			}
			standard2 = current_standard;
			sCH = ns[i].id;
			continue;
		}
		else if( CH_num > 2 && standard3 < current_standard){   //有三個CH時，且當前標準介於第二與第三標準之間
			ns[tCH].isCH_switch = 0;
			ns[i].isCH_switch = 1;
			standard3 = current_standard;
			tCH = ns[i].id;
			continue;
		}
	}

	CH_record[region-1][0] = CH;
	CH_record[region-1][1] = sCH;
	CH_record[region-1][2] = tCH;
	CH_energy[region-1][0] = ns[CH].energy;
	CH_energy[region-1][1] = ns[sCH].energy;
	CH_energy[region-1][2] = ns[tCH].energy;

	for( int j = start; j <= end; j++){   //依照到CH的距離，更改區域內所有節點的CH為選出的CH
		if( sCH != -1 && CH_num == 2){  //CH_num = 2
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
		else if( sCH != -1 && tCH != -1 && CH_num == 3){   //CH_num = 3
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

// double avg_energy(int sIndex, int eIndex){
//     double total_energy = 0;
// 	int region_S_NUM = 0;
//     for( int i = sIndex; i <= eIndex; i++){
// 		total_energy += ns[i].energy;
// 		region_S_NUM++;
//     }
//     double avg_energy = total / region_S_NUM;
//     return avg_energy;
// }

int CheckEnergy()
{
	for (int b = 0; b < S_NUM; b++)
	{
		//fout << "ns " << b << "'s energy = " << ns[b].energy << endl;
		if (ns[b].energy <= 0)
		{
			return b;
			break;
		}
	}
	return SINKID;
}

void Packet_Generate(int now, int t) //generate packet 有能耗
{
	total++;
	ns[now].sense.src = ns[now].id;
	ns[now].sense.dst = ns[now].CH;
	ns[now].sense.data = ns[now].type;
	ns[now].sense.time = t;
	ns[now].energy -= ProbeEnergy;
	//fout << "ns : " << now << "能量減少 "<< ProbeEnergy <<" ,因為產生感測封包 "<< endl;
}

void Packet_Dliver(int sender, int CH) // 有能耗
{
	int rate = rand() % 100 + 1;
	if (rate > successful_rate || sender == CH)  //送到CH因此更改CH內receive內容
	{
		ns[CH].receive.dst = ns[sender].sense.dst;
		ns[CH].receive.src = ns[sender].sense.src;
		ns[CH].receive.data = ns[sender].sense.data;
		ns[CH].receive.time = ns[sender].sense.time;
	}
	else   //CH沒收到因此改為-1
	{
		macdrop++;
		ns[CH].receive.dst = -1;
		ns[CH].receive.src = -1;
		ns[CH].receive.data = -1;
		ns[CH].receive.time = -1;
	}
	double d = distance(sender, CH);
	if (sender != CH) //CH自己給自己不用扣能量
	{
		ns[sender].energy -= TransmitEnergy + d*d*AmplifierEnergy;
		// cons[ns[sender].region1 - 1] += TransmitEnergy + d*d*AmplifierEnergy;
		// trans_time[ns[sender].region1 - 1] += 1;
		//fout << "node : " << sender << "能量減少 "<< TransmitEnergy + d*d*AmplifierEnergy <<" ,因為傳送給節點 " << CH << " 感測封包" << endl;
	} //1個封包
}

void Packet_Receive(int CH) //buffer滿了要變成priority queue 有能耗
{
	if (ns[CH].receive.src != CH) //不是來自自己的才要扣能量
	{
		ns[CH].energy -= ReceiveEnergy;
	} //drop還算是有收
	int full(1);
	for (int a = 0; a < NODE_BUFFER1; a++) //buffer is not full 50 for self-area-sense 50 for other CH 
	{
		if (ns[CH].buffer[a].data == -1)     //CH的receive封包放入buffer
		{
			ns[CH].buffer[a].dst = ns[CH].receive.dst;
			ns[CH].buffer[a].src = ns[CH].receive.src;
			ns[CH].buffer[a].data = ns[CH].receive.data;
			ns[CH].buffer[a].time = ns[CH].receive.time;
			full = 0;
			break;
		}
	}
	if (full == 1 && ns[CH].receive.data != -1)    //buffer滿了且receive的封包不是-1
	{
		drop++;
	}
	ns[CH].receive.dst = -1;
	ns[CH].receive.src = -1;
	ns[CH].receive.data = -1;
	ns[CH].receive.time = -1;
}

void transaction(int trans_node, int t){
	Packet_Generate(trans_node, t);
	Packet_Dliver(trans_node, ns[trans_node].CH);
	Packet_Receive(ns[trans_node].CH);
}

void clean(int CH, int start, int end)
{
	for (start; start < end; start++)
	{
		ns[CH].buffer[start].data = -1;
		ns[CH].buffer[start].dst = -1;
		ns[CH].buffer[start].src = -1;
		ns[CH].buffer[start].time = -1;
	}
}

void CH_to_Sink(int CH){    //CH4到Sink
	int start = 0;    
	for (int b = 0; b < SINK_BUFFER_SIZE; b++)	{
		if (sink.buffer[b].data == -1){   //sink的buffer從哪邊開始是空的
			start = b;
			break;
		}
	}
	if (ns[CH].buffer[NODE_BUFFER1].data != -1){   //幫別的CH傳
		double rate = 0;
		for (int b = NODE_BUFFER1; b < NODE_BUFFER2; b++){    //別區送來的封包存在100~199
			if (ns[CH].buffer[b].data == -1){    //有空的就不用繼續了
				break;
			}
			int d = rand() % 100 + 1;
			if (d > successful_rate){    //CH2送到Sink
				rate = b - NODE_BUFFER1 + 1;
				sink.buffer[start].data = ns[CH].buffer[b].data;
				sink.buffer[start].dst = ns[CH].buffer[b].dst;
				sink.buffer[start].src = ns[CH].buffer[b].src;
				sink.buffer[start].time = ns[CH].buffer[b].time;
				start++;
			}
			else{
				macdrop++;
			}
		}
		//fout << "sink收到" << rate << "個" << endl;
		rate = ceil(rate * R);
		//fout << rate << endl;
		ns[CH].energy -= (TransmitEnergy + pow(distance(CH, SINKID), 2)*AmplifierEnergy)*rate; //將data合併之後一次傳送 所以耗能這樣算(合併未做) 因為可知合併的size必在packet的大小之中
																							   //fout << "幫別人成功,我的能量只剩" << ns[CH].energy << endl;
																							   //fout << "node : " << CH << "能量減少 "<< (TransmitEnergy + pow(distance(CH, SINKID), 2)*AmplifierEnergy)*rate <<" ,因為幫別人傳給sink" << endl;
		clean(CH, NODE_BUFFER1, NODE_BUFFER2); /*傳完之後刪除掉*/
	}
	else      //傳自己區的感測資料
	{
		double rate(0);/*壓縮率0.25*/
		for (int b = 0; b < NODE_BUFFER1; b++)
		{
			if (ns[CH].buffer[b].data == -1) //有空的就不用繼續了
			{
				break;
			}
			int d = rand() % 100 + 1; /*?????????????????????*/
			if (d > successful_rate)
			{
				rate = b + 1;
				sink.buffer[start].data = ns[CH].buffer[b].data;
				sink.buffer[start].dst = ns[CH].buffer[b].dst;
				sink.buffer[start].src = ns[CH].buffer[b].src;
				sink.buffer[start].time = ns[CH].buffer[b].time;
				start++;
			}
			else
			{
				macdrop++;
			}
		}
		//fout << "sink收到" << rate << "個" << endl;
		rate = ceil(rate * R);
		//fout << rate << endl;
		ns[CH].energy -= (TransmitEnergy + pow(distance(CH, SINKID), 2)*AmplifierEnergy)*rate; //將data合併之後一次傳送 所以耗能這樣算(合併未做)
																							   //fout << "自己傳,我的能量只剩" << ns[CH].energy << endl;
																							   //fout << "node : " << CH << "能量減少 "<< (TransmitEnergy + pow(distance(CH, SINKID), 2)*AmplifierEnergy)*rate <<" ,因為幫自己傳給sink" << endl;
		clean(CH, 0, NODE_BUFFER1); /*傳完之後刪除掉*/
	}
}

void CH_to_CH4(int CH) //除了2區以外的區域都丟到2區裡面能量最高的 有能耗
{
	/*取CH到2區+2區到sink的距離相加與其剩餘能量值做加權*/
	int dst = CH_record[3][0];
	double rate = 0;/*壓縮率0.25*/
	for (int b = 0; b < NODE_BUFFER1; b++){
		if (ns[CH].buffer[b].data == -1){   //有空的就不用繼續了
			break;
		}
		int d = rand() % 100 + 1; 
		if (d > successful_rate){   //其他區域送來的存在buffer 100~199
			rate = b + 1;
			ns[dst].buffer[NODE_BUFFER1 + b].data = ns[CH].buffer[b].data;
			ns[dst].buffer[NODE_BUFFER1 + b].dst = ns[CH].buffer[b].dst;
			ns[dst].buffer[NODE_BUFFER1 + b].src = ns[CH].buffer[b].src;
			ns[dst].buffer[NODE_BUFFER1 + b].time = ns[CH].buffer[b].time;
		}
		else
		{
			macdrop++;
		}
	}
	clean(CH, 0, NODE_BUFFER1);
	rate = ceil(rate * R);
	ns[CH].energy -= (TransmitEnergy + pow(distance(CH, dst), 2)*AmplifierEnergy)*rate; //將data合併之後一次傳送 所以耗能這樣算(合併未做)
																						  //fout << "node : " << CH << "能量減少 "<<(TransmitEnergy + pow(distance(CH1, dst), 2)*AmplifierEnergy)*rate<<", 因為傳輸給區域2" << endl;
	ns[dst].energy -= (ReceiveEnergy)*rate;
	CH_to_Sink(dst);
}

void Reselection_judge(int sIndex, int eIndex, int rnum, int region)
{
	double avg_energy = find_avg_energy(sIndex, eIndex, rnum);
	if (((avg_energy - ns[ns[sIndex].CH].energy) / avg_energy) >= 0.15) //這個值不一定大於0 , CH的花費不一定比周圍高 ! 因為資料壓縮的關係
	{
		CH_selection(sIndex, eIndex, region);
	}
}

void CH_Reselection()
{
	int R_NUM = S_NUM * 0.25;
	Reselection_judge(0, R2 - 1, R_NUM, 1);
	Reselection_judge(R2, R3 - 1, R_NUM, 2);
	Reselection_judge(R3, R4 - 1, R_NUM, 3);
	Reselection_judge(R4, S_NUM - 1, R_NUM, 4);
}

int borrow_ch(int ch, double d){
	double reserve_energy = ns[ch].energy - (floor(reservation_energy_time / ns[ch].rate)*(ProbeEnergy + TransmitEnergy + d*d*AmplifierEnergy));
	double ch_standard = standard(ns[ch].dtc, reserve_energy, )
}

void toSink_count(){
	/*計算各CH還可直接傳到Sink的次數*/
	for(int i = 0; i < 4 ; i++ ){   //依序查看各CH是否energy小於閥值
		for(int j = 0; j < 3; j++){
			int ch = CH_record[i][j];
			double d = distance(ch, SINKID); 
			double CHtoSink_energy = ProbeEnergy + TransmitEnergy + d*d*AmplifierEnergy;
			CHtoSink_count[i][j] = static_cast<int>( ns[ch].energy / CHtoSink_energy );   //計算目前CH還能傳送的次數 
		}
	}
}

int ch4_reselection(){
	double Energy = Max_energy(R4, S_NUM-1);
	double Distance = Max_distance(R4, S_NUM-1);
	int start = R4;
	int end = S_NUM-1;
	int CH = R4;
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
		CH_selection(0, R2-1, 1);
		CH_selection(R2, R3-1, 2);
		CH_selection(R3, R4-1, 3);
		CH_selection(R4, S_NUM-1, 4);

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
				if( t % type3f == 0){
					for(int j = 0; j < S_NUM; j++){
						if( ns[j].type == 1 ){
							transaction(j, t);
						}
					}
				}
				if( t % type4f == 0){
					for(int j = 0; j < S_NUM; j++){
						if( ns[j].type == 2 ){
							transaction(j, t);
						}
					}
				}
				if( t % type5f == 0){
					for(int j = 0; j < S_NUM; j++){
						if( ns[j].type == 3 ){
							transaction(j, t);
						}
					}
				}

				if( t % CHf == 0){
					/*各區的CH送到下一跳*/
					for( int m = 0; m < 4; m++ ){
						for( int k = 0; k < 3; k++ ){
							if( m == 3 && CH_record[m][k] != -1 ){  //R4有sCH或tCH
								CH_to_Sink(CH_record[m][k]);     //R4到Sink
							}
							else if( m != 3 && CH_record[m][k] != -1 ){   //除了R4以外的sCH或tCH
								CH_to_CH4(CH_record[m][k]);    //其他CH到CH4
							}
						}
					}
					toSink_count();
					/*判斷CH4是否小於threshold*/
					for( int i = 0; i < 3; i++ ){
						int ch4 = CH_record[3][i];
						double d = distance(ch4, SINKID);
						double toSink_energy = ProbeEnergy + TransmitEnergy + d*d*AmplifierEnergy;
						if( ch4 != -1 && ns[ch4].energy < toSink_energy*ch_energy_th ){
						/*寫到ch4_reselection函數*/	
						}
					}

					// for(int i = 0; i < 4 ; i++ ){   //依序查看各CH是否energy小於閥值
					// 	for(int j = 0; j < 3; j++){
					// 		int ch = CH_record[i][j];
					// 		int ch_NextHop;
					// 		if( i == 3 ){    //CH4的下一跳是Sink
					// 			ch_NextHop = SINKID;
					// 		}
					// 		else{
					// 			ch_NextHop = CH_record[3][0];   //其他區域CH的下一跳是CH2
					// 		}
					// 		double d = distance(ch, ch_NextHop);
					// 		if( ns[ch].region1 == 4 && ch != -1 && ns[ch].energy < toNextHop_energy*ch_energy_th ){  //實驗二設定CH能量閥值，決定是否進借用程序
					// 			/*進入借用程序的條件：CH能傳送的次數 < 鄰近CH能傳送的次數*/
					// 			/*除了Region2以外的其他CH若沒電，則在區域內更換其他節點即可*/
								
					// 		}

					// 	}
					// }

						}
					}
				t++;
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
