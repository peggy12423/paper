#include <iostream>
#include <time.h>
#include <math.h>
#include <fstream>
#include <queue>
#include <algorithm>

#define MAX_energy 6480//j (1.5V*600mA*3600sec*2 = 6480j) 2*3���q��
#define SINKID 2000
#define SINK_X 400
#define SINK_Y 0
#define SINK_BUFFER_SIZE 5000000
#define NODE_BUFFER1 100 //0~49 �@��CH����CM�� node_buffer 40Kbytes (200��) ��F�o�ӰѼ� �U����bomb�]�n��
#define NODE_BUFFER2 200 //50~100 �S�O���ǿ��

#define R 0.25 //���Y�v �]1�h�S�����Y
#define type3f 360 //�`�Wsensing frequency
#define type4f 480
#define type5f 720
#define CHf 120 //CH trans frequency
#define reservation_energy_time 10000

#define ProbeEnergy 0.03 //j (8*200bit*1.5V*25mA*0.5mS = 0.00375j*8 = 0.03j) !
/*(�d�쪺�פ�:bit*50nj+bit*distance(m)����*100nj)*/
#define TransmitEnergy 0.00008 //0.000000001*50*200*8 j (50nj/bit by�a��) bit * 8 = byte !(�O�o�˺��)
#define AmplifierEnergy 0.00016 //100*200*0.000000001*8 j/distance^2
#define ReceiveEnergy 0.00008 //0.000000001*50*200j*8 (50nj/bit by�a��)
#define Package_size 200 //bytes 
#define node_buffer 20 //Kbytes (100��)
#define successful_rate 5 //�]x ���\�v�N�O100-x%

/*�Ǫ����X�̦n���v��*/
#define Alpha 0.2
#define Beta 0.8

/*�ܰʹ���ѼƳ]�w*/
#define round_number 10
#define E_NUM 1000

//����@�i���
#define density_th1 1.2
#define density_th2 1.4

//����G�i���
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
	int CH2;
    double energy;//ns information
	double dtc;    //�`�I�줤�ߪ��Z��
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
int CH_record[4][3] = { {-1,-1,-1}, {-1,-1,-1}, {-1,-1,-1}, {-1,-1,-1} };  //�����C��region��CH���p
int CH5[6] = {-1,-1,-1,-1,-1,-1};


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
	return region_center;   // ��^�����I���y��
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

void change_node_ch(int sIndex, int eIndex, int region){
	
}

double standard(double a, double b, double ENERGY_STANDARD, double DIST_STANDARD) //used to select CH !�o�ӿ�ܤ覡�ثe�O�y����ӹ����q���Ӥ����Ū���](�O�_�n�Ҽ{���ӯ�q���j�p) !��J�`�I�����Ѧ�?
{
	double s = 0.3*(1 - a / DIST_STANDARD) + 0.7*(b / ENERGY_STANDARD); //larger is better
	return s;
}

void CH_Selection(int s, int e) //s=start e=end !energy���w��
{
	double E = Max_energy(s, e);
	double D = Max_distance(s, e);
	int start = s;
	int end = e;
	int CH = s;
	double re = ns[s].energy - (floor(reservation_energy_time / (ns[s].type * 120))*(ProbeEnergy + (TransmitEnergy + pow(distance(s, ns[s].CH), 2)*AmplifierEnergy)));//�����w����q�Ӥ�|�������,�N��O�t�����Ӥ]�వ�P�_ 120�O������
	double MAX_S = standard(ns[s].dtc, re, E, D);
	s += 1;
	for (s; s <= e; s++)//selecting
	{
		re = ns[s].energy - (floor(reservation_energy_time / (ns[s].type * 120))*(ProbeEnergy + (TransmitEnergy + pow(distance(s, ns[s].CH), 2)*AmplifierEnergy)));//�����w����q�Ӥ�|�������,�N��O�t�����Ӥ]�వ�P�_
		double current_s = standard(ns[s].dtc, re, E, D);
		if (MAX_S < current_s)
		{
			MAX_S = current_s;
			CH = ns[s].id;
		}
	}
	for (start; start <= end; start++)//start to change CH
	{
		ns[start].CH = CH;
	}
	//fout << "CH change to " << CH << endl;
}


void CH_selection(int sIndex, int eIndex, int region){
	int CH_num = region_CH_num(sIndex, eIndex);
	double Energy = Max_energy(sIndex, eIndex);
	double Distance = Max_distance(sIndex, eIndex);
	int start = sIndex;
	int end = eIndex;
	int CH = sIndex;
	int sCH = -1, tCH = -1;
	double d = distance(sIndex, ns[sIndex].CH);
	double reserve_energy = ns[sIndex].energy - (floor(reservation_energy_time / ns[sIndex].rate )*(ProbeEnergy + (TransmitEnergy + d*d*AmplifierEnergy)));
	double standard[3];
	standard[0] = Alpha*(1 - ns[sIndex].dtc / Distance) + Beta*(reserve_energy / Energy);
	sIndex += 1;
	if( CH_num == 2 ){
		sCH = sIndex;
		double d2 = distance(sIndex, ns[sIndex].CH);
		reserve_energy = ns[sIndex].energy - (floor(reservation_energy_time / ns[sIndex].rate )*(ProbeEnergy + (TransmitEnergy + d2*d2*AmplifierEnergy)));
		standard[1] = Alpha*(1 - ns[sIndex].dtc / Distance) + Beta*(reserve_energy / Energy);
		sIndex += 1;
	}
	else if( CH_num == 3 ){
		sCH = sIndex;
		double d2 = distance(sIndex, ns[sIndex].CH);
		reserve_energy = ns[sIndex].energy - (floor(reservation_energy_time / ns[sIndex].rate )*(ProbeEnergy + (TransmitEnergy + d2*d2*AmplifierEnergy)));
		standard[1] = Alpha*(1 - ns[sIndex].dtc / Distance) + Beta*(reserve_energy / Energy);
		sIndex += 1;
		
		tCH = sIndex;
		double d3 = distance(sIndex, ns[sIndex].CH);
		reserve_energy = ns[sIndex].energy - (floor(reservation_energy_time / ns[sIndex].rate )*(ProbeEnergy + (TransmitEnergy + d3*d3*AmplifierEnergy)));
		standard[2] = Alpha*(1 - ns[sIndex].dtc / Distance) + Beta*(reserve_energy / Energy);
		sIndex += 1;
	}

	for( int i = sIndex; i <= eIndex; i++){    //���j��CH�BsCh�BtCH
		d = distance(i, ns[i].CH);
		reserve_energy = ns[i].energy - (floor(reservation_energy_time / ns[i].rate )*(ProbeEnergy + (TransmitEnergy + d*d*AmplifierEnergy)));
		double current_standard = Alpha*(1 - ns[i].dtc / Distance) + Beta*(reserve_energy / Energy);
		if( standard[0] < current_standard ){
			if( sCH != -1 && tCH != -1){    // CH_num = 3
				standard[2] = standard[1];
				standard[1] = standard[0];
				tCH = sCH;
				sCH = CH;
			}
			else if( sCH != -1 && tCH == -1){    // CH_num = 2
				standard[1] = standard[0];
				sCH = CH;
			}
			standard[0] = current_standard;
			CH = ns[i].id;
			continue;
		}
		else if( CH_num > 1 && standard[1] < current_standard){  //����өΤT��CH�ɡA�B��e�зǤ���̤j�P�ĤG�зǤ���
			if( sCH != -1 && tCH != -1 ){   // CH_num =3
				standard[2] = standard[1];
				tCH = sCH;
			}
			standard[1] = current_standard;
			sCH = ns[i].id;
			continue;
		}
		else if( CH_num > 2 && standard[2] < current_standard){   //���T��CH�ɡA�B��e�зǤ���ĤG�P�ĤT�зǤ���
			standard[2] = current_standard;
			tCH = ns[i].id;
			continue;
		}
	}

	CH_record[region-1][0] = CH;
	CH_record[region-1][1] = sCH;
	CH_record[region-1][2] = tCH;

	for( int j = start; j <= end; j++){   //�̷Ө�CH���Z���A���ϰ줺�Ҧ��`�I��CH����X��CH
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

void Packet_Generate(int now, int t) //generate packet �����
{
	total++;
	ns[now].sense.src = ns[now].id;
	ns[now].sense.dst = ns[now].CH;
	ns[now].sense.data = ns[now].type;
	ns[now].sense.time = t;
	ns[now].energy -= ProbeEnergy;
	//fout << "ns : " << now << "��q��� "<< ProbeEnergy <<" ,�]�����ͷP���ʥ] "<< endl;
}

void Packet_Dliver(int sender, int CH) // �����
{
	int rate = rand() % 100 + 1;
	if (rate > successful_rate || sender == CH)  //�e��CH�]�����CH��receive���e
	{
		ns[CH].receive.dst = ns[sender].sense.dst;
		ns[CH].receive.src = ns[sender].sense.src;
		ns[CH].receive.data = ns[sender].sense.data;
		ns[CH].receive.time = ns[sender].sense.time;
	}
	else   //CH�S����]���אּ-1
	{
		macdrop++;
		ns[CH].receive.dst = -1;
		ns[CH].receive.src = -1;
		ns[CH].receive.data = -1;
		ns[CH].receive.time = -1;
	}
	double d = distance(sender, CH);
	if (sender != CH) //CH�ۤv���ۤv���Φ���q
	{
		ns[sender].energy -= TransmitEnergy + d*d*AmplifierEnergy;
		// cons[ns[sender].region1 - 1] += TransmitEnergy + d*d*AmplifierEnergy;
		// trans_time[ns[sender].region1 - 1] += 1;
		//fout << "node : " << sender << "��q��� "<< TransmitEnergy + d*d*AmplifierEnergy <<" ,�]���ǰe���`�I " << CH << " �P���ʥ]" << endl;
	} //1�ӫʥ]
}

void Packet_Receive(int CH) //buffer���F�n�ܦ�priority queue �����
{
	if (ns[CH].receive.src != CH) //���O�Ӧۦۤv���~�n����q
	{
		ns[CH].energy -= ReceiveEnergy;
	} //drop�ٺ�O����
	int full(1);
	for (int a = 0; a < NODE_BUFFER1; a++) //buffer is not full 50 for self-area-sense 50 for other CH 
	{
		if (ns[CH].buffer[a].data == -1)     //CH��receive�ʥ]��Jbuffer
		{
			ns[CH].buffer[a].dst = ns[CH].receive.dst;
			ns[CH].buffer[a].src = ns[CH].receive.src;
			ns[CH].buffer[a].data = ns[CH].receive.data;
			ns[CH].buffer[a].time = ns[CH].receive.time;
			full = 0;
			break;
		}
	}
	if (full == 1 && ns[CH].receive.data != -1)    //buffer���F�Breceive���ʥ]���O-1
	{
		drop++;
	}
	ns[CH].receive.dst = -1;
	ns[CH].receive.src = -1;
	ns[CH].receive.data = -1;
	ns[CH].receive.time = -1;
}

// void transaction(int trans_node, int t){
// 	Packet_Generate(trans_node, t);
// 	Packet_Dliver(trans_node, ns[trans_node].CH);
// 	Packet_Receive(ns[trans_node].CH);
// }

void transaction(int j, int t, int v)
{
	if (v == 1)
	{
		Packet_Generate(j, t);
		Packet_Dliver(j, ns[j].CH);
		Packet_Receive(ns[j].CH);
	}
	else
	{
		Packet_Generate(j, t);
		Packet_Dliver(j, ns[j].CH2);
		Packet_Receive(ns[j].CH2);
	}
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

void CH_to_Sink(int CH){    //CH4��Sink
	int start = 0;    
	for (int b = 0; b < SINK_BUFFER_SIZE; b++)	{
		if (sink.buffer[b].data == -1){   //sink��buffer�q����}�l�O�Ū�
			start = b;
			break;
		}
	}
	if (ns[CH].buffer[NODE_BUFFER1].data != -1){   //���O��CH��
		double rate = 0;
		for (int b = NODE_BUFFER1; b < NODE_BUFFER2; b++){    //�O�ϰe�Ӫ��ʥ]�s�b100~199
			if (ns[CH].buffer[b].data == -1){    //���Ū��N�����~��F
				break;
			}
			int d = rand() % 100 + 1;
			if (d > successful_rate){    //CH2�e��Sink
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
		//fout << "sink����" << rate << "��" << endl;
		rate = ceil(rate * R);
		//fout << rate << endl;
		ns[CH].energy -= (TransmitEnergy + pow(distance(CH, SINKID), 2)*AmplifierEnergy)*rate; //�Ndata�X�֤���@���ǰe �ҥH�ӯ�o�˺�(�X�֥���) �]���i���X�֪�size���bpacket���j�p����
																							   //fout << "���O�H���\,�ڪ���q�u��" << ns[CH].energy << endl;
																							   //fout << "node : " << CH << "��q��� "<< (TransmitEnergy + pow(distance(CH, SINKID), 2)*AmplifierEnergy)*rate <<" ,�]�����O�H�ǵ�sink" << endl;
		clean(CH, NODE_BUFFER1, NODE_BUFFER2); /*�ǧ�����R����*/
	}
	else      //�Ǧۤv�Ϫ��P�����
	{
		double rate(0);/*���Y�v0.25*/
		for (int b = 0; b < NODE_BUFFER1; b++)
		{
			if (ns[CH].buffer[b].data == -1) //���Ū��N�����~��F
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
		//fout << "sink����" << rate << "��" << endl;
		rate = ceil(rate * R);
		//fout << rate << endl;
		ns[CH].energy -= (TransmitEnergy + pow(distance(CH, SINKID), 2)*AmplifierEnergy)*rate; //�Ndata�X�֤���@���ǰe �ҥH�ӯ�o�˺�(�X�֥���)
																							   //fout << "�ۤv��,�ڪ���q�u��" << ns[CH].energy << endl;
																							   //fout << "node : " << CH << "��q��� "<< (TransmitEnergy + pow(distance(CH, SINKID), 2)*AmplifierEnergy)*rate <<" ,�]�����ۤv�ǵ�sink" << endl;
		clean(CH, 0, NODE_BUFFER1); /*�ǧ�����R����*/
	}
}

void CH_to_CH4(int CH) //���F2�ϥH�~���ϰ쳣���2�ϸ̭���q�̰��� �����
{
	/*��CH��2��+2�Ϩ�sink���Z���ۥ[�P��Ѿl��q�Ȱ��[�v*/
	int dst = CH_record[3][0];
	double rate = 0;/*���Y�v0.25*/
	for (int b = 0; b < NODE_BUFFER1; b++){
		if (ns[CH].buffer[b].data == -1){   //���Ū��N�����~��F
			break;
		}
		int d = rand() % 100 + 1; 
		if (d > successful_rate){   //��L�ϰ�e�Ӫ��s�bbuffer 100~199
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
	ns[CH].energy -= (TransmitEnergy + pow(distance(CH, dst), 2)*AmplifierEnergy)*rate; //�Ndata�X�֤���@���ǰe �ҥH�ӯ�o�˺�(�X�֥���)
																						  //fout << "node : " << CH << "��q��� "<<(TransmitEnergy + pow(distance(CH1, dst), 2)*AmplifierEnergy)*rate<<", �]���ǿ鵹�ϰ�2" << endl;
	ns[dst].energy -= (ReceiveEnergy)*rate;
	CH_to_Sink(dst);
}

void Reselection_judge(int sIndex, int eIndex, int rnum, int region)
{
	double avg_energy = find_avg_energy(sIndex, eIndex, rnum);
	if (((avg_energy - ns[ns[sIndex].CH].energy) / avg_energy) >= 0.15) //�o�ӭȤ��@�w�j��0 , CH����O���@�w��P�� ! �]��������Y�����Y
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

double find_avg_energy(int sIndex, int eIndex){
	double avg_energy = 0;
	for (int i = sIndex; i <= eIndex; i++){
		avg_energy += ns[i].energy;
	}
	avg_energy /= S_NUM*0.25;
	return avg_energy;
}

void ch4_reselection(){
	int sIndex = R4;
	int eIndex = S_NUM-1;
	double Energy = Max_energy(sIndex, eIndex);
	double Distance = Max_distance(sIndex, eIndex);
	int CH = sIndex;
	int sCH = -1, tCH = -1;
	double d = distance(sIndex, ns[sIndex].CH);
	double reserve_energy = ns[sIndex].energy - (floor(reservation_energy_time / ns[sIndex].rate )*(ProbeEnergy + (TransmitEnergy + d*d*AmplifierEnergy)));
	double standard[3];
	standard[0] = Alpha*(1 - ns[sIndex].dtc / Distance) + Beta*(reserve_energy / Energy);
	sIndex += 1;
	int CH_num;
	if( CH_record[3][1] != 1 && CH_record[3][2] != -1){   //CH4�BsCH4�BtCH4
		CH_num = 3;
		sCH = sIndex;
		double d2 = distance(sIndex, ns[sIndex].CH);
		reserve_energy = ns[sIndex].energy - (floor(reservation_energy_time / ns[sIndex].rate )*(ProbeEnergy + (TransmitEnergy + d2*d2*AmplifierEnergy)));
		standard[1] = Alpha*(1 - ns[sIndex].dtc / Distance) + Beta*(reserve_energy / Energy);
		sIndex += 1;
		
		tCH = sIndex;
		double d3 = distance(sIndex, ns[sIndex].CH);
		reserve_energy = ns[sIndex].energy - (floor(reservation_energy_time / ns[sIndex].rate )*(ProbeEnergy + (TransmitEnergy + d3*d3*AmplifierEnergy)));
		standard[2] = Alpha*(1 - ns[sIndex].dtc / Distance) + Beta*(reserve_energy / Energy);
		sIndex += 1;
	}
	else if( CH_record[3][1] != 1 && CH_record[3][2] == -1){   //CH4�BsCH4
		CH_num = 2;
		sCH = sIndex;
		double d2 = distance(sIndex, ns[sIndex].CH);
		reserve_energy = ns[sIndex].energy - (floor(reservation_energy_time / ns[sIndex].rate )*(ProbeEnergy + (TransmitEnergy + d2*d2*AmplifierEnergy)));
		standard[1] = Alpha*(1 - ns[sIndex].dtc / Distance) + Beta*(reserve_energy / Energy);
		sIndex += 1;
	}

	for( int i = sIndex; i <= eIndex; i++){    //���j��CH�BsCh�BtCH
		d = distance(i, ns[i].CH);
		reserve_energy = ns[i].energy - (floor(reservation_energy_time / ns[i].rate )*(ProbeEnergy + (TransmitEnergy + d*d*AmplifierEnergy)));
		double current_standard = Alpha*(1 - ns[i].dtc / Distance) + Beta*(reserve_energy / Energy);
		if( standard[0] < current_standard ){
			if( sCH != -1 && tCH != -1){    // CH_num = 3
				standard[2] = standard[1];
				standard[1] = standard[0];
				tCH = sCH;
				sCH = CH;
			}
			else if( sCH != -1 && tCH == -1){    // CH_num = 2
				standard[1] = standard[0];
				sCH = CH;
			}
			standard[0] = current_standard;
			CH = ns[i].id;
			continue;
		}
		else if( CH_num > 1 && standard[1] < current_standard){  //����өΤT��CH�ɡA�B��e�зǤ���̤j�P�ĤG�зǤ���
			if( sCH != -1 && tCH != -1 ){   // CH_num =3
				standard[2] = standard[1];
				tCH = sCH;
			}
			standard[1] = current_standard;
			sCH = ns[i].id;
			continue;
		}
		else if( CH_num > 2 && standard[2] < current_standard){   //���T��CH�ɡA�B��e�зǤ���ĤG�P�ĤT�зǤ���
			standard[2] = current_standard;
			tCH = ns[i].id;
			continue;
		}
	}

	CH_record[3][0] = CH;
	CH_record[3][1] = sCH;
	CH_record[3][2] = tCH;

	for(int i = 0; i < 3; i++ ){  //�̧��ˬdCH4�BsCH4�BtCH4�O�_�p��֭�
		int ch4 = CH_record[3][i];
		double d = distance(ch4, SINKID);
		double toSink_energy = ProbeEnergy + TransmitEnergy + d*d*AmplifierEnergy;
		if( ch4 != -1 && ns[ch4].energy < toSink_energy*ch_energy_th ){  //�Y��q�Ӥ֫h������CH
			CH_record[3][i] = -1;
		}
	}

	for( int j = R4; j < S_NUM ; j++){   //�̷Ө�CH���Z���A���ϰ줺�Ҧ��`�I��CH����X��CH
		if( sCH != -1 && tCH != -1){   //CH_num = 3
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
		else if( sCH != -1 && tCH == -1){  //CH_num = 2
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
		
		else{
			ns[j].CH = CH;
		}
	}
}

void CH5_selection(int CH5_num){
	int sIndex = R2;
	int eIndex = S_NUM-1;
	double Energy = Max_energy(sIndex, eIndex);
	double Distance = Max_distance(sIndex, eIndex);
	int start = sIndex;
	int end = eIndex;
	double d = distance(sIndex, ns[sIndex].CH);
	double reserve_energy = ns[sIndex].energy - (floor(reservation_energy_time / ns[sIndex].rate )*(ProbeEnergy + (TransmitEnergy + d*d*AmplifierEnergy)));
	double standard[S_NUM-R2];
	for(int i = R2; i <= S_NUM-1; i++){
		double d = distance(i, ns[i].CH);
		reserve_energy = ns[i].energy - (floor(reservation_energy_time / ns[i].rate )*(ProbeEnergy + (TransmitEnergy + d*d*AmplifierEnergy)));
		standard[i] = Alpha*(1 - ns[i].dtc / Distance) + Beta*(reserve_energy / Energy);
	}
	sort(standard, standard + (S_NUM-R2), greater<double>());
	for(int j = 0; j < CH5_num; j++){
		CH5[j] = standard[j];
	}
	for(int k = start; k <= end; k++){
		double distance_toCH5[CH5_num];
		for(int m = 0; m < CH5_num; m++){
			distance_toCH5[m] = distance(k, CH5[m]);
		}
		sort( distance_toCH5, distance_toCH5+CH5_num );
		/*�Z��CH5�̤p���@���n�ǥh���ت�CH*/
		/*sort�᪺index�|�ñ��A�䤣��CH����*/
	}
}

void merge_grid(){
	for(int i = R2; i < S_NUM ; i++){
		ns[i].region1 = 5;
	}
	int CH5_num = region_CH_num(R2, R3-1) + region_CH_num(R3, R4-1);
	CH5_selection(CH5_num);

}

int main(){
	// ofstream fout("my_normal.txt");
	// streambuf *coutbuf = cout.rdbuf();
	// cout.rdbuf(fout.rdbuf());
	cout << "My method" << endl;

	srand((unsigned)time(NULL)); //random seed
	for (int round = 0; round < round_number; round++){
		cout<<"start"<<endl;
		node_deployed();
		// special_node_deployed();
		
		/*initialization*/
		packet_init();
		sink_init();

   		/*firts CH selection*/
		
		CH_Selection(0, R2-1);
		CH_Selection(R2, R3-1);
		CH_Selection(R3, R4-1);
		CH_Selection(R4, S_NUM-1);
		// CH_election(0, R2-1, 1);
		// CH_election(R2, R3-1, 2);
		// CH_election(R3, R4-1, 3);
		// CH_election(R4, S_NUM-1, 4);

		/*traffic start*/
		int die = 0;
		int t = 1;
		while(!die){
			int dead_node = CheckEnergy();
			if( dead_node < SINKID ){    //��node����
				cout << "die" <<endl;
				avg_time += t;
				die = 1;
				break;
			}
			if (t % type3f == 0)
				{
					for (int j = 0; j < S_NUM; j++)
					{
						if (ns[j].type == 1) //CH need to sense
						{
							if (ns[j].CH2 == -1 || ns[j].CH == ns[j].id)
							{
								transaction(j, t, 1);
							}
							else
							{
								transaction(j, t, 2);
							}
						}
					}
				}
				if (t % type4f == 0)
				{
					for (int j = 0; j < S_NUM; j++)
					{
						if (ns[j].type == 2) //CH need to sense
						{
							if (ns[j].CH2 == -1 || ns[j].CH == ns[j].id)
							{
								transaction(j, t, 1);
							}
							else
							{
								transaction(j, t, 2);
							}
						}
					}
				}
				if (t % type5f == 0)
				{
					for (int j = 0; j < S_NUM; j++)
					{
						if (ns[j].type == 3) //CH need to sense
						{
							if (ns[j].CH2 == -1 || ns[j].CH == ns[j].id)
							{
								transaction(j, t, 1);
							}
							else
							{
								transaction(j, t, 2);
							}
						}
					}
				}

			if( t % CHf == 0){  //CH�Ǩ�Sink��CH�Ǩ�CH4
				/*�U�Ϫ�CH�e��U�@��*/
				// for( int m = 0; m < 4; m++ ){
				// 	for( int k = 0; k < 3; k++ ){
				// 		if( m == 3 && CH_record[m][k] != -1 ){  //R4��sCH��tCH
				// 			CH_to_Sink(CH_record[m][k]);     //R4��Sink
				// 		}
				// 		else if( m != 3 && CH_record[m][k] != -1 ){   //���FR4�H�~��sCH��tCH
				// 			CH_to_CH4(CH_record[m][k]);    //��LCH��CH4
				// 		}
				// 	}
				// }

				int CH[4];
				CH[0] = ns[0].CH;
				CH[1] = ns[R2].CH;
				CH[2] = ns[R3].CH;
				CH[3] = ns[R4].CH;
				CH_to_Sink(CH[3]);
				CH_to_CH4(CH[0]);
				CH_to_CH4(CH[1]);
				CH_to_CH4(CH[2]);
				CH_Reselection();
			}
				// /*�P�_CH4�O�_�p��threshold*/
				// ch4_reselection();
				// if( CH_record[3][0] == -1 &&CH_record[3][1] == -1 && CH_record[3][2] == -1 ){    //R4���S���X�A��CH�F�A�i�J�X�ֵ{��

				// }
			t++;
		}
	}
	total /= round_number;
	macdrop /= round_number;
	drop /= round_number;
	avg_time /= round_number;
	cout << "avg_lifetime : " << avg_time << endl;
	cout << "avg_total : " << total << endl;
	cout << "avg_macdrop : " << macdrop << endl;
	cout << "avg_drop : " << drop << endl;
	cout << "avg_PLR : " << (drop + macdrop) / total << endl;

	// cout.rdbuf(coutbuf);
    // fout.close();
	return 0;
}
