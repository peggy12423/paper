#include <iostream>
#include <time.h>
#include <math.h>
#include <fstream>
#define MAX_energy 6480//j (1.5V*600mA*3600sec*2 = 6480j) 2*3���q��
#define SINKID 2000
#define SINK_X 400
#define SINK_Y 0
#define SINK_BUFFER_SIZE 5000000
#define NODE_BUFFER1 200 //0~49 �@��CH����CM�� node_buffer 40Kbytes (200��) ��F�o�ӰѼ� �U����bomb�]�n��
#define NODE_BUFFER2 400 //50~100 �S�O���ǿ��

#define R 0.5 //���Y�v �]1�h�S�����Y
#define type3f 90//�`�Wsensing frequency
#define type4f 120
#define type5f 150 //720
#define reservation_energy_time 10000
#define CHf 120 //CH trans frequency

/**/
#define ProbeEnergy 0.03 //j (8*200bit*1.5V*25mA*0.5mS = 0.00375j*8 = 0.03j) !
/*(�d�쪺�פ�:bit*50nj+bit*distance(m)����*100nj)*/
#define TransmitEnergy 0.00008 //0.000000001*50*200*8 j (50nj/bit by�a��) bit * 8 = byte !
#define AmplifierEnergy 0.00016 //100*200*0.000000001*8 j/distance^2
#define ReceiveEnergy 0.00008 //0.000000001*50*200j*8 (50nj/bit by�a��)
#define Package_size 200 //bytes 
#define successful_rate 5 //�]x ���\�v�N�O100-x%

/*�ܰʹ���ѼƳ]�w*/
#define round_number 10
#define E_NUM 1000
#define Alpha 0.2
#define Beta 0.8
#define high_density_th1 1.2
#define high_density_th2 1.4

using namespace std;

int S_NUM = 200; //�P�����`��
struct C
{
	double x, y;
};
struct P
{
	int src;
	int dst;
	int data;
	int time;
};
struct N
{
	int id, x, y, CH, type, region1, region2;//region1 for �Ĥ@�hgrid , region2 for �ĤG�hgrid
	double energy;//node information
    int compress;
    int delay;
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
ofstream fout("my_special.txt");
N ns[2000];
S sink;
double avg_t, buffer_drop, mac_drop, total;
int R2, R3, R4;
int R_NUM = S_NUM * 0.25;

double type_a = 33, type_b = 33, type_c = 34; //�վ�QUERE�̭��P����ƪ����

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

C find_center(int s, int e) //�M��ϰ줤�Ҧ��I�����ߺɶq�����Z��
{
	C c;
	c.x = 0.0; c.y = 0.0;
	for (int i = s; i <= e; i++)
	{
		c.x += ns[i].x;
		c.y += ns[i].y;
	}
	int n = e - s + 1; //n=�ϰ�`�I��
	c.x /= n;
	c.y /= n;
	return c;
}

void set_dtc(C c, int s, int e)
{
	for (int i = s; i <= e; i++)
	{
		ns[i].dtc = sqrt(pow(abs(ns[i].x - c.x), 2) + pow(abs(ns[i].y - c.y), 2));
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
		ns[i].region1 = 1;
        ns[i].compress = 0;
		if (ns[i].x <= 100 && ns[i].y <= 100)
		{
			ns[i].region2 = 1;
		}
		if (ns[i].x >= 100 && ns[i].y <= 100)
		{
			ns[i].region2 = 2;
		}
		if (ns[i].x <= 100 && ns[i].y >= 100)
		{
			ns[i].region2 = 3;
		}
		if (ns[i].x >= 100 && ns[i].y >= 100)
		{
			ns[i].region2 = 4;
		}
		//ns[i].dtc = sqrt(pow(abs(ns[i].x - 100), 2) + pow(abs(ns[i].y - 100), 2)); /*�Z���Ϥ���*/
		//ns[i].dtc = abs(ns[i].x - 200);/*�Z����2*/
		if (i == R2 - 1)//�Z���ϩҦ��I����
		{
			C center1 = find_center(0, i);
			set_dtc(center1, 0, i);
		}
		//fout << "node" << ns[i].id << " x : " << ns[i].x << " y : " << ns[i].y << " type : " << ns[i].type << " energy : " << ns[i].energy << " dtc : " << ns[i].dtc << endl;
	}
	for (i; i < R3; i++)
	{
		ns[i].id = i;
		ns[i].x = rand() % 200 + 201;
		ns[i].y = rand() % 200 + 1;
		ns[i].CH = i;
		ns[i].type = rand() % 3 + 3;
		ns[i].energy = MAX_energy;
		ns[i].region1 = 2;
        ns[i].compress = 0;
        if (ns[i].x <= 300 && ns[i].y <= 100)
		{
			ns[i].region2 = 1;
		}
		if (ns[i].x >= 300 && ns[i].y <= 100)
		{
			ns[i].region2 = 2;
		}
		if (ns[i].x <= 300 && ns[i].y >= 100)
		{
			ns[i].region2 = 3;
		}
		if (ns[i].x >= 300 && ns[i].y >= 100)
		{
			ns[i].region2 = 4;
		}
		//ns[i].dtc = sqrt(pow(abs(ns[i].x - 300), 2) + pow(abs(ns[i].y - 100), 2)); /*�Z���Ϥ���*/
		//ns[i].dtc = sqrt(pow(abs(ns[i].x - SINKID), 2) + pow(abs(ns[i].y - 0), 2)); /*�Z��sink*/
		if (i == R3 - 1)//�Z���ϩҦ��I����
		{
			C center2 = find_center(R2, i);
			set_dtc(center2, R2, i);
		}
		//fout << "node" << ns[i].id << " x : " << ns[i].x << " y : " << ns[i].y << " type : " << ns[i].type << " energy : " << ns[i].energy << " dtc : " << ns[i].dtc << endl;
	}
	for (i; i < R4; i++)
	{
		ns[i].id = i;
		ns[i].x = rand() % 200 + 1;
		ns[i].y = rand() % 200 + 201;
		ns[i].CH = i;
		ns[i].type = rand() % 3 + 3;
		ns[i].energy = MAX_energy;
		ns[i].region1 = 3;
        ns[i].compress = 0;
		if (ns[i].x <= 100 && ns[i].y <= 300)
		{
			ns[i].region2 = 1;
		}
		if (ns[i].x >= 100 && ns[i].y <= 300)
		{
			ns[i].region2 = 2;
		}
		if (ns[i].x <= 100 && ns[i].y >= 300)
		{
			ns[i].region2 = 3;
		}
		if (ns[i].x >= 100 && ns[i].y >= 300)
		{
			ns[i].region2 = 4;
		}
		//ns[i].dtc = sqrt(pow(abs(ns[i].x - 100), 2) + pow(abs(ns[i].y - 300), 2));/*�Z���Ϥ���*/
		//ns[i].dtc = sqrt(pow(abs(ns[i].x - 200), 2) + pow(abs(ns[i].y - 200), 2));/*�Z����2*/
		if (i == R4 - 1)//�Z���ϩҦ��I����
		{
			C center3 = find_center(R3, i);
			set_dtc(center3, R3, i);
		}
		//fout << "node" << ns[i].id << " x : " << ns[i].x << " y : " << ns[i].y << " type : " << ns[i].type << " energy : " << ns[i].energy << " dtc : " << ns[i].dtc << endl;
	}
	for (i; i < S_NUM; i++)
	{
		ns[i].id = i;
		ns[i].x = rand() % 200 + 201;
		ns[i].y = rand() % 200 + 201;
		ns[i].CH = i;
		ns[i].type = rand() % 3 + 3;
		ns[i].energy = MAX_energy;
		ns[i].region1 = 4;
        ns[i].compress = 0;
		if (ns[i].x <= 300 && ns[i].y <= 300)
		{
			ns[i].region2 = 1;
		}
		if (ns[i].x >= 300 && ns[i].y <= 300)
		{
			ns[i].region2 = 2;
		}
		if (ns[i].x <= 300 && ns[i].y >= 300)
		{
			ns[i].region2 = 3;
		}
		if (ns[i].x >= 300 && ns[i].y >= 300)
		{
			ns[i].region2 = 4;
		}
		//ns[i].dtc = sqrt(pow(abs(ns[i].x - 300), 2) + pow(abs(ns[i].y - 300), 2));/*�Z���Ϥ���*/
		//ns[i].dtc = abs(ns[i].y - 200);/*�Z����2*/
		if (i == S_NUM - 1)//�Z���ϩҦ��I����
		{
			C center4 = find_center(R4, i);
			set_dtc(center4, R4, i);
		}
		//fout << "node" << ns[i].id << " x : " << ns[i].x << " y : " << ns[i].y << " type : " << ns[i].type << " energy : " << ns[i].energy << " dtc : " << ns[i].dtc << endl;
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
		ns[i].CH = i;
		ns[i].type = rand() % 3 + 3;//3 4 5
		ns[i].energy = MAX_energy;
		ns[i].region1 = 1;
        ns[i].compress = 0;
		if (ns[i].x <= 100 && ns[i].y <= 100)
		{
			ns[i].region2 = 1;
		}
		if (ns[i].x >= 100 && ns[i].y <= 100)
		{
			ns[i].region2 = 2;
		}
		if (ns[i].x <= 100 && ns[i].y >= 100)
		{
			ns[i].region2 = 3;
		}
		if (ns[i].x >= 100 && ns[i].y >= 100)
		{
			ns[i].region2 = 4;
		}
		if (i == R2 - 1)//�Z���ϩҦ��I����
		{
			C center1 = find_center(0, i);
			set_dtc(center1, 0, i);
		}
		//fout << "node" << ns[i].id << " x : " << ns[i].x << " y : " << ns[i].y << " type : " << ns[i].type << " energy : " << ns[i].energy << " dtc : " << ns[i].dtc << endl;
	}
	for (i; i < R3; i++)
	{
		ns[i].id = i;
		ns[i].x = rand() % 200 + 201;
		ns[i].y = rand() % 200 + 1;
		ns[i].CH = i;
		ns[i].type = rand() % 3 + 3;
		ns[i].energy = MAX_energy;
		ns[i].region1 = 2;
        ns[i].compress = 0;
		if (ns[i].x <= 300 && ns[i].y <= 100)
		{
			ns[i].region2 = 1;
		}
		if (ns[i].x >= 300 && ns[i].y <= 100)
		{
			ns[i].region2 = 2;
		}
		if (ns[i].x <= 300 && ns[i].y >= 100)
		{
			ns[i].region2 = 3;
		}
		if (ns[i].x >= 300 && ns[i].y >= 100)
		{
			ns[i].region2 = 4;
		}
		if (i == R3 - 1)//�Z���ϩҦ��I����
		{
			C center2 = find_center(R2, i);
			set_dtc(center2, R2, i);
		}
		//fout << "node" << ns[i].id << " x : " << ns[i].x << " y : " << ns[i].y << " type : " << ns[i].type << " energy : " << ns[i].energy << " dtc : " << ns[i].dtc << endl;
	}
	for (i; i < R4; i++)
	{
		ns[i].id = i;
		ns[i].x = rand() % 200 + 1;
		ns[i].y = rand() % 200 + 201;
		ns[i].CH = i;
		ns[i].type = rand() % 3 + 3;
		ns[i].energy = MAX_energy;
		ns[i].region1 = 3;
        ns[i].compress = 0;
		if (ns[i].x <= 100 && ns[i].y <= 300)
		{
			ns[i].region2 = 1;
		}
		if (ns[i].x >= 100 && ns[i].y <= 300)
		{
			ns[i].region2 = 2;
		}
		if (ns[i].x <= 100 && ns[i].y >= 300)
		{
			ns[i].region2 = 3;
		}
		if (ns[i].x >= 100 && ns[i].y >= 300)
		{
			ns[i].region2 = 4;
		}
		if (i == R4 - 1)//�Z���ϩҦ��I����
		{
			C center3 = find_center(R3, i);
			set_dtc(center3, R3, i);
		}
		//fout << "node" << ns[i].id << " x : " << ns[i].x << " y : " << ns[i].y << " type : " << ns[i].type << " energy : " << ns[i].energy << " dtc : " << ns[i].dtc << endl;
	}
	for (i; i < S_NUM; i++)
	{
		ns[i].id = i;
		ns[i].x = rand() % 200 + 201;
		ns[i].y = rand() % 200 + 201;
		ns[i].CH = i;
		ns[i].type = rand() % 3 + 3;
		ns[i].energy = MAX_energy;
		ns[i].region1 = 4;
        ns[i].compress = 0;
		if (ns[i].x <= 300 && ns[i].y <= 300)
		{
			ns[i].region2 = 1;
		}
		if (ns[i].x >= 300 && ns[i].y <= 300)
		{
			ns[i].region2 = 2;
		}
		if (ns[i].x <= 300 && ns[i].y >= 300)
		{
			ns[i].region2 = 3;
		}
		if (ns[i].x >= 300 && ns[i].y >= 300)
		{
			ns[i].region2 = 4;
		}
		if (i == S_NUM - 1)//�Z���ϩҦ��I����
		{
			C center4 = find_center(R4, i);
			set_dtc(center4, R4, i);
		}
		//fout << "node" << ns[i].id << " x : " << ns[i].x << " y : " << ns[i].y << " type : " << ns[i].type << " energy : " << ns[i].energy << " dtc : " << ns[i].dtc << endl;
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

double CH_standard(double a, double b, double ENERGY_STANDARD, double DIST_STANDARD) //used to select CH !�o�ӿ�ܤ覡�ثe�O�y����ӹ����q���Ӥ����Ū���](�O�_�n�Ҽ{���ӯ�q���j�p) !��J�`�I�����Ѧ�?
{
	double s = Alpha*(1 - a / DIST_STANDARD) + Beta*(b / ENERGY_STANDARD); //larger is better
	return s;
}
double find_max_energy(int s, int e) //!energy���w��
{
	double max(0);
	for (int i = s; i <= e; i++)
	{
		if (max < ns[i].energy)
		{
			max = ns[i].energy;
		}
	}
	return max;
}
double find_avg_energy(int s, int e)
{
	double avg_energy(0.0);
	for (int i = s; i <= e; i++)
	{
		avg_energy += ns[i].energy;
	}
	avg_energy /= R_NUM;
	return avg_energy;
}

double find_max_distance(int s, int e)
{
	double d(0.0);
	for (int i = s; i <= e; i++)
	{
		if (d < ns[i].dtc)
		{
			d = ns[i].dtc;
		}
	}
	return d;
}
double find_max_dts(int s, int e, int CH) //�u��CH�|�Ψ�o��function
{
	double d(0.0);
	for (int i = s; i <= e; i++)
	{
		if (d < distance(CH, i) + distance(i, SINKID))
		{
			d = pow(distance(CH, i), 2) + pow(distance(i, SINKID), 2);
		}
	}
	return d;
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

void CH_selection(int sIndex, int eIndex, int region, int (&CH_record)[4][3] ){
	int CH_num = region_CH_num(sIndex, eIndex);
	double Energy = find_max_energy(sIndex, eIndex);
	double Distance = find_max_distance(sIndex, eIndex);
	int start = sIndex;
	int end = eIndex;
	int CH = sIndex;
	int sCH = -1, tCH = -1;
	double d = distance(sIndex, ns[sIndex].CH);
	double reserve_energy = ns[sIndex].energy - (floor(reservation_energy_time / (ns[sIndex].type * 30) )*(ProbeEnergy + (TransmitEnergy + d*d*AmplifierEnergy)));
	double standard[3];
	standard[0] = CH_standard(ns[sIndex].dtc, reserve_energy, Energy, Distance);
	sIndex += 1;
	if( CH_num == 2 ){
		sCH = sIndex;
		double d2 = distance(sIndex, ns[sIndex].CH);
		reserve_energy = ns[sIndex].energy - (floor(reservation_energy_time / (ns[sIndex].type * 30) )*(ProbeEnergy + (TransmitEnergy + d2*d2*AmplifierEnergy)));
		standard[1] = CH_standard(ns[sIndex].dtc, reserve_energy, Energy, Distance);
		sIndex += 1;
	}
	else if( CH_num == 3 ){
		sCH = sIndex;
		double d2 = distance(sIndex, ns[sIndex].CH);
		reserve_energy = ns[sIndex].energy - (floor(reservation_energy_time / (ns[sIndex].type * 30) )*(ProbeEnergy + (TransmitEnergy + d2*d2*AmplifierEnergy)));
		standard[1] = CH_standard(ns[sIndex].dtc, reserve_energy, Energy, Distance);
		sIndex += 1;
		
		tCH = sIndex;
		double d3 = distance(sIndex, ns[sIndex].CH);
		reserve_energy = ns[sIndex].energy - (floor(reservation_energy_time / (ns[sIndex].type * 30) )*(ProbeEnergy + (TransmitEnergy + d3*d3*AmplifierEnergy)));
		standard[2] = CH_standard(ns[sIndex].dtc, reserve_energy, Energy, Distance);
		sIndex += 1;
	}

	for( sIndex; sIndex <= eIndex; sIndex++){    //���j��CH�BsCh�BtCH�BfCH
		d = distance(sIndex, ns[sIndex].CH);
		reserve_energy = ns[sIndex].energy - (floor(reservation_energy_time / (ns[sIndex].type * 30) )*(ProbeEnergy + (TransmitEnergy + d*d*AmplifierEnergy)));
		double current_standard = CH_standard(ns[sIndex].dtc, reserve_energy, Energy, Distance);
		if( standard[0] < current_standard ){
			if( CH_num == 3 ){    // CH_num = 3
				standard[2] = standard[1];
				standard[1] = standard[0];
				tCH = sCH;
				sCH = CH;
			}
			else if( CH_num == 2 ){    // CH_num = 2
				standard[1] = standard[0];
				sCH = CH;
			}
			standard[0] = current_standard;
			CH = ns[sIndex].id;
		}
		else if( CH_num > 1 && standard[1] < current_standard){  //����өΤT��CH�ɡA�B��e�зǤ���̤j�P�ĤG�зǤ���
			if( CH_num == 3 ){   // CH_num =3
				standard[2] = standard[1];
				tCH = sCH;
			}
			standard[1] = current_standard;
			sCH = ns[sIndex].id;
		}
		else if( CH_num > 2 && standard[2] < current_standard){   //���T��CH�ɡA�B��e�зǤ���ĤG�P�ĤT�зǤ���
			standard[2] = current_standard;
			tCH = ns[sIndex].id;
		}
	}

	CH_record[region][0] = CH;
	CH_record[region][1] = sCH;
	CH_record[region][2] = tCH;

	for( int j = start; j <= end; j++){   //�̷Ө�CH���Z���A���ϰ줺�Ҧ��`�I��CH����X��CH
		if( sCH != -1 && tCH != -1 && CH_num == 3){   //CH_num = 3
			double dist1 = distance(j, CH);
			double dist2 = distance(j, sCH);
			double dist3 = distance(j, tCH);
			if( dist1 <= dist2 && dist1 <= dist3 ){
				ns[j].CH = CH;
			}
			else if( dist2 <= dist1 && dist2 <= dist3){
				ns[j].CH = sCH;
			}
			else{
				ns[j].CH = tCH;
			}
		}
        else if( sCH != -1 && CH_num == 2){  //CH_num = 2
			double dist1 = distance(j, CH);
			double dist2 = distance(j, sCH);
			if( dist1 <= dist2){
				ns[j].CH = CH;
			}
			else{
				ns[j].CH = sCH;
			}
		}
		else{
			ns[j].CH = CH;
		}
	}
}

void Packet_Generate(int now, int t) //generate packet �����
{
	total++;
	ns[now].sense.src = ns[now].id;
	ns[now].sense.dst = ns[now].CH;
	ns[now].sense.data = ns[now].type;
	ns[now].sense.time = t;
	ns[now].energy -= ProbeEnergy;
    ns[now].compress += 1;
	//fout << "node : " << now << "��q��� "<< ProbeEnergy <<" ,�]�����ͷP���ʥ] "<< endl;
}
void Packet_Dliver(int sender, int CH) // �����
{
    double d = distance(sender, CH);
	if (sender != CH) //CH�ۤv���ۤv���Φ���q
	{
		ns[sender].energy -= TransmitEnergy + d*d*AmplifierEnergy;
	}
    int rate = rand() % 100 + 1;
	if (rate > successful_rate || sender == CH)  /*10% drop rate or CH�ۤv�Nsense���ʥ]��ۤv��buffer*/
	{
		ns[CH].receive.dst = ns[sender].sense.dst;
		ns[CH].receive.src = ns[sender].sense.src;
		ns[CH].receive.data = ns[sender].sense.data;
		ns[CH].receive.time = ns[sender].sense.time;
		//fout << "node id: "<<ns[CH].id<<" receive the packet type "<<sender.type <<" from node "<<sender.id << " at " << ns[CH].receive.time<<" sec"<< endl;
	}
	else
	{
		mac_drop += 2;
		ns[CH].receive.dst = -1;
		ns[CH].receive.src = -1;
		ns[CH].receive.data = -1;
		ns[CH].receive.time = -1;
	}
}
void Packet_Receive(int CH) //buffer���F�n�ܦ�priority queue �����
{
	if ( ns[CH].receive.src != CH) //���O�Ӧۦۤv���~�n����q
	{
		ns[CH].energy -= ReceiveEnergy;
	} //drop�ٺ�O����
	int full(1);
	for (int a = 0; a < NODE_BUFFER1; a++) //buffer is not full 50 for self-area-sense 50 for other CH 
	{
		if (ns[CH].buffer[a].data == -1)
		{
			ns[CH].buffer[a].dst = ns[CH].receive.dst;
			ns[CH].buffer[a].src = ns[CH].receive.src;
			ns[CH].buffer[a].data = ns[CH].receive.data;
			ns[CH].buffer[a].time = ns[CH].receive.time;
			full = 0;
			break;
		}
	}
	if (full == 1 && ns[CH].receive.data != -1)//priority queue buffer , �p�G�ʥ]�Qdrop���N���ΤF(-1)
	{
		buffer_drop++;
	}
	ns[CH].receive.dst = -1;
	ns[CH].receive.src = -1;
	ns[CH].receive.data = -1;
	ns[CH].receive.time = -1;
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

void CH2Sink(int CH) //�����
{
	int start(0);/*sink��buffer�q����}�l�O�Ū�*/
	for (int b = 0; b < SINK_BUFFER_SIZE; b++)
	{
		if (sink.buffer[b].data == -1)
		{
			start = b;
			break;
		}
	}
	if (ns[CH].buffer[NODE_BUFFER1].data != -1)/*���O��CH��*/
	{
		double rate(0);/*���Y�v0.25*/
		for (int b = NODE_BUFFER1; b < NODE_BUFFER2; b++)
		{
			if (ns[CH].buffer[b].data == -1)  //���Ū��N�����~��F
			{
				break;
			}
			int d = rand() % 100 + 1; /*?????????????????????*/
			if (d > successful_rate)
			{
				rate = b - NODE_BUFFER1 + 1;
				sink.buffer[start].data = ns[CH].buffer[b].data;
				sink.buffer[start].dst = ns[CH].buffer[b].dst;
				sink.buffer[start].src = ns[CH].buffer[b].src;
				sink.buffer[start].time = ns[CH].buffer[b].time;
				start++;
			}
			else
			{
				mac_drop++;
			}
		}
		rate = ceil(rate * R);
		ns[CH].energy -= (TransmitEnergy + pow(distance(CH, SINKID), 2)*AmplifierEnergy)*rate; //�Ndata�X�֤���@���ǰe �ҥH�ӯ�o�˺�(�X�֥���) �]���i���X�֪�size���bpacket���j�p����
		clean(CH, NODE_BUFFER1, NODE_BUFFER2); /*�ǧ�����R����*/
	}
	else      /*�ۤv��*/
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
				//fout << "CH ID:" << ns[CH].id << " src: " << sink.buffer[start].src << " dst: " << sink.buffer[start].dst << " data: " << sink.buffer[start].data << " time: " << sink.buffer[start].time << " sec" << endl;
				start++;
			}
			else
			{
				mac_drop++;
			}
		}
		rate = ceil(rate * R);
		ns[CH].energy -= (TransmitEnergy + pow(distance(CH, SINKID), 2)*AmplifierEnergy)*rate; //�Ndata�X�֤���@���ǰe �ҥH�ӯ�o�˺�(�X�֥���)
		clean(CH, 0, NODE_BUFFER1); /*�ǧ�����R����*/
	}
}
void CHtoRegion2(int CH1) //���F2�ϥH�~���ϰ쳣���2�ϸ̭���q�̰��� �����
{
	/*��CH��2��+2�Ϩ�sink���Z���ۥ[�P��Ѿl��q�Ȱ��[�v*/
	int dst = R2;
	double d1 = distance(CH1, R2); //�O�Ϩ�Y�I
	double d2 = distance(R2, SINKID);  //�Y�I��sink
	double E = find_max_energy(R2, R3 - 1);
	double D = find_max_dts(R2, R3 - 1, CH1); //d1+d2���̤j��
	double MAX_s = CH_standard(pow(d1, 2) + pow(d2, 2), ns[R2].energy, E, D); //��d1+d2�ۥ[���̤p�� standard���N�O���Z���V��V�n ���ȶV�j�V�n
	for (int b = R2 + 1; b < R3; b++)
	{
		d1 = distance(CH1, b);
		d2 = distance(b, SINKID);
		double s = CH_standard(pow(d1, 2) + pow(d2, 2), ns[b].energy, E, D);
		if (MAX_s < s)
		{
			dst = b;
		}
	}

	double rate(0);/*���Y�v0.25*/
	
    for (int b = 0, a = 0; b < NODE_BUFFER1; b++)
    {
        if (ns[CH1].buffer[b].data == -1) //���Ū��N�����~��F
        {
            break;
        }
        int d = rand() % 100 + 1; /*?????????????????????*/
        if (d > successful_rate)
        {
            rate = b + 1;
            ns[dst].buffer[NODE_BUFFER1 + a].data = ns[CH1].buffer[b].data;
            ns[dst].buffer[NODE_BUFFER1 + a].dst = ns[CH1].buffer[b].dst;
            ns[dst].buffer[NODE_BUFFER1 + a].src = ns[CH1].buffer[b].src;
            ns[dst].buffer[NODE_BUFFER1 + a].time = ns[CH1].buffer[b].time;
            a++;
        }
        else
        {
            mac_drop++;
        }
    }
    clean(CH1, 0, NODE_BUFFER1);
	rate = ceil(rate * R);
	//fout << rate << endl;
	ns[CH1].energy -= (TransmitEnergy + pow(distance(CH1, dst), 2)*AmplifierEnergy)*rate; //�Ndata�X�֤���@���ǰe �ҥH�ӯ�o�˺�(�X�֥���)
																						  //fout << "node : " << CH1 << "��q��� "<<(TransmitEnergy + pow(distance(CH1, dst), 2)*AmplifierEnergy)*rate<<", �]���ǿ鵹�ϰ�2" << endl;
	ns[dst].energy -= (ReceiveEnergy)*rate;
	CH2Sink(dst);
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
void Reselection_judge(int s, int e, int region, int (&CH_record)[4][3])
{
	double avg_energy = find_avg_energy(s, e);
	if (((avg_energy - ns[ns[s].CH].energy) / avg_energy) >= 0.15) //�o�ӭȤ��@�w�j��0 , CH����O���@�w��P�� ! �]��������Y�����Y
	{
		CH_selection(s, e, region, CH_record);
	}
}
void CH_Reselection(int (&CH_record)[4][3])
{
	Reselection_judge(0, R2 - 1, 0, CH_record);
	Reselection_judge(R2, R3 - 1, 1, CH_record);
	Reselection_judge(R3, R4 - 1, 2, CH_record);
	Reselection_judge(R4, S_NUM - 1, 3, CH_record);
}
void transaction(int j, int t, int (&end)[4])
{
    Packet_Generate(j, t);
    if( ns[j].compress % 2 == 0){
        ns[j].delay = t - ns[j].delay;
        // fout << "Node " << j << " delay time: " << ns[j].delay << endl;
        Packet_Dliver(j, ns[j].CH);
        Packet_Receive(ns[j].CH);
    }
    else if( j == end[0] || j == end[1] || j == end[2] || j == end[3] ){
        Packet_Dliver(j, ns[j].CH);
        Packet_Receive(ns[j].CH);
    }
    else{
        ns[j].delay = t;
    }
}

/*�p�G�ϥθ�����Y  , �O�_�n�H��ƶq�j�p������ , �P�����ӯ�ۤ��CH���M����j�ܦh(�ϬM�bCH�Ѿl��q���M�٤񥭧���q��) due to dtc,��ҵ���,����CH��Ϥ��߷|���a���2�٭n����*/
/*code����ڤW�S�����Y*/
/*�ϰ�T�������ӯ�Ӱ�*/
/*�ؼ�:�X*/
/*���e,�P�_,�ĤG�h��traffic����*/
/*2���z������*/
/*CH�P�_�Ѽƫ���*/
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
			// node_deployed();
			special_node_deployed();
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
				int c = CheckEnergy();/*���@�Ӹ`�I�S�q�h���󦺤`*/
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
				

				if (t % CHf == 0) //�C�@�����Ǩ�sink 1��
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

