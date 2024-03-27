#include <iostream>
#include <time.h>
#include <math.h>
#include <fstream>
#include <queue>

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
#define freq_change_switch 0 //0�� 1�} �O�_�n�ϸ�ƶq��M�ɼW���}��
#define b_t 10800 //�jT �C�h�֬�}�@�� �pT �C�@���}�h�֬�
#define s_t 1800
#define bomb_f3 45 //�z��sensing frequency
#define bomb_f4 60
#define bomb_f5 90

#define ProbeEnergy 0.03 //j (8*200bit*1.5V*25mA*0.5mS = 0.00375j*8 = 0.03j) !
/*(�d�쪺�פ�:bit*50nj+bit*distance(m)����*100nj)*/
#define TransmitEnergy 0.00008 //0.000000001*50*200*8 j (50nj/bit by�a��) bit * 8 = byte !(�O�o�˺��)
#define AmplifierEnergy 0.00016 //100*200*0.000000001*8 j/distance^2
#define ReceiveEnergy 0.00008 //0.000000001*50*200j*8 (50nj/bit by�a��)
#define Package_size 200 //bytes 
#define node_buffer 20 //Kbytes (100��)
#define successful_rate 5 //�]x ���\�v�N�O100-x%

/*�ܰʹ���ѼƳ]�w*/
#define round_number 10
#define E_NUM 1000

using namespace std;

int S_NUM = 200;
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
	int id, x, y, CH, type, region1;//region1 for �Ĥ@�hgrid , region2 for �ĤG�hgrid
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
ofstream fout("normal_NRCA.txt");
N ns[2000];
S sink;
double avg_t(0);
double drop(0);
double macdrop(0);
double total(0);
double cons[4] = { 0,0,0,0 };
int trans_time[4] = { 0,0,0,0 };
double consToR2[4] = { 0,0,0,0 };
int ToR2_time[4] = { 0,0,0,0 };
int toSink(0);
double SD(0);
int R2, R3, R4;

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
		ns[i].dtc = distance(i, SINKID);/*�Z����SINK*/
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
		ns[i].dtc = distance(i, SINKID); /*�Z��sink*/
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
		ns[i].dtc = distance(i, SINKID);/*�Z����SINKID*/
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
		ns[i].dtc = distance(i, SINKID);/*�Z����SINKID*/
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
		ns[i].dtc = distance(i, SINKID);/*�Z����SINK*/
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
		ns[i].dtc = distance(i, SINKID); /*�Z��sink*/
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
		ns[i].dtc = distance(i, SINKID);/*�Z����SINKID*/
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
		ns[i].dtc = distance(i, SINKID);/*�Z����SINKID*/
		ns[i].region1 = 4;
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

void CH_Selection(int s, int e) //s=start e=end
{
	double E = find_max_energy(s, e);
	int start = s;
	int end = e;
	queue<int> CH_cdd; //cdd candidate
	int CH;
	for (int i = s; i <= e; i++)//selecting
	{
		if (ns[i].energy == E)
		{
			CH_cdd.push(i);
		}
	}
	CH = CH_cdd.front();
	CH_cdd.pop();
	while (!CH_cdd.empty())
	{
		if (ns[CH_cdd.front()].dtc < ns[CH].dtc)
		{
			CH = CH_cdd.front();
		}
		CH_cdd.pop();
	}
	for (start; start <= end; start++)//start to change CH
	{
		ns[start].CH = CH;
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
	//fout << "node : " << now << "��q��� "<< ProbeEnergy <<" ,�]�����ͷP���ʥ] "<< endl;
}
void Packet_Dliver(int sender, int CH) // �����
{
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
		//fout << "node : " << sender << "��q��� "<< TransmitEnergy + d*d*AmplifierEnergy <<" ,�]���ǰe���`�I " << CH << " �P���ʥ]" << endl;
	} //1�ӫʥ]
}
void Packet_Receive(int CH) //buffer���F�n�ܦ�priority queue �����
{
	if (ns[CH].receive.src != CH) //���O�Ӧۦۤv���~�n����q
	{
		ns[CH].energy -= ReceiveEnergy;
		//fout << "node : " << CH << "��q��� "<< ReceiveEnergy << " ,�]������Ӧ۸`�I " << ns[CH].receive.src << " ���ʥ]" << endl;
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
			/*clean*/
			break;
		}
	}
	if (full == 1) //priority queue buffer , �p�G�ʥ]�Qdrop���N���ΤF(-1)
	{
		drop++;
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
			int d = rand() % 100 + 1;
			if (d > successful_rate)
			{
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
void CHtoRegion2(int CH1, int v) //���F2�ϥH�~���ϰ쳣���2�ϸ̭���q�̰��� �����
{
	/*��CH��2��+2�Ϩ�sink���Z���ۥ[�P��Ѿl��q�Ȱ��[�v*/
	int dst = ns[R2].CH;
	double rate(0);/*���Y�v0.25*/
	if (v == 1)
	{
		for (int b = 0; b < NODE_BUFFER1; b++)
		{
			if (ns[CH1].buffer[b].data == -1) //���Ū��N�����~��F
			{
				break;
			}
			int d = rand() % 100 + 1; /*?????????????????????*/
			if (d > successful_rate)
			{
				rate = b + 1;
				ns[dst].buffer[NODE_BUFFER1 + b].data = ns[CH1].buffer[b].data;
				ns[dst].buffer[NODE_BUFFER1 + b].dst = ns[CH1].buffer[b].dst;
				ns[dst].buffer[NODE_BUFFER1 + b].src = ns[CH1].buffer[b].src;
				ns[dst].buffer[NODE_BUFFER1 + b].time = ns[CH1].buffer[b].time;
			}
			else
			{
				macdrop++;
			}
		}
		clean(CH1, 0, NODE_BUFFER1);
		//fout <<"�Ҧ��@����" <<rate << endl;
	}
	if (v == 2)
	{
		for (int b = NODE_BUFFER1; b < NODE_BUFFER2; b++)
		{
			if (ns[CH1].buffer[b].data == -1) //���Ū��N�����~��F
			{
				break;
			}
			int d = rand() % 100 + 1; /*?????????????????????*/
			if (d > successful_rate)
			{
				rate = b + 1 - NODE_BUFFER1;
				ns[dst].buffer[b].data = ns[CH1].buffer[b].data;
				ns[dst].buffer[b].dst = ns[CH1].buffer[b].dst;
				ns[dst].buffer[b].src = ns[CH1].buffer[b].src;
				ns[dst].buffer[b].time = ns[CH1].buffer[b].time;
			}
			else
			{
				macdrop++;
			}
		}
		clean(CH1, NODE_BUFFER1, NODE_BUFFER2);
		//fout << "�Ҧ��G����" << rate << endl;
	}
	rate = ceil(rate * R);
	ns[CH1].energy -= (TransmitEnergy + pow(distance(CH1, dst), 2)*AmplifierEnergy)*rate; //�Ndata�X�֤���@���ǰe �ҥH�ӯ�o�˺�(�X�֥���)
																						  //fout << "node : " << CH1 << "��q��� "<<(TransmitEnergy + pow(distance(CH1, dst), 2)*AmplifierEnergy)*rate<<", �]���ǿ鵹�ϰ�2" << endl;
	ns[dst].energy -= (ReceiveEnergy)*rate;
	//fout << "node : " << dst << "��q��� "<< (ReceiveEnergy)*rate<<" ,�]���b�ϰ�2����O�����" << endl;
	//fout <<"�ڬO�`�I "<< dst << " ����O�H��" << endl;
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

void transaction(int j, int t)
{
	Packet_Generate(j, t);
	Packet_Dliver(j, ns[j].CH);
	Packet_Receive(ns[j].CH);
}

int main(){
	srand((unsigned)time(NULL)); //random seed
	fout << "NRCA" << endl;
	for( S_NUM ; S_NUM <= E_NUM ; S_NUM += 100){
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

			/*firts CH selection*/
			CH_Selection(0, R2 - 1);
			CH_Selection(R2, R3 - 1);
			CH_Selection(R3, R4 - 1);
			CH_Selection(R4, S_NUM - 1);
			int countround[4] = { 0,0,0,0 };
			/*traffic start*/
			int bombing(0);
			int b_region(0);
			int die(0);
			int t(1);
			while (!die)
			{
				/*if (t % 15000 == 0)
				{
					double avg_re = standard_deviation();
					fout << avg_re << endl;
				}*/
				if (countround[0] == 0)
					countround[0] = 10;
				if (countround[1] == 0)
					countround[1] = 3;
				if (countround[2] == 0)
					countround[2] = 10;
				if (countround[3] == 0)
					countround[3] = 10;
				//fout << "time = " << t << endl;
				int c = CheckEnergy();/*���@�Ӹ`�I�S�q�h���󦺤`*/
				if (c < SINKID)
				{
					avg_t += t;
					die = 1;
					break;
				}
				if (freq_change_switch)
				{
					if (t % b_t <= s_t) //if in this time slot , then bombing.
					{
						if (b_region == 0)
						{
							b_region = rand() % 4 + 1; //bombing region 1~4 !
						}
						bombing = 1;
					}
					else //�����z�� ,�զ^�Ѽ�
					{
						b_region = 0;
						bombing = 0;
					}
				}
				if (bombing)
				{
					/*�z����*/
					if (t % bomb_f3 == 0)
					{
						for (int j = 0; j < S_NUM; j++)
						{
							if (ns[j].type == 3 && ns[j].region1 == b_region)
							{
								transaction(j, t);
							}
						}
					}
					if (t % bomb_f4 == 0)
					{
						for (int j = 0; j < S_NUM; j++)
						{
							if (ns[j].type == 4 && ns[j].region1 == b_region)
							{
								transaction(j, t);
							}
						}
					}
					if (t % bomb_f5 == 0)
					{
						for (int j = 0; j < S_NUM; j++)
						{
							if (ns[j].type == 5 && ns[j].region1 == b_region)
							{
								transaction(j, t);
							}
						}
					}

					/*�D�z����*/
					if (t % type3f == 0)
					{
						for (int j = 0; j < S_NUM; j++)
						{
							if (ns[j].type == 3 && ns[j].region1 != b_region)
							{
								transaction(j, t);
							}
						}
					}
					if (t % type4f == 0)
					{
						for (int j = 0; j < S_NUM; j++)
						{
							if (ns[j].type == 4 && ns[j].region1 != b_region)
							{
								transaction(j, t);
							}
						}
					}
					if (t % type5f == 0)
					{
						for (int j = 0; j < S_NUM; j++)
						{
							if (ns[j].type == 5 && ns[j].region1 != b_region)
							{
								transaction(j, t);
							}
						}
					}
				}
				else//regular trans
				{
					if (t % type3f == 0)
					{
						for (int j = 0; j < S_NUM; j++)
						{
							if (ns[j].type == 3) //CH need to sense
							{
								transaction(j, t);
							}
						}
					}
					if (t % type4f == 0)
					{
						for (int j = 0; j < S_NUM; j++)
						{
							if (ns[j].type == 4) //CH need to sense
							{
								transaction(j, t);
							}
						}
					}
					if (t % type5f == 0)
					{
						for (int j = 0; j < S_NUM; j++)
						{
							if (ns[j].type == 5) //CH need to sense
							{
								transaction(j, t);
							}
						}
					}
				}

				if (t % CHf == 0) //�C�@�����Ǩ�sink 1��
				{
					int CH[4];
					CH[0] = ns[0].CH;
					CH[1] = ns[R2].CH;
					CH[2] = ns[R3].CH;
					CH[3] = ns[R4].CH;

					CH2Sink(CH[1]);
					CHtoRegion2(CH[0], 1);
					CHtoRegion2(CH[2], 1);
					CHtoRegion2(CH[3], 1);
					//CH_Reselection();

					countround[0]--;
					if (countround[0] == 0) { CH_Selection(0, R2 - 1); }
					countround[1]--;
					if (countround[1] == 0) { CH_Selection(R2, R3 - 1); }
					countround[2]--;
					if (countround[2] == 0) { CH_Selection(R3, R4 - 1); }
					countround[3]--;
					if (countround[3] == 0) { CH_Selection(R4, S_NUM - 1); }
				}
				t++;
			}
		}
		total /= round_number;
		macdrop /= round_number;
		drop /= round_number;
		avg_t /= round_number;
		fout << "avg_lifetime : " << avg_t << endl;
		fout << "avg_total : " << total << endl;
		fout << "avg_macdrop : " << macdrop << endl;
		fout << "avg_drop : " << drop << endl;
		fout << "avg_PLR : " << (drop + macdrop) / total << endl;
	}
	return 0;
}
