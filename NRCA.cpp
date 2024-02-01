#include <iostream>
#include <time.h>
#include <math.h>
#include <fstream>
#include <Queue>

#define MAX_energy 6480//j (1.5V*600mA*3600sec*2 = 6480j) 2*3���q��
#define SINKID 2000
#define SINK_X 400
#define SINK_Y 0
#define SINK_BUFFER_SIZE 5000000
#define NODE_BUFFER1 100 //0~49 �@��CH����CM�� node_buffer 40Kbytes (200��) ��F�o�ӰѼ� �U����bomb�]�n��
#define NODE_BUFFER2 200 //50~100 �S�O���ǿ��
/*�ܰʹ���ѼƳ]�w*/
#define S_NUM 900 //�P�����`��
#define R 0.25 //���Y�v �]1�h�S�����Y
#define type3f 360 //�`�Wsensing frequency
#define type4f 480
#define type5f 600
#define CHf 120 //CH trans frequency
#define freq_change_switch 0 //0�� 1�} �O�_�n�ϸ�ƶq��M�ɼW���}��
#define b_t 10800 //�jT �C�h�֬�}�@�� �pT �C�@���}�h�֬�
#define s_t 1800
#define bomb_f3 45 //�z��sensing frequency
#define bomb_f4 60
#define bomb_f5 90
/**/
#define ProbeEnergy 0.03 //j (8*200bit*1.5V*25mA*0.5mS = 0.00375j*8 = 0.03j) !
/*(�d�쪺�פ�:bit*50nj+bit*distance(m)����*100nj)*/
#define TransmitEnergy 0.00008 //0.000000001*50*200*8 j (50nj/bit by�a��) bit * 8 = byte !(�O�o�˺��)
#define AmplifierEnergy 0.00016 //100*200*0.000000001*8 j/distance^2
#define ReceiveEnergy 0.00008 //0.000000001*50*200j*8 (50nj/bit by�a��)
#define Package_size 200 //bytes 
#define node_buffer 20 //Kbytes (100��)
#define round_number 40
using namespace std;
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
ofstream fout("output.txt");
N ns[S_NUM];
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
int R2 = S_NUM / 4;
int R3 = S_NUM / 2;
int R4 = S_NUM * 0.75;
int R_NUM = R2;



double type_a = 33, type_b = 33, type_c = 34; //�վ�QUERE�̭��P����ƪ����
void print_energy()
{
	fout << ns[0].CH << " " << ns[R2].CH << " " << ns[R3].CH << " " << ns[R4].CH << endl;
	for (int i = 0; i < S_NUM; i++)
	{
		fout << "node" << ns[i].id << " x : " << ns[i].x << " y : " << ns[i].y << " type : " << ns[i].type << " energy : " << ns[i].energy << " dtc : " << ns[i].dtc << endl;
	}
	fout << "--------------------------------------------------\n";
}
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
	if (rate > 10 || sender == CH)  /*10% drop rate or CH�ۤv�Nsense���ʥ]��ۤv��buffer*/
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
			rate = b - NODE_BUFFER1 + 1;
			sink.buffer[start].data = ns[CH].buffer[b].data;
			sink.buffer[start].dst = ns[CH].buffer[b].dst;
			sink.buffer[start].src = ns[CH].buffer[b].src;
			sink.buffer[start].time = ns[CH].buffer[b].time;
			//fout << "CH ID:" << ns[CH].id << " src: " << sink.buffer[start].src << " dst: " << sink.buffer[start].dst << " data: " << sink.buffer[start].data << " time: " << sink.buffer[start].time << " sec" << endl;
			start++;
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
			rate = b + 1;
			sink.buffer[start].data = ns[CH].buffer[b].data;
			sink.buffer[start].dst = ns[CH].buffer[b].dst;
			sink.buffer[start].src = ns[CH].buffer[b].src;
			sink.buffer[start].time = ns[CH].buffer[b].time;
			//fout << "CH ID:" << ns[CH].id << " src: " << sink.buffer[start].src << " dst: " << sink.buffer[start].dst << " data: " << sink.buffer[start].data << " time: " << sink.buffer[start].time << " sec" << endl;
			start++;
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
			rate = b + 1;
			ns[dst].buffer[NODE_BUFFER1 + b].data = ns[CH1].buffer[b].data;
			ns[dst].buffer[NODE_BUFFER1 + b].dst = ns[CH1].buffer[b].dst;
			ns[dst].buffer[NODE_BUFFER1 + b].src = ns[CH1].buffer[b].src;
			ns[dst].buffer[NODE_BUFFER1 + b].time = ns[CH1].buffer[b].time;
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
			rate = b + 1 - NODE_BUFFER1;
			ns[dst].buffer[b].data = ns[CH1].buffer[b].data;
			ns[dst].buffer[b].dst = ns[CH1].buffer[b].dst;
			ns[dst].buffer[b].src = ns[CH1].buffer[b].src;
			ns[dst].buffer[b].time = ns[CH1].buffer[b].time;
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
void Reselection_judge(int s, int e, int rnum)
{
	double t; //threshold
	double avg_d(0.0);
	double avg_e(0.0);
	int CH = ns[s].CH;
	for (int i = s; i <= e; i++)
	{
		avg_d += ns[i].dtc;
		avg_e += ns[i].energy;
	}
	avg_d /= rnum;
	avg_e /= rnum;
	t = (ns[CH].energy * avg_d) / (avg_e * ns[CH].dtc);
	cout << t << endl;
}
void CH_Reselection()
{
	Reselection_judge(0, R2 - 1, R_NUM);
	Reselection_judge(R2, R3 - 1, R_NUM);
	Reselection_judge(R3, R4 - 1, R_NUM);
	Reselection_judge(R4, S_NUM - 1, R_NUM);
}

double threshold(int s, int e, int rnum)
{
	double t; //threshold
	double avg_d(0.0);
	double avg_e(0.0);
	int CH = ns[s].CH;
	for (int i = s; i <= e; i++)
	{
		avg_d += ns[i].dtc;
		avg_e += ns[i].energy;
	}
	avg_d /= rnum;
	avg_e /= rnum;
	t = (ns[CH].energy * avg_d) / (avg_e * ns[CH].dtc);
	return t;
}
void transaction(int j, int t)
{
	Packet_Generate(j, t);
	Packet_Dliver(j, ns[j].CH);
	Packet_Receive(ns[j].CH);
}
double standard_deviation()
{
	double a(0);
	double b(0);
	double sd(0);
	for (int i = 0; i < S_NUM; i++)
	{
		b += ns[i].energy;
	}

	b /= S_NUM;
	return b;
}
int main()
{
	/*sensor initialization*/
	srand((unsigned)time(NULL)); //random seed
	for (int round = 0; round < round_number; round++)
	{
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
		packet_init();

		/*sink initialization*/
		sink.id = SINKID;
		for (int b = 0; b < SINK_BUFFER_SIZE; b++)
		{
			sink.buffer[b].data = -1;
			sink.buffer[b].dst = -1;
			sink.buffer[b].src = -1;
			sink.buffer[b].time = -1;
		}

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
				/*double avg_re = standard_deviation();
				fout << t << "  " << avg_re << endl;*/
				//fout << t << endl;
				avg_t += t;
				//fout << "node " << c << " dead !" << endl;
				die = 1;
				//print_energy();
				/*for (int i = 0; i < 4; i++)
				{
				//fout << "�ϰ�" << i + 1 << "���ϰ줺�����ǿ�ӯ� = " << cons[i] / trans_time[i] << endl;
				}
				for (int i = 0; i < 4; i++)
				{
				//fout << "�ϰ�" << i + 1 << "�������ǿ��2�ӯ� = " << consToR2[i] / ToR2_time[i] << endl;
				}*/
				/*int e(0);
				while (sink.buffer[e].data != -1)
				{
				//fout << "src: " << sink.buffer[e].src << " dst: " << sink.buffer[e].dst << " data: " << sink.buffer[e].data << " time: " << sink.buffer[e].time << " sec" << endl;
				e++;
				}
				fout << "total = " << total << endl;
				fout << "drop = " << drop << endl;
				fout << "macdrop = " << macdrop << endl;
				fout << "sink ��" << e << endl; //total�ʥ]��*/
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
	fout << "avg_time : " << avg_t << endl;
	fout << "avg_total : " << total << endl;
	fout << "avg_macdrop : " << macdrop << endl;
	fout << "avg_drop : " << drop << endl;
	fout << "avg_PLR : " << (drop + macdrop) / total << endl;
	//system("PAUSE");
	return 0;
}

