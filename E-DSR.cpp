#include"common.h"
/*�ܰʹ���ѼƳ]�w*/
#define S_NUM 600 //�P�����`��
#define R 0.25 //���Y�v �]1�h�S�����Y
#define type3f 360//�`�Wsensing frequency
#define type4f 480
#define type5f 720 //720
#define reservation_energy_time 10000
#define CHf 120 //CH trans frequency
#define bomb_switch 1 //0�� 1�} �O�_�n�ϥ����h���}��
#define freq_change_switch 1 //0�� 1�} �O�_�n�ϸ�ƶq��M�ɼW���}��
#define b_t 10800 //�jT �C�h�֬�}�@�� �pT �C�@���}�h�֬�
#define s_t 1800
#define bomb_f3 45 //�z��sensing frequency
#define bomb_f4 60
#define bomb_f5 90
#define full_th 2
/**/
#define successful_rate 5 //�]x ���\�v�N�O100-x%
using namespace std;
struct C{
	double x, y;
};

struct Node
{
	int id, x, y, CH, type, region1, region2;//region1 for �Ĥ@�hgrid , region2 for �ĤG�hgrid
	int CH2;
	double energy;//node information
	Package receive;
	Package sense;
	Package buffer[NODE_BUFFER2];//buffer in sensor node
	double dtc;//dist = distance to sink
};

ofstream fout("E-DSR_output.txt");
Node node[S_NUM];
Sink sink;
double p_in_sink(0);
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
void print_energy(){
	fout << node[0].CH << " " << node[R2].CH << " " << node[R3].CH << " " << node[R4].CH << endl;
	for (int i = 0; i < S_NUM; i++)	{
		fout << "node" << node[i].id << " x : " << node[i].x << " y : " << node[i].y << " type : " << node[i].type << " energy : " << node[i].energy << " dtc : " << node[i].dtc << endl;
	}
	fout << "--------------------------------------------------\n";
}
double distance(int a, int b){
	if (b != SINKID){
		return sqrt(pow(abs(node[a].x - node[b].x), 2) + pow(abs(node[a].y - node[b].y), 2));
	}
	else{ //SINK
		return sqrt(pow(abs(node[a].x - SINK_X), 2) + pow(abs(node[a].y - SINK_Y), 2));
	}
}
void packet_init(){
	for (int a = 0; a < S_NUM; a++)	{
		node[a].receive.data = -1;
		node[a].receive.dst = -1;
		node[a].receive.src = -1;
		node[a].receive.time = -1;
		node[a].sense.data = -1;
		node[a].sense.dst = -1;
		node[a].sense.src = -1;
		node[a].sense.time = -1;
		for (int b = 0; b < NODE_BUFFER2; b++)
		{
			node[a].buffer[b].data = -1;
			node[a].buffer[b].dst = -1;
			node[a].buffer[b].src = -1;
			node[a].buffer[b].time = -1;
		}
	}
}

//����z��
int bomb_times[4] = { 0,0,0,0 };
int normal_times[4] = { 0,0,0,0 };
int old_t[4] = { 0,0,0,0 };
int current_state[4] = { 0,0,0,0 };
int g2[4][4] = { { -1,-1,-1,-1 },{ -1,-1,-1,-1 },{ -1,-1,-1,-1 },{ -1,-1,-1,-1 } }; //�ΨӦs��ĤG�h��CH

void sec_ch(int r1, int r2)
{
	double max(0); //�ثe����Ѿl��q�̤j��
	int ch2(-1);//�o�̥�-1�O�]�����ɭԳo�Ӱϰ�ڥ��N�S���I
	for (int i = 0; i < S_NUM; i++)
	{
		if (node[i].region1 == r1 && node[i].region2 == r2)
		{
			if (max < node[i].energy)
			{
				max = node[i].energy;
				ch2 = i;
			}
		}
	}
	g2[r1 - 1][r2 - 1] = ch2; //�x�s2�h�Y
							  //fout << "��" << r1 << "�Ϥ�����" << r2 << "�Ϫ��Y�O" << ch2 << endl;
	for (int i = 0; i < S_NUM; i++)
	{
		if (node[i].region1 == r1 && node[i].region2 == r2)
		{
			node[i].CH2 = ch2;
		}
	}
}
void sec_ch_re_check(int r1, int r2) //��2��CH�������
{
	double avg_re(0);//���p�Ϫ�������q
	double r2_num(0);//�o�Ӥp�ϸ̭��@���X���I
	for (int i = 0; i < S_NUM; i++)
	{
		if (node[i].region1 == r1 && node[i].region2 == r2)
		{
			avg_re += node[i].energy;
			r2_num++;
		}
	}
	if (r2_num != 0) //��X�o�Ӥp�Ϫ�������q
	{
		avg_re /= r2_num;
	}
	if (node[g2[r1 - 1][r2 - 1]].energy < avg_re)
	{
		sec_ch(r1, r2);
	}
}
void sec_grid(int r)  // r�O�����@��,(2�ϫ��)    /*�M�w�O���@�ϵM���CH*/
{
	//cout << r << "�϶}" << endl;
	if (r == 1)
	{
		for (int i = 1; i <= 4; i++)
		{
			sec_ch(1, i);
		}
	}
	if (r == 2)
	{
		for (int i = 1; i <= 4; i++)
		{
			sec_ch(2, i);
		}
	}
	if (r == 3)
	{
		for (int i = 1; i <= 4; i++)
		{
			sec_ch(3, i);
		}
	}
	if (r == 4)
	{
		for (int i = 1; i <= 4; i++)
		{
			sec_ch(4, i);
		}
	}
}
int Packet_num(int CH)
{
	int count(0);
	for (int i = 0; i < NODE_BUFFER1; i++)
	{
		if (node[CH].buffer[i].data != -1)
		{
			count++;
		}
	}
	return count;
}
void bomb_cancel(int region1)
{
	//cout << region1 << "����" << endl;
	for (int i = 0; i < S_NUM; i++)
	{
		if (node[i].region1 == region1)
		{
			node[i].CH2 = -1;
		}
	}
	for (int i = 0; i < 4; i++) //��^-1
	{
		g2[region1 - 1][i] = -1;
	}
}


double standard(double a, double b, double ENERGY_STANDARD, double DIST_STANDARD) //used to select CH !�o�ӿ�ܤ覡�ثe�O�y����ӹ����q���Ӥ����Ū���](�O�_�n�Ҽ{���ӯ�q���j�p) !��J�`�I�����Ѧ�?
{
	double s = 0.3*(1 - a / DIST_STANDARD) + 0.7*(b / ENERGY_STANDARD); //larger is better
	return s;
}
double find_max_energy(int s, int e) //!energy���w��
{
	double max(0);
	for (int i = s; i <= e; i++)
	{
		if (max < node[i].energy)
		{
			max = node[i].energy;
		}
	}
	return max;
}
double find_avg_energy(int s, int e, int rnum)
{
	double avg_energy(0.0);
	for (int i = s; i <= e; i++)
	{
		avg_energy += node[i].energy;
	}
	avg_energy /= rnum;
	return avg_energy;
}
C find_center(int s, int e) //�M��ϰ줤�Ҧ��I�����ߺɶq�����Z��
{
	C c;
	c.x = 0.0; c.y = 0.0;
	for (int i = s; i <= e; i++)
	{
		c.x += node[i].x;
		c.y += node[i].y;
	}
	int n = e - s + 1; //n=�ϰ�`�I��
	c.x /= n;
	c.y /= n;
	return c;
}
double find_max_distance(int s, int e)
{
	double d(0.0);
	for (int i = s; i <= e; i++)
	{
		if (d < node[i].dtc)
		{
			d = node[i].dtc;
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
void set_dtc(C c, int s, int e)
{
	for (int i = s; i <= e; i++)
	{
		node[i].dtc = sqrt(pow(abs(node[i].x - c.x), 2) + pow(abs(node[i].y - c.y), 2));
	}
}

void CH_Selection(int s, int e) //s=start e=end !energy���w��
{
	double E = find_max_energy(s, e);
	double D = find_max_distance(s, e);
	int start = s;
	int end = e;
	int CH = s;
	double re = node[s].energy - (floor(reservation_energy_time / (node[s].type * 120))*(ProbeEnergy + (TransmitEnergy + pow(distance(s, node[s].CH), 2)*AmplifierEnergy)));//�����w����q�Ӥ�|�������,�N��O�t�����Ӥ]�వ�P�_ 120�O������
	double MAX_S = standard(node[s].dtc, re, E, D);
	s += 1;
	for (s; s <= e; s++)//selecting
	{
		re = node[s].energy - (floor(reservation_energy_time / (node[s].type * 120))*(ProbeEnergy + (TransmitEnergy + pow(distance(s, node[s].CH), 2)*AmplifierEnergy)));//�����w����q�Ӥ�|�������,�N��O�t�����Ӥ]�వ�P�_
		double current_s = standard(node[s].dtc, re, E, D);
		if (MAX_S < current_s)
		{
			MAX_S = current_s;
			CH = node[s].id;
		}
	}
	for (start; start <= end; start++)//start to change CH
	{
		node[start].CH = CH;
	}
	//fout << "CH change to " << CH << endl;
}

void Packet_Generate(int now, int t) //generate packet �����
{
	total++;
	node[now].sense.src = node[now].id;
	node[now].sense.dst = node[now].CH;
	node[now].sense.data = node[now].type;
	node[now].sense.time = t;
	node[now].energy -= ProbeEnergy;
	//fout << "node : " << now << "��q��� "<< ProbeEnergy <<" ,�]�����ͷP���ʥ] "<< endl;
}
void Packet_Dliver(int sender, int CH) // �����
{
	int rate = rand() % 100 + 1;
	if (rate > successful_rate || sender == CH)  /*10% drop rate or CH�ۤv�Nsense���ʥ]��ۤv��buffer*/
	{
		node[CH].receive.dst = node[sender].sense.dst;
		node[CH].receive.src = node[sender].sense.src;
		node[CH].receive.data = node[sender].sense.data;
		node[CH].receive.time = node[sender].sense.time;
		//fout << "node id: "<<node[CH].id<<" receive the packet type "<<sender.type <<" from node "<<sender.id << " at " << node[CH].receive.time<<" sec"<< endl;
	}
	else
	{
		macdrop++;
		node[CH].receive.dst = -1;
		node[CH].receive.src = -1;
		node[CH].receive.data = -1;
		node[CH].receive.time = -1;
	}
	double d = distance(sender, CH);
	if (sender != CH) //CH�ۤv���ۤv���Φ���q
	{
		node[sender].energy -= TransmitEnergy + d*d*AmplifierEnergy;
		cons[node[sender].region1 - 1] += TransmitEnergy + d*d*AmplifierEnergy;
		trans_time[node[sender].region1 - 1] += 1;
		//fout << "node : " << sender << "��q��� "<< TransmitEnergy + d*d*AmplifierEnergy <<" ,�]���ǰe���`�I " << CH << " �P���ʥ]" << endl;
	} //1�ӫʥ]
}

void Packet_Receive(int CH) //buffer���F�n�ܦ�priority queue �����
{
	if (node[CH].receive.src != CH) //���O�Ӧۦۤv���~�n����q
	{
		node[CH].energy -= ReceiveEnergy;
		//fout << "node : " << CH << "��q��� "<< ReceiveEnergy << " ,�]������Ӧ۸`�I " << node[CH].receive.src << " ���ʥ]" << endl;
	} //drop�ٺ�O����
	int full(1);
	for (int a = 0; a < NODE_BUFFER1; a++) //buffer is not full 50 for self-area-sense 50 for other CH 
	{
		if (node[CH].buffer[a].data == -1)
		{
			node[CH].buffer[a].dst = node[CH].receive.dst;
			node[CH].buffer[a].src = node[CH].receive.src;
			node[CH].buffer[a].data = node[CH].receive.data;
			node[CH].buffer[a].time = node[CH].receive.time;
			full = 0;
			break;
		}
	}
	if (full == 1 && node[CH].receive.data != -1)//priority queue buffer , �p�G�ʥ]�Qdrop���N���ΤF(-1)
	{
		drop++;
	}
	node[CH].receive.dst = -1;
	node[CH].receive.src = -1;
	node[CH].receive.data = -1;
	node[CH].receive.time = -1;
}
void clean(int CH, int start, int end)
{
	for (start; start < end; start++)
	{
		node[CH].buffer[start].data = -1;
		node[CH].buffer[start].dst = -1;
		node[CH].buffer[start].src = -1;
		node[CH].buffer[start].time = -1;
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
	if (node[CH].buffer[NODE_BUFFER1].data != -1)/*���O��CH��*/
	{
		double rate(0);/*���Y�v0.25*/
		for (int b = NODE_BUFFER1; b < NODE_BUFFER2; b++)
		{
			if (node[CH].buffer[b].data == -1)  //���Ū��N�����~��F
			{
				break;
			}
			int d = rand() % 100 + 1; /*?????????????????????*/
			if (d > successful_rate)
			{
				rate = b - NODE_BUFFER1 + 1;
				sink.buffer[start].data = node[CH].buffer[b].data;
				sink.buffer[start].dst = node[CH].buffer[b].dst;
				sink.buffer[start].src = node[CH].buffer[b].src;
				sink.buffer[start].time = node[CH].buffer[b].time;
				start++;
			}
			else
			{
				macdrop++;
			}
			//fout << "CH ID:" << node[CH].id << " src: " << sink.buffer[start].src << " dst: " << sink.buffer[start].dst << " data: " << sink.buffer[start].data << " time: " << sink.buffer[start].time << " sec" << endl;
		}
		//fout << "sink����" << rate << "��" << endl;
		rate = ceil(rate * R);
		//fout << rate << endl;
		node[CH].energy -= (TransmitEnergy + pow(distance(CH, SINKID), 2)*AmplifierEnergy)*rate; //�Ndata�X�֤���@���ǰe �ҥH�ӯ�o�˺�(�X�֥���) �]���i���X�֪�size���bpacket���j�p����
																							   //fout << "���O�H���\,�ڪ���q�u��" << node[CH].energy << endl;
																							   //fout << "node : " << CH << "��q��� "<< (TransmitEnergy + pow(distance(CH, SINKID), 2)*AmplifierEnergy)*rate <<" ,�]�����O�H�ǵ�sink" << endl;
		clean(CH, NODE_BUFFER1, NODE_BUFFER2); /*�ǧ�����R����*/
	}
	else      /*�ۤv��*/
	{
		double rate(0);/*���Y�v0.25*/
		for (int b = 0; b < NODE_BUFFER1; b++)
		{
			if (node[CH].buffer[b].data == -1) //���Ū��N�����~��F
			{
				break;
			}
			int d = rand() % 100 + 1; /*?????????????????????*/
			if (d > successful_rate)
			{
				rate = b + 1;
				sink.buffer[start].data = node[CH].buffer[b].data;
				sink.buffer[start].dst = node[CH].buffer[b].dst;
				sink.buffer[start].src = node[CH].buffer[b].src;
				sink.buffer[start].time = node[CH].buffer[b].time;
				//fout << "CH ID:" << node[CH].id << " src: " << sink.buffer[start].src << " dst: " << sink.buffer[start].dst << " data: " << sink.buffer[start].data << " time: " << sink.buffer[start].time << " sec" << endl;
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
		node[CH].energy -= (TransmitEnergy + pow(distance(CH, SINKID), 2)*AmplifierEnergy)*rate; //�Ndata�X�֤���@���ǰe �ҥH�ӯ�o�˺�(�X�֥���)
																							   //fout << "�ۤv��,�ڪ���q�u��" << node[CH].energy << endl;
																							   //fout << "node : " << CH << "��q��� "<< (TransmitEnergy + pow(distance(CH, SINKID), 2)*AmplifierEnergy)*rate <<" ,�]�����ۤv�ǵ�sink" << endl;
		clean(CH, 0, NODE_BUFFER1); /*�ǧ�����R����*/
	}
}
void CHtoRegion2(int CH1, int v) //���F2�ϥH�~���ϰ쳣���2�ϸ̭���q�̰��� �����
{
	/*��CH��2��+2�Ϩ�sink���Z���ۥ[�P��Ѿl��q�Ȱ��[�v*/
	int dst = R2;
	double d1 = distance(CH1, R2); //�O�Ϩ�Y�I
	double d2 = distance(R2, SINKID);  //�Y�I��sink
	double E = find_max_energy(R2, R3 - 1);
	double D = find_max_dts(R2, R3 - 1, CH1); //d1+d2���̤j��
	double MAX_s = standard(pow(d1, 2) + pow(d2, 2), node[R2].energy, E, D); //��d1+d2�ۥ[���̤p�� standard���N�O���Z���V��V�n ���ȶV�j�V�n
	for (int b = R2 + 1; b < R3; b++)
	{
		d1 = distance(CH1, b);
		d2 = distance(b, SINKID);
		double s = standard(pow(d1, 2) + pow(d2, 2), node[b].energy, E, D);
		if (MAX_s < s)
		{
			dst = b;
		}
	}

	double rate(0);/*���Y�v0.25*/
	if (v == 1)
	{
		for (int b = 0, a = 0; b < NODE_BUFFER1; b++)
		{
			if (node[CH1].buffer[b].data == -1) //���Ū��N�����~��F
			{
				break;
			}
			int d = rand() % 100 + 1; /*?????????????????????*/
			if (d > successful_rate)
			{
				rate = b + 1;
				node[dst].buffer[NODE_BUFFER1 + a].data = node[CH1].buffer[b].data;
				node[dst].buffer[NODE_BUFFER1 + a].dst = node[CH1].buffer[b].dst;
				node[dst].buffer[NODE_BUFFER1 + a].src = node[CH1].buffer[b].src;
				node[dst].buffer[NODE_BUFFER1 + a].time = node[CH1].buffer[b].time;
				a++;
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
		for (int b = NODE_BUFFER1, a = NODE_BUFFER1; b < NODE_BUFFER2; b++)
		{
			if (node[CH1].buffer[b].data == -1) //���Ū��N�����~��F
			{
				break;
			}
			int d = rand() % 100 + 1; /*?????????????????????*/
			if (d > successful_rate)
			{
				rate = b + 1 - NODE_BUFFER1;
				node[dst].buffer[a].data = node[CH1].buffer[b].data;
				node[dst].buffer[a].dst = node[CH1].buffer[b].dst;
				node[dst].buffer[a].src = node[CH1].buffer[b].src;
				node[dst].buffer[a].time = node[CH1].buffer[b].time;
				a++;
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
	//fout << rate << endl;
	node[CH1].energy -= (TransmitEnergy + pow(distance(CH1, dst), 2)*AmplifierEnergy)*rate; //�Ndata�X�֤���@���ǰe �ҥH�ӯ�o�˺�(�X�֥���)
																						  //fout << "node : " << CH1 << "��q��� "<<(TransmitEnergy + pow(distance(CH1, dst), 2)*AmplifierEnergy)*rate<<", �]���ǿ鵹�ϰ�2" << endl;
	consToR2[node[CH1].region1 - 1] += (TransmitEnergy + pow(distance(CH1, dst), 2)*AmplifierEnergy)*rate;
	ToR2_time[node[CH1].region1 - 1] += 1;
	node[dst].energy -= (ReceiveEnergy)*rate;
	//fout << "node : " << dst << "��q��� "<< (ReceiveEnergy)*rate<<" ,�]���b�ϰ�2����O�����" << endl;
	//fout <<"�ڬO�`�I "<< dst << " ����O�H��" << endl;
	CH2Sink(dst);
}
void g2toCH(int CH1, int CH2) //CH1 �ǰe CH2 ��
{
	double rate(0);/*���Y�v0.25*/
	for (int b = 0, a = 0; b < NODE_BUFFER1; b++)
	{
		if (node[CH1].buffer[b].data == -1) //���Ū��N�����~��F
		{
			break;
		}
		int d = rand() % 100 + 1; /*?????????????????????*/
		if (d > successful_rate)
		{
			rate = b + 1;
			node[CH2].buffer[NODE_BUFFER1 + a].data = node[CH1].buffer[b].data;
			node[CH2].buffer[NODE_BUFFER1 + a].dst = node[CH1].buffer[b].dst;
			node[CH2].buffer[NODE_BUFFER1 + a].src = node[CH1].buffer[b].src;
			node[CH2].buffer[NODE_BUFFER1 + a].time = node[CH1].buffer[b].time;
			a++;
		}
		else
		{
			macdrop++;
		}
	}
	rate = ceil(rate * R);
	//fout << rate << endl;
	node[CH1].energy -= (TransmitEnergy + pow(distance(CH1, CH2), 2)*AmplifierEnergy)*rate; //�Ndata�X�֤���@���ǰe �ҥH�ӯ�o�˺�(�X�֥���)
																						  //fout << "node : " << CH1 << "��q��� " << (TransmitEnergy + pow(distance(CH1, CH2), 2)*AmplifierEnergy)*rate << ", �]���q�p�϶ǵ��j��" << endl;
	cons[node[CH1].region1 - 1] += (TransmitEnergy + pow(distance(CH1, CH2), 2)*AmplifierEnergy)*rate; //��O�ϰ줺�����
	trans_time[node[CH1].region1 - 1] += 1; //��O�ϰ줺���ǰe����
	node[CH2].energy -= (ReceiveEnergy)*rate;
	//fout << "node : " << CH2 << "��q��� " << (ReceiveEnergy)*rate << " ,�]������p�ϸ��" << endl;
	clean(CH1, 0, NODE_BUFFER1);

	CHtoRegion2(CH2, 2);
}

int CheckEnergy()
{
	for (int b = 0; b < S_NUM; b++)
	{
		//fout << "node " << b << "'s energy = " << node[b].energy << endl;
		if (node[b].energy <= 0)
		{
			return b;
			break;
		}
	}
	return SINKID;
}
void Reselection_judge(int s, int e, int rnum)
{
	double avg_energy = find_avg_energy(s, e, rnum);
	if (((avg_energy - node[node[s].CH].energy) / avg_energy) >= 0.15) //�o�ӭȤ��@�w�j��0 , CH����O���@�w��P�� ! �]��������Y�����Y
	{
		CH_Selection(s, e);
	}
}
void CH_Reselection()
{
	Reselection_judge(0, R2 - 1, R_NUM);
	Reselection_judge(R2, R3 - 1, R_NUM);
	Reselection_judge(R3, R4 - 1, R_NUM);
	Reselection_judge(R4, S_NUM - 1, R_NUM);
}
void transaction(int j, int t, int v)
{
	if (v == 1)
	{
		Packet_Generate(j, t);
		Packet_Dliver(j, node[j].CH);
		Packet_Receive(node[j].CH);
	}
	else
	{
		Packet_Generate(j, t);
		Packet_Dliver(j, node[j].CH2);
		Packet_Receive(node[j].CH2);
	}
}

double standard_deviation()
{
	double a(0);
	double b(0);
	double sd(0);
	for (int i = 0; i < S_NUM; i++)
	{
		b += node[i].energy;
	}

	b /= S_NUM;
	return b;
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
	for (int round = 0; round < round_number; round++)
	{
		cout << round << endl;
		int i = 0;
		for (i; i < R2; i++)
		{
			node[i].id = i;
			node[i].x = rand() % 200 + 1;
			node[i].y = rand() % 200 + 1;
			node[i].CH = i;
			node[i].CH2 = -1;
			node[i].type = rand() % 3 + 3;//3 4 5
			node[i].energy = MAX_energy;
			node[i].region1 = 1;
			if (node[i].x <= 100 && node[i].y <= 100)
			{
				node[i].region2 = 1;
			}
			if (node[i].x >= 100 && node[i].y <= 100)
			{
				node[i].region2 = 2;
			}
			if (node[i].x <= 100 && node[i].y >= 100)
			{
				node[i].region2 = 3;
			}
			if (node[i].x >= 100 && node[i].y >= 100)
			{
				node[i].region2 = 4;
			}
			//node[i].dtc = sqrt(pow(abs(node[i].x - 100), 2) + pow(abs(node[i].y - 100), 2)); /*�Z���Ϥ���*/
			//node[i].dtc = abs(node[i].x - 200);/*�Z����2*/
			if (i == R2 - 1)//�Z���ϩҦ��I����
			{
				C center1 = find_center(0, i);
				set_dtc(center1, 0, i);
			}
			//fout << "node" << node[i].id << " x : " << node[i].x << " y : " << node[i].y << " type : " << node[i].type << " energy : " << node[i].energy << " dtc : " << node[i].dtc << endl;
		}
		for (i; i < R3; i++)
		{
			node[i].id = i;
			node[i].x = rand() % 200 + 201;
			node[i].y = rand() % 200 + 1;
			node[i].CH = i;
			node[i].CH2 = -1;
			node[i].type = rand() % 3 + 3;
			node[i].energy = MAX_energy;
			node[i].region1 = 2;
			if (node[i].x <= 300 && node[i].y <= 100)
			{
				node[i].region2 = 1;
			}
			if (node[i].x >= 300 && node[i].y <= 100)
			{
				node[i].region2 = 2;
			}
			if (node[i].x <= 300 && node[i].y >= 100)
			{
				node[i].region2 = 3;
			}
			if (node[i].x >= 300 && node[i].y >= 100)
			{
				node[i].region2 = 4;
			}
			//node[i].dtc = sqrt(pow(abs(node[i].x - 300), 2) + pow(abs(node[i].y - 100), 2)); /*�Z���Ϥ���*/
			//node[i].dtc = sqrt(pow(abs(node[i].x - SINKID), 2) + pow(abs(node[i].y - 0), 2)); /*�Z��sink*/
			if (i == R3 - 1)//�Z���ϩҦ��I����
			{
				C center2 = find_center(R2, i);
				set_dtc(center2, R2, i);
			}
			//fout << "node" << node[i].id << " x : " << node[i].x << " y : " << node[i].y << " type : " << node[i].type << " energy : " << node[i].energy << " dtc : " << node[i].dtc << endl;
		}
		for (i; i < R4; i++)
		{
			node[i].id = i;
			node[i].x = rand() % 200 + 1;
			node[i].y = rand() % 200 + 201;
			node[i].CH = i;
			node[i].CH2 = -1;
			node[i].type = rand() % 3 + 3;
			node[i].energy = MAX_energy;
			node[i].region1 = 3;
			if (node[i].x <= 100 && node[i].y <= 300)
			{
				node[i].region2 = 1;
			}
			if (node[i].x >= 100 && node[i].y <= 300)
			{
				node[i].region2 = 2;
			}
			if (node[i].x <= 100 && node[i].y >= 300)
			{
				node[i].region2 = 3;
			}
			if (node[i].x >= 100 && node[i].y >= 300)
			{
				node[i].region2 = 4;
			}
			//node[i].dtc = sqrt(pow(abs(node[i].x - 100), 2) + pow(abs(node[i].y - 300), 2));/*�Z���Ϥ���*/
			//node[i].dtc = sqrt(pow(abs(node[i].x - 200), 2) + pow(abs(node[i].y - 200), 2));/*�Z����2*/
			if (i == R4 - 1)//�Z���ϩҦ��I����
			{
				C center3 = find_center(R3, i);
				set_dtc(center3, R3, i);
			}
			//fout << "node" << node[i].id << " x : " << node[i].x << " y : " << node[i].y << " type : " << node[i].type << " energy : " << node[i].energy << " dtc : " << node[i].dtc << endl;
		}
		for (i; i < S_NUM; i++)
		{
			node[i].id = i;
			node[i].x = rand() % 200 + 201;
			node[i].y = rand() % 200 + 201;
			node[i].CH = i;
			node[i].CH2 = -1;
			node[i].type = rand() % 3 + 3;
			node[i].energy = MAX_energy;
			node[i].region1 = 4;
			if (node[i].x <= 300 && node[i].y <= 300)
			{
				node[i].region2 = 1;
			}
			if (node[i].x >= 300 && node[i].y <= 300)
			{
				node[i].region2 = 2;
			}
			if (node[i].x <= 300 && node[i].y >= 300)
			{
				node[i].region2 = 3;
			}
			if (node[i].x >= 300 && node[i].y >= 300)
			{
				node[i].region2 = 4;
			}
			//node[i].dtc = sqrt(pow(abs(node[i].x - 300), 2) + pow(abs(node[i].y - 300), 2));/*�Z���Ϥ���*/
			//node[i].dtc = abs(node[i].y - 200);/*�Z����2*/
			if (i == S_NUM - 1)//�Z���ϩҦ��I����
			{
				C center4 = find_center(R4, i);
				set_dtc(center4, R4, i);
			}
			//fout << "node" << node[i].id << " x : " << node[i].x << " y : " << node[i].y << " type : " << node[i].type << " energy : " << node[i].energy << " dtc : " << node[i].dtc << endl;
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

		/*traffic start*/
		int bombing(0);
		int b_region(0);
		int die(0);
		int t(1);
		while (!die)
		{
			/*if(t % 15000 == 0)
			{
			double avg_re = standard_deviation();
			fout << avg_re << endl;
			}*/
			//fout << "time = " << t << endl;
			int c = CheckEnergy();/*���@�Ӹ`�I�S�q�h���󦺤`*/
			if (c < SINKID)
			{
				//fout << t << endl;
				avg_t += t;
				/*double avg_re = standard_deviation();
				fout << t << "  " << avg_re << endl;*/
				//fout << "node " << c << " dead !" << endl;
				die = 1;
				//print_energy();
				/*
				for (int i = 0; i < 4; i++)
				{
				//fout << "�ϰ�" << i + 1 << "���ϰ줺�����ǿ�ӯ� = " << cons[i] / trans_time[i] << endl;
				}
				for (int i = 0; i < 4; i++)
				{
				//fout << "�ϰ�" << i + 1 << "�������ǿ��2�ӯ� = " << consToR2[i] / ToR2_time[i] << endl;
				}
				*/
				/*int e(0);
				while (sink.buffer[e].data != -1)
				{
				p_in_sink++;
				e++;
				}*/
				//fout << "total = " << total << endl;
				//fout << "drop = " << drop << endl;
				//fout << "macdrop = " << macdrop << endl;
				/*�B�I�ƹB��p�G�B�⤸�Oint �h���i��|���ͥX�u����ƪ����G �ҥH�n�o�˼g*/
				//double PLR = 0;
				//PLR = (macdrop + drop);
				//PLR /= total;
				/**/
				//fout << "packet loss rate = " << PLR << endl;
				//fout << "sink ��" << e << endl; //total�ʥ]��*/
				break;
			}


			/*Data genetate , trans , receive*/
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
						if (node[j].type == 3 && node[j].region1 == b_region)
						{
							if (node[j].CH2 == -1 || node[j].CH == node[j].id)
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
				if (t % bomb_f4 == 0)
				{
					for (int j = 0; j < S_NUM; j++)
					{
						if (node[j].type == 4 && node[j].region1 == b_region)
						{
							if (node[j].CH2 == -1 || node[j].CH == node[j].id)
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
				if (t % bomb_f5 == 0)
				{
					for (int j = 0; j < S_NUM; j++)
					{
						if (node[j].type == 5 && node[j].region1 == b_region)
						{
							if (node[j].CH2 == -1 || node[j].CH == node[j].id)
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

				/*�D�z����*/
				if (t % type3f == 0)
				{
					for (int j = 0; j < S_NUM; j++)
					{
						if (node[j].type == 3 && node[j].region1 != b_region)
						{
							if (node[j].CH2 == -1 || node[j].CH == node[j].id)
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
						if (node[j].type == 4 && node[j].region1 != b_region)
						{
							if (node[j].CH2 == -1 || node[j].CH == node[j].id)
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
						if (node[j].type == 5 && node[j].region1 != b_region)
						{
							if (node[j].CH2 == -1 || node[j].CH == node[j].id)
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
			}
			else//regular trans
			{
				if (t % type3f == 0)
				{
					for (int j = 0; j < S_NUM; j++)
					{
						if (node[j].type == 3) //CH need to sense
						{
							if (node[j].CH2 == -1 || node[j].CH == node[j].id)
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
						if (node[j].type == 4) //CH need to sense
						{
							if (node[j].CH2 == -1 || node[j].CH == node[j].id)
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
						if (node[j].type == 5) //CH need to sense
						{
							if (node[j].CH2 == -1 || node[j].CH == node[j].id)
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
			}



			if (t % CHf == 0) //�C�@�����Ǩ�sink 1��
			{
				int CH[4];
				CH[0] = node[0].CH;
				CH[1] = node[R2].CH;
				CH[2] = node[R3].CH;
				CH[3] = node[R4].CH;

				if (bomb_switch)
				{
					for (int i = 0; i < 4; i++)
					{
						if (current_state[i] == 1)
						{
							int Pcount(0);//�P�_�U�Ӥp�Y���ֿn�q�O�_�i�H�X�֤F
							for (int j = 0; j < 4; j++)
							{
								if (g2[i][j] != -1)
								{
									//cout << Pcount << endl;
									Pcount += Packet_num(g2[i][j]); //�u����e��BUFFER1(�z�L���`CH(�Ҧ�1)��o���ʥ])
									if (i != 1) //��2
									{
										g2toCH(g2[i][j], CH[i]);
									}
									else //�D��2
									{
										CH2Sink(g2[i][j]); //�ϰ�2���p�Y�����ǵ�SINK
									}
									sec_ch_re_check(i + 1, j + 1);
								}
							}
							if (Pcount < NODE_BUFFER1) //��ܳo�@�ϳo�@�����ʥ]�ƶq���O�z��
							{
								//cout <<i+1 <<"�Ϫ��p�Y�`�@��"<< Pcount << endl;
								if (t - old_t[i] == CHf) //���լO�_�s��:�p�G�{�b��t��L�h��t = CHf���ܥN��s��X�{�z��
								{
									normal_times[i]++;
								}
								else
								{
									normal_times[i] = 1; //���s�� �����Ĥ@��
								}
								old_t[i] = t;
								if (normal_times[i] == full_th) //�s3���`
								{
									//cout <<i<< "�ϦX �ɶ��O" <<t<< endl;
									normal_times[i] = 0;
									bomb_cancel(i + 1);//�Ƕi�h���O�ϰ�s���A���O�}�C�Ʀr
									current_state[i] = 0;
								}
							}
						}
						else
						{
							if (Packet_num(CH[i]) == NODE_BUFFER1)
							{
								//cout << i + 1 << "���z���F" << endl;
								if (t - old_t[i] == CHf) //���լO�_�s��:�p�G�{�b��t��L�h��t = CHf���ܥN��s��X�{�z��
								{
									bomb_times[i]++;
								}
								else
								{
									bomb_times[i] = 1; //���s�� �����Ĥ@��
								}
								old_t[i] = t;
								if (bomb_times[i] == full_th) //�s3�z
								{
									//cout <<i<< "���z �ɶ��O" << t << endl;
									bomb_times[i] = 0;
									sec_grid(i + 1);//�Ƕi�h���O�ϰ�s���A���O�}�C�Ʀr
									current_state[i] = 1;
								}
							}
						}
					}
				}

				CH2Sink(CH[1]);
				CHtoRegion2(CH[0], 1);
				CHtoRegion2(CH[2], 1);
				CHtoRegion2(CH[3], 1);
				CH_Reselection();
			}
			t++;
		}
	}
	p_in_sink /= round_number;
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

