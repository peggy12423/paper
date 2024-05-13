#include <iostream>
#include <time.h>
#include <math.h>
#include <fstream>
#include <cmath>
#include <vector>
#include <algorithm>

#define MAX_energy 6480//j (1.5V*600mA*3600sec*2 = 6480j) 2*3���q��
#define SINKID 2000
#define SINK_X 400
#define SINK_Y 0
#define SINK_BUFFER_SIZE 5000000
#define NODE_BUFFER1 300 //0~49 �@��CH����CM�� node_buffer 40Kbytes (200��) ��F�o�ӰѼ� �U����bomb�]�n��
#define NODE_BUFFER2 600 //50~100 �S�O���ǿ��

#define type3f 90//�`�Wsensing frequency
#define type4f 120
#define type5f 150
#define reservation_energy_time 10000
#define CHf 100 //CH trans frequency

#define ProbeEnergy 0.03 //j (8*200bit*1.5V*25mA*0.5mS = 0.00375j*8 = 0.03j) !
/*(�d�쪺�פ�:bit*50nj+bit*distance(m)����*100nj)*/
#define TransmitEnergy 0.00008 //0.000000001*50*200*8 j (50nj/bit by�a��) bit * 8 = byte !
#define AmplifierEnergy 0.00016 //100*200*0.000000001*8 j/distance^2
#define ReceiveEnergy 0.00008 //0.000000001*50*200j*8 (50nj/bit by�a��)
#define Package_size 200 //bytes 
#define successful_rate 5 //�]x ���\�v�N�O100-x%

/*�ܰʹ���ѼƳ]�w*/
<<<<<<< HEAD
#define round_number 1
#define E_NUM 400
=======
#define round_number 20
#define E_NUM 1000
>>>>>>> branch1
#define round_interval 100
#define Per 0.8  //CH�w���ƶq
#define Pro 0.5  //�`�I����CH�����v
#define R 0.5 //���Y�v �]1�h�S�����Y

using namespace std;

int S_NUM = 400; //�P�����`��
int Ere_switch = 0; //1�N��n��XEre
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
	double random_num;
    int Eth;
	double energy;//node information
	P receive;
    // P receive2;
	P sense;
    // P sense2;
	P buffer[NODE_BUFFER2];//buffer in sensor node
	double dtc;//dist = distance to sink
};
struct S
{
	int id;//node information
	P buffer[SINK_BUFFER_SIZE];//buffer
};
<<<<<<< HEAD
ofstream fout("OLeach_Ere.txt");
=======
ofstream fout("OLeach_spe2.txt");
>>>>>>> branch1
N ns[2000];
S sink;
double avg_t, buffer_drop, mac_drop, total;
int R2, R3, R4;
int CH[2000];
int G[2000];
vector<int> CHarr = {};

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
        ns[i].type = rand() % 3 + 3;//3 4 5
        ns[i].energy = MAX_energy;
        ns[i].region1 = 1;
        ns[i].random_num = (rand() % 101 )* 0.01;
    }
    for (i; i < R3; i++)
    {
        ns[i].id = i;
        ns[i].x = rand() % 200 + 201;
        ns[i].y = rand() % 200 + 1;
        ns[i].type = rand() % 3 + 3;
        ns[i].energy = MAX_energy;
        ns[i].region1 = 2;
        ns[i].random_num = (rand() % 101) * 0.01;
    }
    for (i; i < R4; i++)
    {
        ns[i].id = i;
        ns[i].x = rand() % 200 + 1;
        ns[i].y = rand() % 200 + 201;
        ns[i].type = rand() % 3 + 3;
        ns[i].energy = MAX_energy;
        ns[i].region1 = 3;
        ns[i].random_num = (rand() % 101) * 0.01;
    }
    for (i; i < S_NUM; i++)
    {
        ns[i].id = i;
        ns[i].x = rand() % 200 + 201;
        ns[i].y = rand() % 200 + 201;
        ns[i].type = rand() % 3 + 3;
        ns[i].energy = MAX_energy;
        ns[i].region1 = 4;
        ns[i].random_num = (rand() % 101) * 0.01;
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
        ns[i].random_num = (rand() % 101 )* 0.01;
    }
    for (i; i < R3; i++)
    {
        ns[i].id = i;
        ns[i].x = rand() % 200 + 201;
        ns[i].y = rand() % 200 + 1;
        ns[i].type = rand() % 3 + 3;
        ns[i].energy = MAX_energy;
        ns[i].region1 = 2;
        ns[i].random_num = (rand() % 101) * 0.01;
    }
    for (i; i < R4; i++)
    {
        ns[i].id = i;
        ns[i].x = rand() % 200 + 1;
        ns[i].y = rand() % 200 + 201;
        ns[i].type = rand() % 3 + 3;
        ns[i].energy = MAX_energy;
        ns[i].region1 = 3;
        ns[i].random_num = (rand() % 101) * 0.01;
    }
    for (i; i < S_NUM; i++)
    {
        ns[i].id = i;
        ns[i].x = rand() % 200 + 201;
        ns[i].y = rand() % 200 + 201;
        ns[i].type = rand() % 3 + 3;
        ns[i].energy = MAX_energy;
        ns[i].region1 = 4;
        ns[i].random_num = (rand() % 101) * 0.01;
    }
}

void special2_node_deployed(){
	R2 = S_NUM * 0.4;
	R3 = S_NUM * 0.5;
	R4 = S_NUM * 0.9;
	int i = 0;
    for (i; i < R2; i++)
    {
        ns[i].id = i;
        ns[i].x = rand() % 200 + 1;
        ns[i].y = rand() % 200 + 1;
        ns[i].type = rand() % 3 + 3;//3 4 5
        ns[i].energy = MAX_energy;
        ns[i].region1 = 1;
        ns[i].random_num = (rand() % 101 )* 0.01;
    }
    for (i; i < R3; i++)
    {
        ns[i].id = i;
        ns[i].x = rand() % 200 + 201;
        ns[i].y = rand() % 200 + 1;
        ns[i].type = rand() % 3 + 3;
        ns[i].energy = MAX_energy;
        ns[i].region1 = 2;
        ns[i].random_num = (rand() % 101) * 0.01;
    }
    for (i; i < R4; i++)
    {
        ns[i].id = i;
        ns[i].x = rand() % 200 + 1;
        ns[i].y = rand() % 200 + 201;
        ns[i].type = rand() % 3 + 3;
        ns[i].energy = MAX_energy;
        ns[i].region1 = 3;
        ns[i].random_num = (rand() % 101) * 0.01;
    }
    for (i; i < S_NUM; i++)
    {
        ns[i].id = i;
        ns[i].x = rand() % 200 + 201;
        ns[i].y = rand() % 200 + 201;
        ns[i].type = rand() % 3 + 3;
        ns[i].energy = MAX_energy;
        ns[i].region1 = 4;
        ns[i].random_num = (rand() % 101) * 0.01;
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
   		// ns[a].receive2.data = -1;
		// ns[a].receive2.dst = -1;
		// ns[a].receive2.src = -1;
		// ns[a].receive2.time = -1;

		ns[a].sense.data = -1;
		ns[a].sense.dst = -1;
		ns[a].sense.src = -1;
		ns[a].sense.time = -1;
        // ns[a].sense2.data = -1;
		// ns[a].sense2.dst = -1;
		// ns[a].sense2.src = -1;
		// ns[a].sense2.time = -1;
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

int inG(int id)
{
	for (int i = 0; i < S_NUM; i++)
	{
		if (id == G[i])
		{
			return 1;
			break;
		}
	}
	return 0;
}

void add_to_CHarr(vector<int>& CHarr, int num) {
    // �ˬd�Ʀr�O�_�w�s�b��}�C��
    if (find(CHarr.begin(), CHarr.end(), num) == CHarr.end()) {
        // �p�G�Ʀr���s�b�A�[�J��}�C��
        CHarr.push_back(num);
    }
}

void CH_set(int round){
	int x = 1 / Per;
	double Tth = Per / ( 1 - Per*( round % x));
    double Eth[2000];
    for (int i = 0; i < S_NUM; i++){
        Eth[i] = 2 * Pro * ns[i].energy / MAX_energy;
    }
	for (int i = 0; i < S_NUM; i++)
	{
		if (inG(ns[i].id) == 0){  //���bG�}�C��(�DCH)
            if( ns[i].energy >= 0.5 * MAX_energy){
                if (ns[i].random_num < Tth)
                {
                    ns[i].CH = ns[i].id;//����CH
					add_to_CHarr(CHarr, ns[i].id);
                    for (int j = 0; j < S_NUM; j++) //��iG�̭� �T�O���ᤣ�|���L��CH
                    {
                        if (G[j] != -1)
                        {
                            G[j] = ns[i].id;
                            break;
                        }
                    }
                }
                else
                {
                    ns[i].CH = -1;
                }
            }
            else{
                if (ns[i].random_num < Eth[i])
                {
                    ns[i].CH = ns[i].id;//����CH
					add_to_CHarr(CHarr, ns[i].id);
                    for (int j = 0; j < S_NUM; j++) //��iG�̭� �T�O���ᤣ�|���L��CH
                    {
                        if (G[j] != -1)
                        {
                            G[j] = ns[i].id;
                            break;
                        }
                    }
                }
                else
                {
                    ns[i].CH = -1;
                }
            }
		}
		else
		{
			ns[i].CH = -1;
		}
	}
	for (int i = 0; i < S_NUM; i++)
	{
		if (ns[i].CH == -1)
		{
			double min(10000);
			for (int j = 0; j < S_NUM; j++)
			{
				if (ns[j].CH != -1)
				{
					if (min > distance(ns[i].id, ns[j].id))
					{
						ns[i].CH = ns[j].id;
					}
				}
			}
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
	}
	else
	{
		mac_drop++;
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
			ns[CH].receive.dst = -1;
			ns[CH].receive.src = -1;
			ns[CH].receive.data = -1;
			ns[CH].receive.time = -1;
			break;
		}
	}
	if (full == 1 && ns[CH].receive.data != -1) //priority queue buffer , �p�G�ʥ]�Qdrop���N���ΤF(-1)
	{
		ns[CH].receive.dst = -1;
		ns[CH].receive.src = -1;
		ns[CH].receive.data = -1;
		ns[CH].receive.time = -1;
		buffer_drop++;
	}
}

void transaction(int j, int t)
{
	Packet_Generate(j, t);
	Packet_Dliver(j, ns[j].CH);
	Packet_Receive(ns[j].CH);
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

double remaining_energy()
{
	double avg_energy;
	for (int i = 0; i < S_NUM; i++)
	{
		avg_energy += ns[i].energy;
	}
	avg_energy /= S_NUM;
	return avg_energy;
}

int main()
{
	/*sensor initialization*/
	srand((unsigned)time(NULL)); //random seed
	fout << "Optimal of LEACH" << endl;
	for( S_NUM ; S_NUM <= E_NUM ; S_NUM += 100){
		avg_t = 0;
		buffer_drop = 0;
		mac_drop = 0;
		total = 0;
		int CH_count = 0;
		cout << "sensors: " << S_NUM << endl;
		fout << endl << "------------ Sensors " << S_NUM << " ------------" << endl;
		/*sensor initialization*/
		for (int r = 0; r < round_number; r++)
        {
            cout << r+1 << endl;
            // node_deployed();
            // special_node_deployed();
			special2_node_deployed();
            packet_init();

            /*sink initialization*/
            sink_init();

            for (int g = 0; g < S_NUM; g++)
            {
                G[g] = -1;
            }
            /*firts CH selection*/
            int round(1);
            CH_set(round);

            /*traffic start*/
            int die(0);
            int t(1);
            while (!die)
            {
                int c = CheckEnergy();/*���@�Ӹ`�I�S�q�h���󦺤`*/
                if (c < SINKID)
                {
                    die = 1;
                    int e(0);
                    avg_t += t;
                    break;
                }
                
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
                

                if (t % CHf == 0)
                {
                    for (int q = 0; q < S_NUM; q++)
                    {
                        if (ns[q].CH == ns[q].id)
                        {
                            CH2Sink(ns[q].id);
                        }
                    }
                }
                if (t % round_interval == 0)
                {
                    round++;
                    CH_set(round);
                }
<<<<<<< HEAD
				if( t % 2000 == 0){
=======
				if( (Ere_switch == 1) && (t % 2000 == 0) ){
>>>>>>> branch1
					double re_energy = remaining_energy();
					fout << "------time " << t << "------  " << "Remaining energy: " << re_energy << endl;
				}
                t++;
            }
			CH_count += CHarr.size();
			CHarr.clear();
        }
		total /= round_number;
		mac_drop /= round_number;
		buffer_drop /= round_number;
		avg_t /= round_number;
		CH_count /= round_number;
		fout << "CH_count : " << CH_count << endl;
		fout << "avg_lifetime : " << avg_t << endl;
		fout << "avg_total : " << total << endl;
		fout << "avg_macdrop : " << mac_drop << endl;
		fout << "avg_drop : " << buffer_drop << endl;
		fout << "avg_PLR : " << (buffer_drop + mac_drop) / total << endl;
	}
	return 0;
}
