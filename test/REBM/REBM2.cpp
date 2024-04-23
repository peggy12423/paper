#include <iostream>
#include <time.h>
#include <math.h>
#include <fstream>
#include <queue>     

#define roundnumber 1
#define MAX_energy 6480//j (1.5V*600mA*3600sec*2 = 6480j) 2*3號電池
#define SINKBUFFER 10000000
#define SINKID 2000
#define SINK_X 400
#define SINK_Y 0
/*變動實驗參數設定*/
#define E_NUM 1000 //感測器總數
#define R 0.5 //壓縮率 設1則沒有壓縮
#define type3f 90 //常規sensing frequency
#define type4f 120
#define type5f 150
#define CHf 100 //CH trans frequency
#define freq_change_switch 0 //0關 1開 是否要使資料量突然暴增的開關
#define b_t 10800 //大T 每多少秒開一次 小T 每一次開多少秒
#define s_t 1800
#define bomb_f3 45 //爆炸sensing frequency
#define bomb_f4 60
#define bomb_f5 90
/**/
#define ProbeEnergy 0.03 //j (8*200bit*1.5V*25mA*0.5mS = 0.00375j*8 = 0.03j) !/*(查到的論文:bit*50nj+bit*distance(m)平方*100nj)*/
#define TransmitEnergy 0.00008 //0.000000001*50*200*8 j (50nj/bit by冠中) bit * 8 = byte !(是這樣算嗎)
#define AmplifierEnergy 0.00016 //100*200*0.000000001*8 j/distance^2
#define ReceiveEnergy 0.00008 //0.000000001*50*200j*8 (50nj/bit by冠中)
#define Package_size 200 //bytes 
#define node_buffer 20 //Kbytes (100格)
#define trans_dis 60 //m 80幾乎可以確定他傳的到sink

using namespace std;
int S_NUM = 400;
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
	P buffer[100];//buffer in sensor node
	double dtc;//dist = distance to sink
	int non; //nomber of negihbor
	int neighbor[1000]; //存取節點的鄰居
	queue<int>route;
};
struct S
{
	int id;//node information
	P buffer[SINKBUFFER];//buffer
};

ofstream fout("output.txt");
N ns[2000];
S sink;
double avg_t(0);
double drop(0);
double macdrop(0);
double total(0);
int R2 , R3, R4;
void print_energy()
{
	for (int i = 0; i < S_NUM; i++)
	{
		fout << "node" << ns[i].id << " x : " << ns[i].x << " y : " << ns[i].y << " type : " << ns[i].type << " energy : " << ns[i].energy << endl;
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
void neighbor_init()
{
	for (int i = 0; i < S_NUM; i++)
	{
		for (int j = 0; j < S_NUM; j++)
		{
			if (distance(i, SINKID) > distance(j, SINKID) && i != j && distance(i, j) <= trans_dis)
			{
				ns[i].neighbor[ns[i].non] = j;
				ns[i].non++;
			}
		}
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
		for (int b = 0; b < 100; b++)
		{
			ns[a].buffer[b].data = -1;
			ns[a].buffer[b].dst = -1;
			ns[a].buffer[b].src = -1;
			ns[a].buffer[b].time = -1;
		}
	}
}
void Packet_Generate(int now, int t) //generate packet 有能耗
{
	total++;
	ns[now].sense.src = ns[now].id;
	ns[now].sense.dst = SINKID;
	ns[now].sense.data = ns[now].type;
	ns[now].sense.time = t;
	ns[now].energy -= ProbeEnergy;
	//dout << "src: " << ns[now].sense.src << " dst: " << ns[now].sense.dst << " data: " << ns[now].sense.data << " time: " << ns[now].sense.time << " sec" << endl;
	//fout << "node : " << now << "能量減少 " << ProbeEnergy << " ,因為產生感測封包 " << endl;
}
int Packet_Dliver(int sender, int receiver) // 有能耗
{
	double d = distance(sender, receiver);
	ns[sender].energy -= TransmitEnergy + d*d*AmplifierEnergy;
	int rate = rand() % 100 + 1;
	if (rate > 5)  /*10% drop rate or CH自己將sense的封包放自己的buffer*/
	{
		ns[receiver].sense.dst = ns[sender].sense.dst;
		ns[receiver].sense.src = ns[sender].sense.src;
		ns[receiver].sense.data = ns[sender].sense.data;
		ns[receiver].sense.time = ns[sender].sense.time;
		ns[receiver].energy -= ReceiveEnergy;
		return 1;
	}
	else
	{
		macdrop++;
		ns[receiver].sense.dst = -1;
		ns[receiver].sense.src = -1;
		ns[receiver].sense.data = -1;
		ns[receiver].sense.time = -1;
		return -1;
	}
	//fout << "node : " << sender << "能量減少 " << TransmitEnergy + d*d*AmplifierEnergy << " ,因為傳送給節點 " << receiver << " 感測封包" << endl;
	//fout << "node : " << receiver << "能量減少 " << ReceiveEnergy << " ,因為收到來自節點 " << sender << " 的封包" << endl;
	//1個封包
}
void DeliveToSink(int sender) //有能耗
{
	for (int i = 0; i < 100000; i++)
	{
		if (sink.buffer[i].data == -1)
		{
			sink.buffer[i].data = ns[sender].sense.data;
			sink.buffer[i].dst = ns[sender].sense.dst;
			sink.buffer[i].src = ns[sender].sense.src;
			sink.buffer[i].time = ns[sender].sense.time;
			break;
		}
	}
	double d = distance(sender, SINKID);
	ns[sender].energy -= TransmitEnergy + d*d*AmplifierEnergy;
	//fout << "node : " << sender << "能量減少 " << TransmitEnergy + d*d*AmplifierEnergy << " ,因為傳送給sink封包" << endl;
}

int max(int n[], int non,int node)
{
	int next = -1;
	if (non == 0)
	{
		next = -1;
	}
	else
	{
		next = n[0];
		for (int i = 0; i < non; i++)
		{
			if (ns[next].energy < ns[n[i]].energy)
			{
				next = n[i];
			}
			else if (ns[next].energy == ns[n[i]].energy)
			{
				if (distance(node, next) > distance(node, i))
				{
					next = n[i];
				}
				else if (distance(node, next) == distance(node, i))
				{
					if (ns[next].non > ns[n[i]].non)
					{
						next = n[i];
					}
				}
			}
		}
	}
	return next;
}
int next_hop(int node_id)
{
	int next;
	if (distance(node_id, SINKID) <= trans_dis)
	{
		next = SINKID;
	}
	else
	{
		next = max(ns[node_id].neighbor, ns[node_id].non,node_id);
	}
	return next;
}
void EXLIOSE(int node_id, int t)
{
	Packet_Generate(node_id, t);//產生封包
	if (distance(node_id, SINKID) <= trans_dis) //如果就在sink直接傳就好
	{
		DeliveToSink(node_id);
	}
	else //如果不在需要多跳
	{
		int del, recv;
		queue<int> path;
		path.push(node_id);
		while (path.back() != SINKID && path.back() != -1)
		{
			path.push(next_hop(path.back()));
		}
		if (path.back() == -1) //沒有路徑就不用傳了
		{
			drop++;
		}
		else
		{
			while (path.front() != SINKID)
			{
				del = path.front();
				path.pop();
				recv = path.front();
				if (recv != SINKID)
				{
					int s = Packet_Dliver(del, recv);
					while (s != 1)
					{
						total++;
						s = Packet_Dliver(del, recv);
					}
				}
				else
				{
					DeliveToSink(del);
				}
			}
		}
	}
}
int CheckEnergy()
{
	for (int b = 0; b < S_NUM; b++)
	{
		if (ns[b].energy <= 0)
		{
			return b;
			break;
		}
	}
	return SINKID;
}

int main()
{
	/*sensor initialization*/
	srand((unsigned)time(NULL)); //random seed
	for( S_NUM ; S_NUM <= E_NUM ; S_NUM += 100){
		avg_t = 0;
		drop = 0;
		macdrop = 0;
		total = 0;
		int CH_count = 0;
		cout << "sensors: " << S_NUM << endl;
		fout << endl << "------------ Sensors " << S_NUM << " ------------" << endl;
		for (int rn = 0; rn < roundnumber; rn++)
		{
			cout << rn+1 << endl;
			int i = 0;
			for (i; i < R2; i++)
			{
				ns[i].id = i;
				ns[i].x = rand() % 200 + 1;
				ns[i].y = rand() % 200 + 1;
				ns[i].type = rand() % 3 + 3;//3 4 5
				ns[i].energy = MAX_energy;
				ns[i].non = 0;
				ns[i].region1 = 1;
			}
			for (i; i < R3; i++)
			{
				ns[i].id = i;
				ns[i].x = rand() % 200 + 201;
				ns[i].y = rand() % 200 + 1;
				ns[i].type = rand() % 3 + 3;
				ns[i].energy = MAX_energy;
				ns[i].non = 0;
				ns[i].region1 = 2;
			}
			for (i; i < R4; i++)
			{
				ns[i].id = i;
				ns[i].x = rand() % 200 + 1;
				ns[i].y = rand() % 200 + 201;
				ns[i].type = rand() % 3 + 3;
				ns[i].energy = MAX_energy;
				ns[i].non = 0;
				ns[i].region1 = 3;
			}
			for (i; i < S_NUM; i++)
			{
				ns[i].id = i;
				ns[i].x = rand() % 200 + 201;
				ns[i].y = rand() % 200 + 201;
				ns[i].type = rand() % 3 + 3;
				ns[i].energy = MAX_energy;
				ns[i].non = 0;
				ns[i].region1 = 4;
			}
			packet_init();
			/*sink initialization*/
			sink.id = SINKID;
			for (int b = 0; b < SINKBUFFER; b++)
			{
				sink.buffer[b].data = -1;
				sink.buffer[b].dst = -1;
				sink.buffer[b].src = -1;
				sink.buffer[b].time = -1;
			}
			/*neighbor initialization*/
			neighbor_init();
			/*traffic start*/
			int bombing(0);
			int b_region(0);
			int die(0);//die=1就不跑迴圈
			int t(1);
			while (!die)
			{
				//fout << "time = " << t << endl;
				int c = CheckEnergy();/*有一個節點沒電則等於死亡*/
				if (c < SINKID)
				{
					die = 1;
					avg_t += t;
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
					else //結束爆炸 ,調回參數
					{
						b_region = 0;
						bombing = 0;
					}
				}
				if (bombing)
				{
					/*爆炸區*/
					if (t % bomb_f3 == 0)
					{
						for (int j = 0; j < S_NUM; j++)
						{
							if (ns[j].type == 3 && ns[j].region1 == b_region)
							{
								EXLIOSE(j, t);
							}
						}
					}
					if (t % bomb_f4 == 0)
					{
						for (int j = 0; j < S_NUM; j++)
						{
							if (ns[j].type == 4 && ns[j].region1 == b_region)
							{
								EXLIOSE(j, t);
							}
						}
					}
					if (t % bomb_f5 == 0)
					{
						for (int j = 0; j < S_NUM; j++)
						{
							if (ns[j].type == 5 && ns[j].region1 == b_region)
							{
								EXLIOSE(j, t);
							}
						}
					}

					/*非爆炸區*/
					if (t % type3f == 0)
					{
						for (int j = 0; j < S_NUM; j++)
						{
							if (ns[j].type == 3 && ns[j].region1 != b_region)
							{
								EXLIOSE(j, t);
							}
						}
					}
					if (t % type4f == 0)
					{
						for (int j = 0; j < S_NUM; j++)
						{
							if (ns[j].type == 4 && ns[j].region1 != b_region)
							{
								EXLIOSE(j, t);
							}
						}
					}
					if (t % type5f == 0)
					{
						for (int j = 0; j < S_NUM; j++)
						{
							if (ns[j].type == 5 && ns[j].region1 != b_region)
							{
								EXLIOSE(j, t);
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
								EXLIOSE(j, t);
							}
						}
					}
					if (t % type4f == 0)
					{
						for (int j = 0; j < S_NUM; j++)
						{
							if (ns[j].type == 4) //CH need to sense
							{
								EXLIOSE(j, t);
							}
						}
					}
					if (t % type5f == 0)
					{
						for (int j = 0; j < S_NUM; j++)
						{
							if (ns[j].type == 5) //CH need to sense
							{
								EXLIOSE(j, t);
							}
						}
					}
				}

				t++;
			}
		}
		total /= roundnumber;
		avg_t /= roundnumber;
		drop /= roundnumber;
		macdrop /= roundnumber;
		fout << "avg_t : " << avg_t << endl;
		fout << "avg_total : " << total << endl;
		fout << "avg_drop : " << drop << endl;
		fout << "avg_macdrop : " << macdrop << endl;
		fout << "avg_PLR : " << (drop + macdrop) / total << endl;
	}
	return 0;
}

