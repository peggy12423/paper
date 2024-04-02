#include <iostream>
#include <time.h>
#include <math.h>
#include <fstream>

#define MAX_energy 6480//j (1.5V*600mA*3600sec*2 = 6480j) 2*3號電池
#define SINKID 2000
#define SINK_X 400
#define SINK_Y 0
#define SINK_BUFFER_SIZE 1000000
#define NODE_BUFFER1 200 //0~49 一般CH接收CM用
#define NODE_BUFFER2 400 //50~100 特別的傳輸用
/*變動實驗參數設定*/
#define S_NUM 900 //感測器總數
#define R 0.25 //壓縮率 設1則沒有壓縮
#define type3f 360 //常規sensing frequency
#define type4f 480
#define type5f 600
#define CHf 120 //CH trans frequency
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

#define round_number 30
#define round_interval 300
using namespace std;
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
ofstream fout("output.txt");
N ns[S_NUM];
S sink;
double avg_t(0);
double drop(0);
double macdrop(0);
double total(0);
int G[S_NUM];
double SD(0);
int R2 = S_NUM / 4;
int R3 = S_NUM / 2;
int R4 = S_NUM * 0.75;

double type_a = 33, type_b = 33, type_c = 34; //調整QUERE裡面感測資料的比例
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

double standard(double a, double b, double ENERGY_STANDARD, double DIST_STANDARD) //used to select CH !這個選擇方式目前是造成整個實驗能量消耗不平衡的原因(是否要考慮消耗能量的大小) !放入節點種類參考?
{
	double s = 0.3*(1 - a / DIST_STANDARD) + 0.7*(b / ENERGY_STANDARD); //larger is better
	return s;
}
double find_max_energy(int s, int e) //!energy的預扣
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
void CH_set(double P, int r)
{
	double t;
	int x = 1 / P;
	t = P / (1 - P*(r % x));
	for (int i = 0; i < S_NUM; i++)
	{
		if (inG(ns[i].id) == 0)
		{
			ns[i].random_num = rand() % 11;
			ns[i].random_num /= 10;
			if (ns[i].random_num < t)
			{
				ns[i].CH = ns[i].id;//此為CH
				for (int j = 0; j < S_NUM; j++) //放進G裡面 確保之後不會選到他當CH
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


void Packet_Generate(int now, int t) //generate packet 有能耗
{
	total++;
	ns[now].sense.src = ns[now].id;
	ns[now].sense.dst = ns[now].CH;
	ns[now].sense.data = ns[now].type;
	ns[now].sense.time = t;
	ns[now].energy -= ProbeEnergy;
	//fout << "node : " << now << "能量減少 "<< ProbeEnergy <<" ,因為產生感測封包 "<< endl;
}
void Packet_Dliver(int sender, int CH) // 有能耗
{
	int rate = rand() % 100 + 1;
	if (rate > 10 || sender == CH)  /*10% drop rate or CH自己將sense的封包放自己的buffer*/
	{
		ns[CH].receive.dst = ns[sender].sense.dst;
		ns[CH].receive.src = ns[sender].sense.src;
		ns[CH].receive.data = ns[sender].sense.data;
		ns[CH].receive.time = ns[sender].sense.time;
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
	if (sender != CH) //CH自己給自己不用扣能量
	{
		ns[sender].energy -= TransmitEnergy + d*d*AmplifierEnergy;
		//fout << "node : " << sender << "能量減少 "<< TransmitEnergy + d*d*AmplifierEnergy <<" ,因為傳送給節點 " << CH << " 感測封包" << endl;
	} //1個封包
}

void Packet_Receive(int CH) //buffer滿了要變成priority queue 有能耗
{
	if (ns[CH].receive.src != CH) //不是來自自己的才要扣能量
	{
		ns[CH].energy -= ReceiveEnergy;
		//fout << "node : " << CH << "能量減少 "<< ReceiveEnergy << " ,因為收到來自節點 " << ns[CH].receive.src << " 的封包" << endl;
	} //drop還算是有收
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
	if (full == 1 && ns[CH].receive.data != -1) //priority queue buffer , 如果封包被drop掉就不用了(-1)
	{
		ns[CH].receive.dst = -1;
		ns[CH].receive.src = -1;
		ns[CH].receive.data = -1;
		ns[CH].receive.time = -1;
		drop++;
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

void CH2Sink(int CH) //有能耗
{
	int start(0);/*sink的buffer從哪邊開始是空的*/
	for (int b = 0; b < SINK_BUFFER_SIZE; b++)
	{
		if (sink.buffer[b].data == -1)
		{
			start = b;
			break;
		}
	}
	if (ns[CH].buffer[NODE_BUFFER1].data != -1)/*幫別的CH傳*/
	{
		double rate(0);/*壓縮率0.25*/
		for (int b = NODE_BUFFER1; b < NODE_BUFFER2; b++)
		{
			if (ns[CH].buffer[b].data == -1)  //有空的就不用繼續了
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
		//fout << "sink收到" << rate << "個" << endl;
		rate = ceil(rate * R);
		//fout << rate << endl;
		ns[CH].energy -= (TransmitEnergy + pow(distance(CH, SINKID), 2)*AmplifierEnergy)*rate; //將data合併之後一次傳送 所以耗能這樣算(合併未做) 因為可知合併的size必在packet的大小之中
																							   //fout << "幫別人成功,我的能量只剩" << ns[CH].energy << endl;
																							   //fout << "node : " << CH << "能量減少 "<< (TransmitEnergy + pow(distance(CH, SINKID), 2)*AmplifierEnergy)*rate <<" ,因為幫別人傳給sink" << endl;
		clean(CH, NODE_BUFFER1, NODE_BUFFER2); /*傳完之後刪除掉*/
	}
	else      /*自己傳*/
	{
		double rate(0);/*壓縮率0.25*/
		for (int b = 0; b < NODE_BUFFER1; b++)
		{
			if (ns[CH].buffer[b].data == -1) //有空的就不用繼續了
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
		//fout << "sink收到" << rate << "個" << endl;
		rate = ceil(rate * R);
		//fout << rate << endl;
		ns[CH].energy -= (TransmitEnergy + pow(distance(CH, SINKID), 2)*AmplifierEnergy)*rate; //將data合併之後一次傳送 所以耗能這樣算(合併未做)
																							   //fout << "自己傳,我的能量只剩" << ns[CH].energy << endl;
																							   //fout << "node : " << CH << "能量減少 "<< (TransmitEnergy + pow(distance(CH, SINKID), 2)*AmplifierEnergy)*rate <<" ,因為幫自己傳給sink" << endl;
		clean(CH, 0, NODE_BUFFER1); /*傳完之後刪除掉*/
	}
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
	for (int r = 0; r < round_number; r++)
	{
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
		for (int g = 0; g < S_NUM; g++)
		{
			G[g] = -1;
		}
		/*firts CH selection*/
		int round(1);
		double p = 0.2;
		CH_set(p, round);

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
			//fout << "time = " << t << endl;
			int c = CheckEnergy();/*有一個節點沒電則等於死亡*/
			if (c < SINKID)
			{
				/*double avg_re = standard_deviation();
				fout << t << "  " << avg_re << endl;*/
				//fout << "node " << c << " dead !" << endl;
				die = 1;
				//print_energy();
				int e(0);
				/*while (sink.buffer[e].data != -1)
				{
				//fout << "src: " << sink.buffer[e].src << " dst: " << sink.buffer[e].dst << " data: " << sink.buffer[e].data << " time: " << sink.buffer[e].time << " sec" << endl;
				e++;
				}*/
				avg_t += t;
				/*fout << t << endl;
				fout << "total = " << total << endl;
				fout << "drop = " << drop << endl;
				fout << "macdrop = " << macdrop << endl;
				fout << "sink 有" << e << endl; //total封包數*/
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

				/*非爆炸區*/
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
				CH_set(p, round);
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
	return 0;
}

