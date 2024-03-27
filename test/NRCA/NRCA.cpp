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
#define freq_change_switch 0 //0關 1開 是否要使資料量突然暴增的開關
#define b_t 10800 //大T 每多少秒開一次 小T 每一次開多少秒
#define s_t 1800
#define bomb_f3 45 //爆炸sensing frequency
#define bomb_f4 60
#define bomb_f5 90

#define ProbeEnergy 0.03 //j (8*200bit*1.5V*25mA*0.5mS = 0.00375j*8 = 0.03j) !
/*(查到的論文:bit*50nj+bit*distance(m)平方*100nj)*/
#define TransmitEnergy 0.00008 //0.000000001*50*200*8 j (50nj/bit by冠中) bit * 8 = byte !(是這樣算嗎)
#define AmplifierEnergy 0.00016 //100*200*0.000000001*8 j/distance^2
#define ReceiveEnergy 0.00008 //0.000000001*50*200j*8 (50nj/bit by冠中)
#define Package_size 200 //bytes 
#define node_buffer 20 //Kbytes (100格)
#define successful_rate 5 //設x 成功率就是100-x%

/*變動實驗參數設定*/
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
	int id, x, y, CH, type, region1;//region1 for 第一層grid , region2 for 第二層grid
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

double type_a = 33, type_b = 33, type_c = 34; //調整QUERE裡面感測資料的比例

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
		ns[i].dtc = distance(i, SINKID);/*距離區SINK*/
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
		ns[i].dtc = distance(i, SINKID); /*距離sink*/
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
		ns[i].dtc = distance(i, SINKID);/*距離區SINKID*/
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
		ns[i].dtc = distance(i, SINKID);/*距離區SINKID*/
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
		ns[i].dtc = distance(i, SINKID);/*距離區SINK*/
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
		ns[i].dtc = distance(i, SINKID); /*距離sink*/
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
		ns[i].dtc = distance(i, SINKID);/*距離區SINKID*/
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
		ns[i].dtc = distance(i, SINKID);/*距離區SINKID*/
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
	if (rate > successful_rate || sender == CH)  /*10% drop rate or CH自己將sense的封包放自己的buffer*/
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
			break;
		}
	}
	if (full == 1) //priority queue buffer , 如果封包被drop掉就不用了(-1)
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
void CHtoRegion2(int CH1, int v) //除了2區以外的區域都丟到2區裡面能量最高的 有能耗
{
	/*取CH到2區+2區到sink的距離相加與其剩餘能量值做加權*/
	int dst = ns[R2].CH;
	double rate(0);/*壓縮率0.25*/
	if (v == 1)
	{
		for (int b = 0; b < NODE_BUFFER1; b++)
		{
			if (ns[CH1].buffer[b].data == -1) //有空的就不用繼續了
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
		//fout <<"模式一收到" <<rate << endl;
	}
	if (v == 2)
	{
		for (int b = NODE_BUFFER1; b < NODE_BUFFER2; b++)
		{
			if (ns[CH1].buffer[b].data == -1) //有空的就不用繼續了
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
		//fout << "模式二收到" << rate << endl;
	}
	rate = ceil(rate * R);
	ns[CH1].energy -= (TransmitEnergy + pow(distance(CH1, dst), 2)*AmplifierEnergy)*rate; //將data合併之後一次傳送 所以耗能這樣算(合併未做)
																						  //fout << "node : " << CH1 << "能量減少 "<<(TransmitEnergy + pow(distance(CH1, dst), 2)*AmplifierEnergy)*rate<<", 因為傳輸給區域2" << endl;
	ns[dst].energy -= (ReceiveEnergy)*rate;
	//fout << "node : " << dst << "能量減少 "<< (ReceiveEnergy)*rate<<" ,因為在區域2收到別的資料" << endl;
	//fout <<"我是節點 "<< dst << " 收到別人的" << endl;
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
				int c = CheckEnergy();/*有一個節點沒電則等於死亡*/
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

				if (t % CHf == 0) //每一分鐘傳到sink 1次
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
