#include"common.h"
/*變動實驗參數設定*/
#define S_NUM 600 //感測器總數
#define R 0.25 //壓縮率 設1則沒有壓縮
#define type3f 360//常規sensing frequency
#define type4f 480
#define type5f 720 //720
#define reservation_energy_time 10000
#define CHf 120 //CH trans frequency
#define bomb_switch 1 //0關 1開 是否要使用雙層的開關
#define freq_change_switch 1 //0關 1開 是否要使資料量突然暴增的開關
#define b_t 10800 //大T 每多少秒開一次 小T 每一次開多少秒
#define s_t 1800
#define bomb_f3 45 //爆炸sensing frequency
#define bomb_f4 60
#define bomb_f5 90
#define full_th 2
/**/
#define successful_rate 5 //設x 成功率就是100-x%
using namespace std;
struct C{
	double x, y;
};

struct Node
{
	int id, x, y, CH, type, region1, region2;//region1 for 第一層grid , region2 for 第二層grid
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

double type_a = 33, type_b = 33, type_c = 34; //調整QUERE裡面感測資料的比例
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

//資料爆炸
int bomb_times[4] = { 0,0,0,0 };
int normal_times[4] = { 0,0,0,0 };
int old_t[4] = { 0,0,0,0 };
int current_state[4] = { 0,0,0,0 };
int g2[4][4] = { { -1,-1,-1,-1 },{ -1,-1,-1,-1 },{ -1,-1,-1,-1 },{ -1,-1,-1,-1 } }; //用來存放第二層的CH

void sec_ch(int r1, int r2)
{
	double max(0); //目前先選剩餘能量最大的
	int ch2(-1);//這裡打-1是因為有時候這個區域根本就沒有點
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
	g2[r1 - 1][r2 - 1] = ch2; //儲存2層頭
							  //fout << "第" << r1 << "區中的第" << r2 << "區的頭是" << ch2 << endl;
	for (int i = 0; i < S_NUM; i++)
	{
		if (node[i].region1 == r1 && node[i].region2 == r2)
		{
			node[i].CH2 = ch2;
		}
	}
}
void sec_ch_re_check(int r1, int r2) //區2的CH重選機制
{
	double avg_re(0);//此小區的平均能量
	double r2_num(0);//這個小區裡面共有幾個點
	for (int i = 0; i < S_NUM; i++)
	{
		if (node[i].region1 == r1 && node[i].region2 == r2)
		{
			avg_re += node[i].energy;
			r2_num++;
		}
	}
	if (r2_num != 0) //算出這個小區的平均能量
	{
		avg_re /= r2_num;
	}
	if (node[g2[r1 - 1][r2 - 1]].energy < avg_re)
	{
		sec_ch(r1, r2);
	}
}
void sec_grid(int r)  // r是指哪一區,(2區怎麼做)    /*決定是哪一區然後選CH*/
{
	//cout << r << "區開" << endl;
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
	//cout << region1 << "區關" << endl;
	for (int i = 0; i < S_NUM; i++)
	{
		if (node[i].region1 == region1)
		{
			node[i].CH2 = -1;
		}
	}
	for (int i = 0; i < 4; i++) //改回-1
	{
		g2[region1 - 1][i] = -1;
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
C find_center(int s, int e) //尋找區域中所有點的中心盡量平均距離
{
	C c;
	c.x = 0.0; c.y = 0.0;
	for (int i = s; i <= e; i++)
	{
		c.x += node[i].x;
		c.y += node[i].y;
	}
	int n = e - s + 1; //n=區域節點數
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
double find_max_dts(int s, int e, int CH) //只有CH會用到這個function
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

void CH_Selection(int s, int e) //s=start e=end !energy的預扣
{
	double E = find_max_energy(s, e);
	double D = find_max_distance(s, e);
	int start = s;
	int end = e;
	int CH = s;
	double re = node[s].energy - (floor(reservation_energy_time / (node[s].type * 120))*(ProbeEnergy + (TransmitEnergy + pow(distance(s, node[s].CH), 2)*AmplifierEnergy)));//扣掉預約能量來比會比較公平,就算是負數應該也能做判斷 120是指倍數
	double MAX_S = standard(node[s].dtc, re, E, D);
	s += 1;
	for (s; s <= e; s++)//selecting
	{
		re = node[s].energy - (floor(reservation_energy_time / (node[s].type * 120))*(ProbeEnergy + (TransmitEnergy + pow(distance(s, node[s].CH), 2)*AmplifierEnergy)));//扣掉預約能量來比會比較公平,就算是負數應該也能做判斷
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

void Packet_Generate(int now, int t) //generate packet 有能耗
{
	total++;
	node[now].sense.src = node[now].id;
	node[now].sense.dst = node[now].CH;
	node[now].sense.data = node[now].type;
	node[now].sense.time = t;
	node[now].energy -= ProbeEnergy;
	//fout << "node : " << now << "能量減少 "<< ProbeEnergy <<" ,因為產生感測封包 "<< endl;
}
void Packet_Dliver(int sender, int CH) // 有能耗
{
	int rate = rand() % 100 + 1;
	if (rate > successful_rate || sender == CH)  /*10% drop rate or CH自己將sense的封包放自己的buffer*/
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
	if (sender != CH) //CH自己給自己不用扣能量
	{
		node[sender].energy -= TransmitEnergy + d*d*AmplifierEnergy;
		cons[node[sender].region1 - 1] += TransmitEnergy + d*d*AmplifierEnergy;
		trans_time[node[sender].region1 - 1] += 1;
		//fout << "node : " << sender << "能量減少 "<< TransmitEnergy + d*d*AmplifierEnergy <<" ,因為傳送給節點 " << CH << " 感測封包" << endl;
	} //1個封包
}

void Packet_Receive(int CH) //buffer滿了要變成priority queue 有能耗
{
	if (node[CH].receive.src != CH) //不是來自自己的才要扣能量
	{
		node[CH].energy -= ReceiveEnergy;
		//fout << "node : " << CH << "能量減少 "<< ReceiveEnergy << " ,因為收到來自節點 " << node[CH].receive.src << " 的封包" << endl;
	} //drop還算是有收
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
	if (full == 1 && node[CH].receive.data != -1)//priority queue buffer , 如果封包被drop掉就不用了(-1)
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
	if (node[CH].buffer[NODE_BUFFER1].data != -1)/*幫別的CH傳*/
	{
		double rate(0);/*壓縮率0.25*/
		for (int b = NODE_BUFFER1; b < NODE_BUFFER2; b++)
		{
			if (node[CH].buffer[b].data == -1)  //有空的就不用繼續了
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
		//fout << "sink收到" << rate << "個" << endl;
		rate = ceil(rate * R);
		//fout << rate << endl;
		node[CH].energy -= (TransmitEnergy + pow(distance(CH, SINKID), 2)*AmplifierEnergy)*rate; //將data合併之後一次傳送 所以耗能這樣算(合併未做) 因為可知合併的size必在packet的大小之中
																							   //fout << "幫別人成功,我的能量只剩" << node[CH].energy << endl;
																							   //fout << "node : " << CH << "能量減少 "<< (TransmitEnergy + pow(distance(CH, SINKID), 2)*AmplifierEnergy)*rate <<" ,因為幫別人傳給sink" << endl;
		clean(CH, NODE_BUFFER1, NODE_BUFFER2); /*傳完之後刪除掉*/
	}
	else      /*自己傳*/
	{
		double rate(0);/*壓縮率0.25*/
		for (int b = 0; b < NODE_BUFFER1; b++)
		{
			if (node[CH].buffer[b].data == -1) //有空的就不用繼續了
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
		//fout << "sink收到" << rate << "個" << endl;
		rate = ceil(rate * R);
		//fout << rate << endl;
		node[CH].energy -= (TransmitEnergy + pow(distance(CH, SINKID), 2)*AmplifierEnergy)*rate; //將data合併之後一次傳送 所以耗能這樣算(合併未做)
																							   //fout << "自己傳,我的能量只剩" << node[CH].energy << endl;
																							   //fout << "node : " << CH << "能量減少 "<< (TransmitEnergy + pow(distance(CH, SINKID), 2)*AmplifierEnergy)*rate <<" ,因為幫自己傳給sink" << endl;
		clean(CH, 0, NODE_BUFFER1); /*傳完之後刪除掉*/
	}
}
void CHtoRegion2(int CH1, int v) //除了2區以外的區域都丟到2區裡面能量最高的 有能耗
{
	/*取CH到2區+2區到sink的距離相加與其剩餘能量值做加權*/
	int dst = R2;
	double d1 = distance(CH1, R2); //別區到某點
	double d2 = distance(R2, SINKID);  //某點到sink
	double E = find_max_energy(R2, R3 - 1);
	double D = find_max_dts(R2, R3 - 1, CH1); //d1+d2的最大值
	double MAX_s = standard(pow(d1, 2) + pow(d2, 2), node[R2].energy, E, D); //取d1+d2相加的最小值 standard本就是取距離越近越好 此值越大越好
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

	double rate(0);/*壓縮率0.25*/
	if (v == 1)
	{
		for (int b = 0, a = 0; b < NODE_BUFFER1; b++)
		{
			if (node[CH1].buffer[b].data == -1) //有空的就不用繼續了
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
		//fout <<"模式一收到" <<rate << endl;
	}
	if (v == 2)
	{
		for (int b = NODE_BUFFER1, a = NODE_BUFFER1; b < NODE_BUFFER2; b++)
		{
			if (node[CH1].buffer[b].data == -1) //有空的就不用繼續了
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
		//fout << "模式二收到" << rate << endl;
	}
	rate = ceil(rate * R);
	//fout << rate << endl;
	node[CH1].energy -= (TransmitEnergy + pow(distance(CH1, dst), 2)*AmplifierEnergy)*rate; //將data合併之後一次傳送 所以耗能這樣算(合併未做)
																						  //fout << "node : " << CH1 << "能量減少 "<<(TransmitEnergy + pow(distance(CH1, dst), 2)*AmplifierEnergy)*rate<<", 因為傳輸給區域2" << endl;
	consToR2[node[CH1].region1 - 1] += (TransmitEnergy + pow(distance(CH1, dst), 2)*AmplifierEnergy)*rate;
	ToR2_time[node[CH1].region1 - 1] += 1;
	node[dst].energy -= (ReceiveEnergy)*rate;
	//fout << "node : " << dst << "能量減少 "<< (ReceiveEnergy)*rate<<" ,因為在區域2收到別的資料" << endl;
	//fout <<"我是節點 "<< dst << " 收到別人的" << endl;
	CH2Sink(dst);
}
void g2toCH(int CH1, int CH2) //CH1 傳送 CH2 收
{
	double rate(0);/*壓縮率0.25*/
	for (int b = 0, a = 0; b < NODE_BUFFER1; b++)
	{
		if (node[CH1].buffer[b].data == -1) //有空的就不用繼續了
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
	node[CH1].energy -= (TransmitEnergy + pow(distance(CH1, CH2), 2)*AmplifierEnergy)*rate; //將data合併之後一次傳送 所以耗能這樣算(合併未做)
																						  //fout << "node : " << CH1 << "能量減少 " << (TransmitEnergy + pow(distance(CH1, CH2), 2)*AmplifierEnergy)*rate << ", 因為從小區傳給大區" << endl;
	cons[node[CH1].region1 - 1] += (TransmitEnergy + pow(distance(CH1, CH2), 2)*AmplifierEnergy)*rate; //算是區域內的能耗
	trans_time[node[CH1].region1 - 1] += 1; //算是區域內的傳送次數
	node[CH2].energy -= (ReceiveEnergy)*rate;
	//fout << "node : " << CH2 << "能量減少 " << (ReceiveEnergy)*rate << " ,因為收到小區資料" << endl;
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
	if (((avg_energy - node[node[s].CH].energy) / avg_energy) >= 0.15) //這個值不一定大於0 , CH的花費不一定比周圍高 ! 因為資料壓縮的關係
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


/*如果使用資料壓縮  , 是否要以資料量大小做策略 , 感測的耗能相比於CH竟然比較大很多(反映在CH剩餘能量竟然還比平均能量高) due to dtc,比例等等,為何CH選區中心會比選靠近區2還要長壽*/
/*code中實際上沒有壓縮*/
/*區域三的平均耗能太高*/
/*目標:合*/
/*門檻,判斷,第二層的traffic控制*/
/*2區爆炸怎麼辦*/
/*CH判斷參數怎麼改*/
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
			//node[i].dtc = sqrt(pow(abs(node[i].x - 100), 2) + pow(abs(node[i].y - 100), 2)); /*距離區中心*/
			//node[i].dtc = abs(node[i].x - 200);/*距離區2*/
			if (i == R2 - 1)//距離區所有點中心
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
			//node[i].dtc = sqrt(pow(abs(node[i].x - 300), 2) + pow(abs(node[i].y - 100), 2)); /*距離區中心*/
			//node[i].dtc = sqrt(pow(abs(node[i].x - SINKID), 2) + pow(abs(node[i].y - 0), 2)); /*距離sink*/
			if (i == R3 - 1)//距離區所有點中心
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
			//node[i].dtc = sqrt(pow(abs(node[i].x - 100), 2) + pow(abs(node[i].y - 300), 2));/*距離區中心*/
			//node[i].dtc = sqrt(pow(abs(node[i].x - 200), 2) + pow(abs(node[i].y - 200), 2));/*距離區2*/
			if (i == R4 - 1)//距離區所有點中心
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
			//node[i].dtc = sqrt(pow(abs(node[i].x - 300), 2) + pow(abs(node[i].y - 300), 2));/*距離區中心*/
			//node[i].dtc = abs(node[i].y - 200);/*距離區2*/
			if (i == S_NUM - 1)//距離區所有點中心
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
			int c = CheckEnergy();/*有一個節點沒電則等於死亡*/
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
				//fout << "區域" << i + 1 << "的區域內平均傳輸耗能 = " << cons[i] / trans_time[i] << endl;
				}
				for (int i = 0; i < 4; i++)
				{
				//fout << "區域" << i + 1 << "的平均傳輸區2耗能 = " << consToR2[i] / ToR2_time[i] << endl;
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
				/*浮點數運算如果運算元是int 則有可能會產生出只有整數的結果 所以要這樣寫*/
				//double PLR = 0;
				//PLR = (macdrop + drop);
				//PLR /= total;
				/**/
				//fout << "packet loss rate = " << PLR << endl;
				//fout << "sink 有" << e << endl; //total封包數*/
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

				/*非爆炸區*/
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



			if (t % CHf == 0) //每一分鐘傳到sink 1次
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
							int Pcount(0);//判斷各個小頭的累積量是否可以合併了
							for (int j = 0; j < 4; j++)
							{
								if (g2[i][j] != -1)
								{
									//cout << Pcount << endl;
									Pcount += Packet_num(g2[i][j]); //只有算前面BUFFER1(透過正常CH(模式1)獲得的封包)
									if (i != 1) //區2
									{
										g2toCH(g2[i][j], CH[i]);
									}
									else //非區2
									{
										CH2Sink(g2[i][j]); //區域2的小頭直接傳給SINK
									}
									sec_ch_re_check(i + 1, j + 1);
								}
							}
							if (Pcount < NODE_BUFFER1) //表示這一區這一次的封包數量不是爆炸
							{
								//cout <<i+1 <<"區的小頭總共有"<< Pcount << endl;
								if (t - old_t[i] == CHf) //測試是否連續:如果現在的t減掉過去的t = CHf的話代表連續出現爆炸
								{
									normal_times[i]++;
								}
								else
								{
									normal_times[i] = 1; //未連續 視為第一次
								}
								old_t[i] = t;
								if (normal_times[i] == full_th) //連3正常
								{
									//cout <<i<< "區合 時間是" <<t<< endl;
									normal_times[i] = 0;
									bomb_cancel(i + 1);//傳進去的是區域編號，不是陣列數字
									current_state[i] = 0;
								}
							}
						}
						else
						{
							if (Packet_num(CH[i]) == NODE_BUFFER1)
							{
								//cout << i + 1 << "區爆炸了" << endl;
								if (t - old_t[i] == CHf) //測試是否連續:如果現在的t減掉過去的t = CHf的話代表連續出現爆炸
								{
									bomb_times[i]++;
								}
								else
								{
									bomb_times[i] = 1; //未連續 視為第一次
								}
								old_t[i] = t;
								if (bomb_times[i] == full_th) //連3爆
								{
									//cout <<i<< "區爆 時間是" << t << endl;
									bomb_times[i] = 0;
									sec_grid(i + 1);//傳進去的是區域編號，不是陣列數字
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

