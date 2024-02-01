#include "common.h"

/*變動實驗參數設定*/
#define S_NUM 1000 //感測器總數
#define R 0.25 //壓縮率 設1則沒有壓縮
#define type3f 360 //常規sensing frequency
#define type4f 480
#define type5f 600
#define CHf 120 //CH trans frequency
#define freq_change_switch 1 //0關 1開 是否要使資料量突然暴增的開關
#define b_t 10800 //大T 每多少秒開一次 小T 每一次開多少秒
#define s_t 1800
#define bomb_f3 45 //爆炸sensing frequency
#define bomb_f4 60
#define bomb_f5 90
/**/

Node node[S_NUM];
Sink sink;
int R2 = S_NUM / 4;
int R3 = S_NUM / 2;
int R4 = S_NUM * 0.75;

ofstream fout("output.txt");

void print_energy()
{
	for (int i = 0; i < S_NUM; i++)
	{
		fout << "node" << node[i].id << " x : " << node[i].x << " y : " << node[i].y << " type : " << node[i].type << " energy : " << node[i].energy << " visited = " << node[i].visited << endl;
	}
	fout << "--------------------------------------------------\n";
}
double distance(int sender, int receiver)
{
	if (receiver != SINKID)
	{
		return sqrt(pow(abs(node[sender].x - node[receiver].x), 2) + pow(abs(node[sender].y - node[receiver].y), 2));
	}
	else //SINK
	{
		return sqrt(pow(abs(node[sender].x - SINK_X), 2) + pow(abs(node[sender].y - SINK_Y), 2));
	}
}
void neighbor_init()
{
	for (int i = 0; i < S_NUM; i++)
	{
		for (int j = 0; j < S_NUM; j++)
		{
			if (i != j && distance(i, j) <= trans_dis)
			{
				node[i].neighbor[node[i].non] = j;
				node[i].non++;
			}
		}
	}
}
queue<RREQ> route_table;
queue<RREQ> Q; //用來裝每一條路徑
void RREQ_BC(RREQ r) //廣播=把自己鄰近的鄰居改成1,並且繼承路徑父點
{
	if (distance(r.route.back(), SINKID) <= trans_dis)
	{
		route_table.push(r);
	}
	else
	{
		for (int i = 0; i < node[r.route.back()].non; i++)
		{
			if (node[node[r.route.back()].neighbor[i]].visited == 0)
			{
				RREQ rreq;
				rreq = r;
				rreq.route.push(node[r.route.back()].neighbor[i]);
				rreq.hop_count++;
				node[node[r.route.back()].neighbor[i]].visited = 1;
				Q.push(rreq);
			}
		}
	}
}
void packet_init()
{
	for (int a = 0; a < S_NUM; a++)
	{
		node[a].receive.data = -1;
		node[a].receive.dst = -1;
		node[a].receive.src = -1;
		node[a].receive.time = -1;
		node[a].sense.data = -1;
		node[a].sense.dst = -1;
		node[a].sense.src = -1;
		node[a].sense.time = -1;
		for (int b = 0; b < 100; b++)
		{
			node[a].buffer[b].data = -1;
			node[a].buffer[b].dst = -1;
			node[a].buffer[b].src = -1;
			node[a].buffer[b].time = -1;
		}
	}
}
void Packet_Generate(int now, int t) //generate packet 有能耗
{
	total++;
	node[now].sense.src = node[now].id;
	node[now].sense.dst = SINKID;
	node[now].sense.data = node[now].type;
	node[now].sense.time = t;
	node[now].energy -= ProbeEnergy;
	//dout << "src: " << node[now].sense.src << " dst: " << node[now].sense.dst << " data: " << node[now].sense.data << " time: " << node[now].sense.time << " sec" << endl;
	//fout << "node : " << now << "能量減少 " << ProbeEnergy << " ,因為產生感測封包 " << endl;
}
int Packet_Dliver(int sender, int receiver) // 有能耗
{
	double d = distance(sender, receiver);
	node[sender].energy -= TransmitEnergy + d*d*AmplifierEnergy;
	int rate = rand() % 100 + 1;
	if (rate > 5)  /*10% drop rate or CH自己將sense的封包放自己的buffer*/
	{
		node[receiver].sense.dst = node[sender].sense.dst;
		node[receiver].sense.src = node[sender].sense.src;
		node[receiver].sense.data = node[sender].sense.data;
		node[receiver].sense.time = node[sender].sense.time;
		node[receiver].energy -= ReceiveEnergy;
		return 1;
	}
	else
	{
		macdrop++;
		node[receiver].sense.dst = -1;
		node[receiver].sense.src = -1;
		node[receiver].sense.data = -1;
		node[receiver].sense.time = -1;
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
			sink.buffer[i].data = node[sender].sense.data;
			sink.buffer[i].dst = node[sender].sense.dst;
			sink.buffer[i].src = node[sender].sense.src;
			sink.buffer[i].time = node[sender].sense.time;
			break;
		}
	}
	double d = distance(sender, SINKID);
	node[sender].energy -= TransmitEnergy + d*d*AmplifierEnergy;
	//fout << "node : " << sender << "能量減少 " << TransmitEnergy + d*d*AmplifierEnergy << " ,因為傳送給sink封包" << endl;
}
void set_visited()
{
	for (int i = 0; i < S_NUM; i++)
	{
		node[i].visited = 0;
	}
}
void AODV_establish(int node_id) //路徑建立
{
	RREQ r;
	r.route.push(node_id);
	r.hop_count = 0;
	node[node_id].visited = 1;
	Q.push(r);
	while (!Q.empty()) //BFS(QUEUE)的作法比較貼近廣播
	{
		RREQ_BC(Q.front());
		Q.pop();
	}
	set_visited();//產生路徑之後就可以改回0
	RREQ min;
	if (!route_table.empty())
	{
		min.hop_count = route_table.front().hop_count;
		min.route = route_table.front().route;
		route_table.pop();
		while (!route_table.empty()) //選擇一個hop count最短的路徑
		{
			if (route_table.front().hop_count < min.hop_count)
			{
				min.hop_count = route_table.front().hop_count;
				min.route = route_table.front().route;
			}
			route_table.pop();
		}
	}
	node[node_id].route = min.route;
}
void AODV_path_assign()
{
	for (int i = 0; i < S_NUM; i++)
	{
		AODV_establish(i);
		//cout << "節點" << i << "的路徑已建立" << endl;
	}
}
void AODV_routing(int node_id, int t)
{
	Packet_Generate(node_id, t);//產生封包
	int del, recv;
	queue<int>path = node[node_id].route;
	if (!node[node_id].route.empty())
	{
		while (!path.empty()) //選出來之後才做傳輸
		{
			del = path.front();
			path.pop();
			if (!path.empty())
			{
				recv = path.front();
				int s = Packet_Dliver(del, recv);
				while (s!=1)
				{
					//cout << del << " to " << recv << "FAIL!" << endl;
					s = Packet_Dliver(del, recv);
				}
				//cout << del << " to " << recv << "SUCCESS!" << endl;
			}
			else
			{
				DeliveToSink(del);
			}
		}
	}
	else //沒有路徑
	{
		drop++;
	}
}
int CheckEnergy()
{
	for (int b = 0; b < S_NUM; b++)
	{
		if (node[b].energy <= 0)
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
	for (int rn = 0; rn < roundnumber; rn++)
	{
		int i = 0;
		for (i; i < R2; i++)
		{
			node[i].id = i;
			node[i].x = rand() % 200 + 1;
			node[i].y = rand() % 200 + 1;
			node[i].type = rand() % 3 + 3;//3 4 5
			node[i].energy = MAX_energy;
			node[i].visited = 0;
			node[i].non = 0;
			node[i].region1 = 1;
		}
		for (i; i < R3; i++)
		{
			node[i].id = i;
			node[i].x = rand() % 200 + 201;
			node[i].y = rand() % 200 + 1;
			node[i].type = rand() % 3 + 3;
			node[i].energy = MAX_energy;
			node[i].visited = 0;
			node[i].non = 0;
			node[i].region1 = 2;
		}
		for (i; i < R4; i++)
		{
			node[i].id = i;
			node[i].x = rand() % 200 + 1;
			node[i].y = rand() % 200 + 201;
			node[i].type = rand() % 3 + 3;
			node[i].energy = MAX_energy;
			node[i].visited = 0;
			node[i].non = 0;
			node[i].region1 = 3;
		}
		for (i; i < S_NUM; i++)
		{
			node[i].id = i;
			node[i].x = rand() % 200 + 201;
			node[i].y = rand() % 200 + 201;
			node[i].type = rand() % 3 + 3;
			node[i].energy = MAX_energy;
			node[i].visited = 0;
			node[i].non = 0;
			node[i].region1 = 4;
		}
		packet_init();
		/*sink initialization*/
		sink.id = SINKID;
		/*neighbor_initialization*/
		neighbor_init();
		for (int b = 0; b < SINKBUFFER; b++)
		{
			sink.buffer[b].data = -1;
			sink.buffer[b].dst = -1;
			sink.buffer[b].src = -1;
			sink.buffer[b].time = -1;
		}
		/*AODV路徑產生*/
		AODV_path_assign();
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
				avg_t += t;
				//fout << "node " << c << " dead !" << endl;
				die = 1;
				//print_energy();
				/*int e(0);
				while (sink.buffer[e].data != -1)
				{
				e++;
				}*/
				//fout << t << endl;
				//fout << "total = " << total << endl;
				//fout << "drop = " << drop << endl;
				//fout << "macdrop = " << macdrop << endl;
				//fout << "sink 有" << e << endl; //total封包數,有些會找不到去sink的路徑
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
						if (node[j].type == 3 && node[j].region1 == b_region)
						{
							AODV_routing(j, t);
						}
					}
				}
				if (t % bomb_f4 == 0)
				{
					for (int j = 0; j < S_NUM; j++)
					{
						if (node[j].type == 4 && node[j].region1 == b_region)
						{
							AODV_routing(j, t);
						}
					}
				}
				if (t % bomb_f5 == 0)
				{
					for (int j = 0; j < S_NUM; j++)
					{
						if (node[j].type == 5 && node[j].region1 == b_region)
						{
							AODV_routing(j, t);
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
							AODV_routing(j, t);
						}
					}
				}
				if (t % type4f == 0)
				{
					for (int j = 0; j < S_NUM; j++)
					{
						if (node[j].type == 4 && node[j].region1 != b_region)
						{
							AODV_routing(j, t);
						}
					}
				}
				if (t % type5f == 0)
				{
					for (int j = 0; j < S_NUM; j++)
					{
						if (node[j].type == 5 && node[j].region1 != b_region)
						{
							AODV_routing(j, t);
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
							AODV_routing(j, t);
						}
					}
				}
				if (t % type4f == 0)
				{
					for (int j = 0; j < S_NUM; j++)
					{
						if (node[j].type == 4) //CH need to sense
						{
							AODV_routing(j, t);
						}
					}
				}
				if (t % type5f == 0)
				{
					for (int j = 0; j < S_NUM; j++)
					{
						if (node[j].type == 5) //CH need to sense
						{
							AODV_routing(j, t);
						}
					}
				}
			}

			t++;
		}
		cout << rn << endl;
	}
	total /= roundnumber;
	avg_t /= roundnumber;
	drop /= roundnumber;
	macdrop /= roundnumber;
	double PLR = (drop + macdrop)/ total;
	fout << "avg_t : " << avg_t << endl;
	fout << "avg_total : " << total << endl;
	fout << "avg_drop : " << drop << endl;
	fout << "avg_macdrop : " << macdrop << endl;
	fout << "avg_PLR : " << PLR << endl;
	return 0;
}

