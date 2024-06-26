#include <iostream>
#include <time.h>
#include <math.h>
#include <fstream>
#include <queue>     

#define SINKBUFFER 10000000
#define MAX_energy 6480//j (1.5V*600mA*3600sec*2 = 6480j) 2*3號電池
#define SINKID 2000
#define SINK_X 400
#define SINK_Y 0

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
#define trans_dis 60 //m 80幾乎可以確定他傳的到sink(AODV)

/*變動實驗參數設定*/
#define round_number 20
#define E_NUM 1000

using namespace std;
int S_NUM = 400;
int Ere_switch = 0; //1代表要輸出Ere

struct P
{
	int src;
	int dst;
	int data;
	int time;
};
struct N
{
	int id, x, y, CH, type,region1;//region1 for 第一層grid , region2 for 第二層grid
	double random_num;
	double energy;//node information
	P receive;
	P sense;
	P buffer[100];//buffer in sensor node
	double dtc;//dist = distance to sink
	int non; //nomber of negihbor
	int neighbor[1000]; //存取節點的鄰居
	int visited;
	queue<int>route;
};
struct S
{
	int id;//node information
	P buffer[SINKBUFFER];//buffer
};
struct RREQ
{
	queue<int>route;
	int hop_count;
};
ofstream fout("AODV_spe2.txt");
N ns[2000];
S sink;
double avg_t(0);
double drop(0);
double macdrop(0);
double total(0);
int R2, R3, R4;

void print_energy()
{
	for (int i = 0; i < S_NUM; i++)
	{
		fout << "node" << ns[i].id << " x : " << ns[i].x << " y : " << ns[i].y << " type : " << ns[i].type << " energy : " << ns[i].energy << " visited = " << ns[i].visited << endl;
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
        ns[i].visited = 0;
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
        ns[i].visited = 0;
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
        ns[i].visited = 0;
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
        ns[i].visited = 0;
        ns[i].non = 0;
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
        ns[i].type = rand() % 3 + 3;//3 4 5
        ns[i].energy = MAX_energy;
        ns[i].visited = 0;
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
        ns[i].visited = 0;
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
        ns[i].visited = 0;
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
        ns[i].visited = 0;
        ns[i].non = 0;
        ns[i].region1 = 4;
    }
}

void special2_node_deployed(){
	R2 = S_NUM * 0.4;  //region 1
	R3 = S_NUM * 0.5;  //region 2
	R4 = S_NUM * 0.9;  //region 3 
	int i = 0;
    for (i; i < R2; i++)
    {
        ns[i].id = i;
        ns[i].x = rand() % 200 + 1;
        ns[i].y = rand() % 200 + 1;
        ns[i].type = rand() % 3 + 3;//3 4 5
        ns[i].energy = MAX_energy;
        ns[i].visited = 0;
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
        ns[i].visited = 0;
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
        ns[i].visited = 0;
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
        ns[i].visited = 0;
        ns[i].non = 0;
        ns[i].region1 = 4;
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
				ns[i].neighbor[ns[i].non] = j;
				ns[i].non++;
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
		for (int i = 0; i < ns[r.route.back()].non; i++)
		{
			if (ns[ns[r.route.back()].neighbor[i]].visited == 0)
			{
				RREQ rreq;
				rreq = r;
				rreq.route.push(ns[r.route.back()].neighbor[i]);
				rreq.hop_count++;
				ns[ns[r.route.back()].neighbor[i]].visited = 1;
				Q.push(rreq);
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
void set_visited()
{
	for (int i = 0; i < S_NUM; i++)
	{
		ns[i].visited = 0;
	}
}
void AODV_establish(int node_id) //路徑建立
{
	RREQ r;
	r.route.push(node_id);
	r.hop_count = 0;
	ns[node_id].visited = 1;
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
	ns[node_id].route = min.route;
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
	queue<int>path = ns[node_id].route;
	if (!ns[node_id].route.empty())
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
		if (ns[b].energy <= 0)
		{
			return b;
			break;
		}
	}
	return SINKID;
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
    fout << "AODV (normal)" << endl;
    for( S_NUM ; S_NUM <= E_NUM ; S_NUM += 100){
		cout << "sensors: " << S_NUM << endl;
		fout << endl << "------------ Sensors " << S_NUM << " ------------" << endl;
        for (int rn = 0; rn < round_number; rn++)
        {
            // node_deployed();
            // special_node_deployed();
			special2_node_deployed();
			
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
                    die = 1;
                    break;
                }
                if (t % type3f == 0)
                {
                    for (int j = 0; j < S_NUM; j++)
                    {
                        if (ns[j].type == 3) //CH need to sense
                        {
                            AODV_routing(j, t);
                        }
                    }
                }
                if (t % type4f == 0)
                {
                    for (int j = 0; j < S_NUM; j++)
                    {
                        if (ns[j].type == 4) //CH need to sense
                        {
                            AODV_routing(j, t);
                        }
                    }
                }
                if (t % type5f == 0)
                {
                    for (int j = 0; j < S_NUM; j++)
                    {
                        if (ns[j].type == 5) //CH need to sense
                        {
                            AODV_routing(j, t);
                        }
                    }
                }
				if( (Ere_switch == 1) && (t % 2000 == 0) ){
					double re_energy = remaining_energy();
					fout << "------time " << t << "------  " << "Remaining energy: " << re_energy << endl;
				}
                t++;
            }
            cout << rn+1 << endl;
        }
	    total /= round_number;
        avg_t /= round_number;
        drop /= round_number;
        macdrop /= round_number;
        double PLR = (drop + macdrop)/ total;
        fout << "avg_lifetime : " << avg_t << endl;
        fout << "avg_total : " << total << endl;
        fout << "avg_drop : " << drop << endl;
        fout << "avg_macdrop : " << macdrop << endl;
        fout << "avg_PLR : " << PLR << endl;
    }
	return 0;
}

