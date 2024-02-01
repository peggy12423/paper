#include "common.h"

/*�ܰʹ���ѼƳ]�w*/
#define S_NUM 1000 //�P�����`��
#define R 0.25 //���Y�v �]1�h�S�����Y
#define type3f 360 //�`�Wsensing frequency
#define type4f 480
#define type5f 600
#define CHf 120 //CH trans frequency
#define freq_change_switch 1 //0�� 1�} �O�_�n�ϸ�ƶq��M�ɼW���}��
#define b_t 10800 //�jT �C�h�֬�}�@�� �pT �C�@���}�h�֬�
#define s_t 1800
#define bomb_f3 45 //�z��sensing frequency
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
queue<RREQ> Q; //�ΨӸ˨C�@�����|
void RREQ_BC(RREQ r) //�s��=��ۤv�F�񪺾F�~�令1,�åB�~�Ӹ��|���I
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
void Packet_Generate(int now, int t) //generate packet �����
{
	total++;
	node[now].sense.src = node[now].id;
	node[now].sense.dst = SINKID;
	node[now].sense.data = node[now].type;
	node[now].sense.time = t;
	node[now].energy -= ProbeEnergy;
	//dout << "src: " << node[now].sense.src << " dst: " << node[now].sense.dst << " data: " << node[now].sense.data << " time: " << node[now].sense.time << " sec" << endl;
	//fout << "node : " << now << "��q��� " << ProbeEnergy << " ,�]�����ͷP���ʥ] " << endl;
}
int Packet_Dliver(int sender, int receiver) // �����
{
	double d = distance(sender, receiver);
	node[sender].energy -= TransmitEnergy + d*d*AmplifierEnergy;
	int rate = rand() % 100 + 1;
	if (rate > 5)  /*10% drop rate or CH�ۤv�Nsense���ʥ]��ۤv��buffer*/
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
	//fout << "node : " << sender << "��q��� " << TransmitEnergy + d*d*AmplifierEnergy << " ,�]���ǰe���`�I " << receiver << " �P���ʥ]" << endl;
	//fout << "node : " << receiver << "��q��� " << ReceiveEnergy << " ,�]������Ӧ۸`�I " << sender << " ���ʥ]" << endl;
	//1�ӫʥ]
}
void DeliveToSink(int sender) //�����
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
	//fout << "node : " << sender << "��q��� " << TransmitEnergy + d*d*AmplifierEnergy << " ,�]���ǰe��sink�ʥ]" << endl;
}
void set_visited()
{
	for (int i = 0; i < S_NUM; i++)
	{
		node[i].visited = 0;
	}
}
void AODV_establish(int node_id) //���|�إ�
{
	RREQ r;
	r.route.push(node_id);
	r.hop_count = 0;
	node[node_id].visited = 1;
	Q.push(r);
	while (!Q.empty()) //BFS(QUEUE)���@�k����K��s��
	{
		RREQ_BC(Q.front());
		Q.pop();
	}
	set_visited();//���͸��|����N�i�H��^0
	RREQ min;
	if (!route_table.empty())
	{
		min.hop_count = route_table.front().hop_count;
		min.route = route_table.front().route;
		route_table.pop();
		while (!route_table.empty()) //��ܤ@��hop count�̵u�����|
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
		//cout << "�`�I" << i << "�����|�w�إ�" << endl;
	}
}
void AODV_routing(int node_id, int t)
{
	Packet_Generate(node_id, t);//���ͫʥ]
	int del, recv;
	queue<int>path = node[node_id].route;
	if (!node[node_id].route.empty())
	{
		while (!path.empty()) //��X�Ӥ���~���ǿ�
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
	else //�S�����|
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
		/*AODV���|����*/
		AODV_path_assign();
		/*traffic start*/
		int bombing(0);
		int b_region(0);
		int die(0);//die=1�N���]�j��
		int t(1);
		while (!die)
		{
			//fout << "time = " << t << endl;
			int c = CheckEnergy();/*���@�Ӹ`�I�S�q�h���󦺤`*/
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
				//fout << "sink ��" << e << endl; //total�ʥ]��,���Ƿ|�䤣��hsink�����|
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

				/*�D�z����*/
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

