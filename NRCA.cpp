#include"common.h"

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
struct C{
	double x, y;
};

struct Node
{
	int id, x, y, CH, type, region1, RegionID;  //RegionID用於紀錄該節點在這個region內的ID
	double energy;//node information
	Package receive;
	Package sense;
	Package buffer[NODE_BUFFER2];//buffer in sensor node
	double dtc;//dist = distance to sink
};

ofstream fout("NRCA_output.txt");
Node node[S_NUM];
Sink sink;
list<Node> R1_cluster, R2_cluster, R3_cluster, R4_cluster;
double cons[4] = { 0,0,0,0 };
int trans_time[4] = { 0,0,0,0 };
double consToR2[4] = { 0,0,0,0 };
int ToR2_time[4] = { 0,0,0,0 };
int toSink(0);
double SD(0);
// int R2 = S_NUM / 4;
// int R3 = S_NUM / 2;
// int R4 = S_NUM * 0.75;
// int R_NUM = R2;

double type_a = 33, type_b = 33, type_c = 34; //調整QUERE裡面感測資料的比例

void print_energy()
{
	fout << node[0].CH << " " << node[R2].CH << " " << node[R3].CH << " " << node[R4].CH << endl;
	for (int i = 0; i < S_NUM; i++)
	{
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

void node_deployed(){
	R1_cluster.clear();
	R2_cluster.clear();
	R3_cluster.clear();
	R4_cluster.clear();
    for(int n = 0 ; n < S_NUM ; n++ ){
        // 檢查該節點是否已經被放入了某個區域中
        bool nodeAlreadyInCluster = false;
        for (const auto& nodeInCluster : R1_cluster) {
            if (nodeInCluster.id == n) {
                nodeAlreadyInCluster = true;
                break;
            }
        }
        for (const auto& nodeInCluster : R2_cluster) {
            if (nodeInCluster.id == n) {
                nodeAlreadyInCluster = true;
                break;
            }
        }
        for (const auto& nodeInCluster : R3_cluster) {
            if (nodeInCluster.id == n) {
                nodeAlreadyInCluster = true;
                break;
            }
        }
        for (const auto& nodeInCluster : R4_cluster) {
            if (nodeInCluster.id == n) {
                nodeAlreadyInCluster = true;
                break;
            }
        }
        
        if (nodeAlreadyInCluster) {
            continue; // 如果節點已經被放入了某個區域中，則跳過本次迴圈
        }

        node[n].id = n;
		node[n].x = rand() % 400 + 1;  //節點x座標1~400隨機值
		node[n].y = rand() % 400 + 1;  //節點y座標1~400隨機值
		node[n].CH = n;
        node[n].type = rand() % 3 + 3;//3 4 5
		node[n].energy = MAX_energy;
		node[n].dtc = distance(n, SINKID);  //距離區sink
		if( node[n].x <= 200 && node[n].y <= 200 ){
            //(x,y) = (1~200, 1~200)
			node[n].region1 = 1;
			R1_cluster.push_back(node[n]);
        }
        else if( node[n].x > 200 && node[n].y <= 200 ){
			//(x,y) = (201~400, 1~200)
			node[n].region1 = 2;
			R2_cluster.push_back(node[n]);
        }        
        else if( node[n].x <= 200 && node[n].y > 200 ){
            //(x,y) = (1~200, 201~400)
			node[n].region1 = 3;
			R3_cluster.push_back(node[n]);
        }        
        else{
			//(x,y) = (201~400, 201~400)
			node[n].region1 = 4;
			R4_cluster.push_back(node[n]);
        }
    }
}

void sink_buffer_init(int sink_buffer_size){
	for (int i = 0; i < sink_buffer_size; i++){
		sink.buffer[i].data = -1;
		sink.buffer[i].dst = -1;
		sink.buffer[i].src = -1;
		sink.buffer[i].time = -1;
	}
}


void packet_init(){
	for (int i = 0; i < S_NUM; i++){
		node[i].receive.data = -1;
		node[i].receive.dst = -1;
		node[i].receive.src = -1;
		node[i].receive.time = -1;
		node[i].sense.data = -1;
		node[i].sense.dst = -1;
		node[i].sense.src = -1;
		node[i].sense.time = -1;
		for (int j = 0; j < NODE_BUFFER2; j++){
			node[i].buffer[j].data = -1;
			node[i].buffer[j].dst = -1;
			node[i].buffer[j].src = -1;
			node[i].buffer[j].time = -1;
		}
	}
}

double find_max_energy(const list<Node>& Region_cluster){     //!energy的預扣
	double cluster_max_energy = Region_cluster.front().energy;
	for(const auto& node : Region_cluster){
        if(node.energy > cluster_max_energy){
            cluster_max_energy = node.energy;
        }
	}
	return cluster_max_energy;
}

/*
double find_avg_energy(int start_node, int end_node, int rnum)
{
	double avg_energy(0.0);
	for (int i = start_node; i <= end_node; i++){
		avg_energy += node[i].energy;
	}
	avg_energy /= rnum;
	return avg_energy;
}
*/

Node CH_Selection(list<Node>& region){
	double cluster_max_energy = find_max_energy(region);
	queue<int> CH_cdd; //cdd candidate
	int CH;
	Node selected_CH;
	for(auto& nd : region){  //在cluster內尋找最大剩餘能量(En)的node作為候選CH(cch)
		if (nd.energy == cluster_max_energy){
			CH_cdd.push(nd.id);
		}
	}
	CH = CH_cdd.front();
	CH_cdd.pop();
	while (!CH_cdd.empty()){
		if (node[CH_cdd.front()].dtc < node[CH].dtc){
			CH = CH_cdd.front();
		}
		CH_cdd.pop();
	}
	for (auto& nd : region){ //start to change CH
		nd.CH = CH;
		if (nd.id == CH) {
            selected_CH = nd;
        }
	}
	return selected_CH;
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
	if (rate > 10 || sender == CH)  /*10% drop rate or CH自己將sense的封包放自己的buffer*/
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
			/*clean*/
			break;
		}
	}
	if (full == 1) //priority queue buffer , 如果封包被drop掉就不用了(-1)
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
			rate = b - NODE_BUFFER1 + 1;
			sink.buffer[start].data = node[CH].buffer[b].data;
			sink.buffer[start].dst = node[CH].buffer[b].dst;
			sink.buffer[start].src = node[CH].buffer[b].src;
			sink.buffer[start].time = node[CH].buffer[b].time;
			//fout << "CH ID:" << node[CH].id << " src: " << sink.buffer[start].src << " dst: " << sink.buffer[start].dst << " data: " << sink.buffer[start].data << " time: " << sink.buffer[start].time << " sec" << endl;
			start++;
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
			rate = b + 1;
			sink.buffer[start].data = node[CH].buffer[b].data;
			sink.buffer[start].dst = node[CH].buffer[b].dst;
			sink.buffer[start].src = node[CH].buffer[b].src;
			sink.buffer[start].time = node[CH].buffer[b].time;
			//fout << "CH ID:" << node[CH].id << " src: " << sink.buffer[start].src << " dst: " << sink.buffer[start].dst << " data: " << sink.buffer[start].data << " time: " << sink.buffer[start].time << " sec" << endl;
			start++;
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
	int dst = node[R2].CH;
	double rate(0);/*壓縮率0.25*/
	if (v == 1)
	{
		for (int b = 0; b < NODE_BUFFER1; b++)
		{
			if (node[CH1].buffer[b].data == -1) //有空的就不用繼續了
			{
				break;
			}
			rate = b + 1;
			node[dst].buffer[NODE_BUFFER1 + b].data = node[CH1].buffer[b].data;
			node[dst].buffer[NODE_BUFFER1 + b].dst = node[CH1].buffer[b].dst;
			node[dst].buffer[NODE_BUFFER1 + b].src = node[CH1].buffer[b].src;
			node[dst].buffer[NODE_BUFFER1 + b].time = node[CH1].buffer[b].time;
		}
		clean(CH1, 0, NODE_BUFFER1);
		//fout <<"模式一收到" <<rate << endl;
	}
	if (v == 2)
	{
		for (int b = NODE_BUFFER1; b < NODE_BUFFER2; b++)
		{
			if (node[CH1].buffer[b].data == -1) //有空的就不用繼續了
			{
				break;
			}
			rate = b + 1 - NODE_BUFFER1;
			node[dst].buffer[b].data = node[CH1].buffer[b].data;
			node[dst].buffer[b].dst = node[CH1].buffer[b].dst;
			node[dst].buffer[b].src = node[CH1].buffer[b].src;
			node[dst].buffer[b].time = node[CH1].buffer[b].time;
		}
		clean(CH1, NODE_BUFFER1, NODE_BUFFER2);
		//fout << "模式二收到" << rate << endl;
	}
	rate = ceil(rate * R);
	node[CH1].energy -= (TransmitEnergy + pow(distance(CH1, dst), 2)*AmplifierEnergy)*rate; //將data合併之後一次傳送 所以耗能這樣算(合併未做)
																						  //fout << "node : " << CH1 << "能量減少 "<<(TransmitEnergy + pow(distance(CH1, dst), 2)*AmplifierEnergy)*rate<<", 因為傳輸給區域2" << endl;
	node[dst].energy -= (ReceiveEnergy)*rate;
	//fout << "node : " << dst << "能量減少 "<< (ReceiveEnergy)*rate<<" ,因為在區域2收到別的資料" << endl;
	//fout <<"我是節點 "<< dst << " 收到別人的" << endl;
	CH2Sink(dst);
}
int CheckEnergy(){
	for (int b = 0; b < S_NUM; b++){
		//fout << "node " << b << "'s energy = " << node[b].energy << endl;
		if (node[b].energy <= 0){
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
	int CH = node[s].CH;
	for (int i = s; i <= e; i++)
	{
		avg_d += node[i].dtc;
		avg_e += node[i].energy;
	}
	avg_d /= rnum;
	avg_e /= rnum;
	t = (node[CH].energy * avg_d) / (avg_e * node[CH].dtc);
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
	int CH = node[s].CH;
	for (int i = s; i <= e; i++)
	{
		avg_d += node[i].dtc;
		avg_e += node[i].energy;
	}
	avg_d /= rnum;
	avg_e /= rnum;
	t = (node[CH].energy * avg_d) / (avg_e * node[CH].dtc);
	return t;
}
void transaction(int j, int t)
{
	Packet_Generate(j, t);
	Packet_Dliver(j, node[j].CH);
	Packet_Receive(node[j].CH);
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

int main(){
	/*sensor initialization*/
	srand((unsigned)time(NULL)); //random seed
	for (int round = 0; round < round_number; round++){
		node_deployed();
		packet_init();

		/*sink initialization*/
		sink.id = SINKID;
		sink_buffer_init(SINK_BUFFER_SIZE);

		/*firts CH selection*/
		Node R1_CH = CH_Selection(R1_cluster);
		Node R2_CH = CH_Selection(R2_cluster);
		Node R3_CH = CH_Selection(R3_cluster);
		Node R4_CH = CH_Selection(R4_cluster);
		//------------2/28 修改到這---------------
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
				/*double avg_re = standard_deviation();
				fout << t << "  " << avg_re << endl;*/
				//fout << t << endl;
				avg_t += t;
				//fout << "node " << c << " dead !" << endl;
				die = 1;
				//print_energy();
				/*for (int i = 0; i < 4; i++)
				{
				//fout << "區域" << i + 1 << "的區域內平均傳輸耗能 = " << cons[i] / trans_time[i] << endl;
				}
				for (int i = 0; i < 4; i++)
				{
				//fout << "區域" << i + 1 << "的平均傳輸區2耗能 = " << consToR2[i] / ToR2_time[i] << endl;
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
						if (node[j].type == 3 && node[j].region1 == b_region)
						{
							transaction(j, t);
						}
					}
				}
				if (t % bomb_f4 == 0)
				{
					for (int j = 0; j < S_NUM; j++)
					{
						if (node[j].type == 4 && node[j].region1 == b_region)
						{
							transaction(j, t);
						}
					}
				}
				if (t % bomb_f5 == 0)
				{
					for (int j = 0; j < S_NUM; j++)
					{
						if (node[j].type == 5 && node[j].region1 == b_region)
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
						if (node[j].type == 3 && node[j].region1 != b_region)
						{
							transaction(j, t);
						}
					}
				}
				if (t % type4f == 0)
				{
					for (int j = 0; j < S_NUM; j++)
					{
						if (node[j].type == 4 && node[j].region1 != b_region)
						{
							transaction(j, t);
						}
					}
				}
				if (t % type5f == 0)
				{
					for (int j = 0; j < S_NUM; j++)
					{
						if (node[j].type == 5 && node[j].region1 != b_region)
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
						if (node[j].type == 3) //CH need to sense
						{
							transaction(j, t);
						}
					}
				}
				if (t % type4f == 0)
				{
					for (int j = 0; j < S_NUM; j++)
					{
						if (node[j].type == 4) //CH need to sense
						{
							transaction(j, t);
						}
					}
				}
				if (t % type5f == 0)
				{
					for (int j = 0; j < S_NUM; j++)
					{
						if (node[j].type == 5) //CH need to sense
						{
							transaction(j, t);
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
		cout << round+1 << endl;
	}
	total /= round_number;
	macdrop /= round_number;
	drop /= round_number;
	avg_t /= round_number;
	fout << "[NRCA]" << endl;
	fout << "avg_time : " << avg_t << endl;
	fout << "avg_total : " << total << endl;
	fout << "avg_macdrop : " << macdrop << endl;
	fout << "avg_drop : " << drop << endl;
	fout << "avg_PLR : " << (drop + macdrop) / total << endl;
	//system("PAUSE");
	return 0;
}

