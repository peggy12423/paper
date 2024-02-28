#include"common.h"

/*�ܰʹ���ѼƳ]�w*/
#define S_NUM 900 //�P�����`��
#define R 0.25 //���Y�v �]1�h�S�����Y
#define type3f 360 //�`�Wsensing frequency
#define type4f 480
#define type5f 600
#define CHf 120 //CH trans frequency
#define freq_change_switch 0 //0�� 1�} �O�_�n�ϸ�ƶq��M�ɼW���}��
#define b_t 10800 //�jT �C�h�֬�}�@�� �pT �C�@���}�h�֬�
#define s_t 1800
#define bomb_f3 45 //�z��sensing frequency
#define bomb_f4 60
#define bomb_f5 90
/**/
struct C{
	double x, y;
};

struct Node
{
	int id, x, y, CH, type, region1, RegionID;  //RegionID�Ω�����Ӹ`�I�b�o��region����ID
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

double type_a = 33, type_b = 33, type_c = 34; //�վ�QUERE�̭��P����ƪ����

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
        // �ˬd�Ӹ`�I�O�_�w�g�Q��J�F�Y�Ӱϰ줤
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
            continue; // �p�G�`�I�w�g�Q��J�F�Y�Ӱϰ줤�A�h���L�����j��
        }

        node[n].id = n;
		node[n].x = rand() % 400 + 1;  //�`�Ix�y��1~400�H����
		node[n].y = rand() % 400 + 1;  //�`�Iy�y��1~400�H����
		node[n].CH = n;
        node[n].type = rand() % 3 + 3;//3 4 5
		node[n].energy = MAX_energy;
		node[n].dtc = distance(n, SINKID);  //�Z����sink
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

double find_max_energy(const list<Node>& Region_cluster){     //!energy���w��
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
	for(auto& nd : region){  //�bcluster���M��̤j�Ѿl��q(En)��node�@���Կ�CH(cch)
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
	if (rate > 10 || sender == CH)  /*10% drop rate or CH�ۤv�Nsense���ʥ]��ۤv��buffer*/
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
			/*clean*/
			break;
		}
	}
	if (full == 1) //priority queue buffer , �p�G�ʥ]�Qdrop���N���ΤF(-1)
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
			rate = b - NODE_BUFFER1 + 1;
			sink.buffer[start].data = node[CH].buffer[b].data;
			sink.buffer[start].dst = node[CH].buffer[b].dst;
			sink.buffer[start].src = node[CH].buffer[b].src;
			sink.buffer[start].time = node[CH].buffer[b].time;
			//fout << "CH ID:" << node[CH].id << " src: " << sink.buffer[start].src << " dst: " << sink.buffer[start].dst << " data: " << sink.buffer[start].data << " time: " << sink.buffer[start].time << " sec" << endl;
			start++;
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
			rate = b + 1;
			sink.buffer[start].data = node[CH].buffer[b].data;
			sink.buffer[start].dst = node[CH].buffer[b].dst;
			sink.buffer[start].src = node[CH].buffer[b].src;
			sink.buffer[start].time = node[CH].buffer[b].time;
			//fout << "CH ID:" << node[CH].id << " src: " << sink.buffer[start].src << " dst: " << sink.buffer[start].dst << " data: " << sink.buffer[start].data << " time: " << sink.buffer[start].time << " sec" << endl;
			start++;
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
	int dst = node[R2].CH;
	double rate(0);/*���Y�v0.25*/
	if (v == 1)
	{
		for (int b = 0; b < NODE_BUFFER1; b++)
		{
			if (node[CH1].buffer[b].data == -1) //���Ū��N�����~��F
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
		//fout <<"�Ҧ��@����" <<rate << endl;
	}
	if (v == 2)
	{
		for (int b = NODE_BUFFER1; b < NODE_BUFFER2; b++)
		{
			if (node[CH1].buffer[b].data == -1) //���Ū��N�����~��F
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
		//fout << "�Ҧ��G����" << rate << endl;
	}
	rate = ceil(rate * R);
	node[CH1].energy -= (TransmitEnergy + pow(distance(CH1, dst), 2)*AmplifierEnergy)*rate; //�Ndata�X�֤���@���ǰe �ҥH�ӯ�o�˺�(�X�֥���)
																						  //fout << "node : " << CH1 << "��q��� "<<(TransmitEnergy + pow(distance(CH1, dst), 2)*AmplifierEnergy)*rate<<", �]���ǿ鵹�ϰ�2" << endl;
	node[dst].energy -= (ReceiveEnergy)*rate;
	//fout << "node : " << dst << "��q��� "<< (ReceiveEnergy)*rate<<" ,�]���b�ϰ�2����O�����" << endl;
	//fout <<"�ڬO�`�I "<< dst << " ����O�H��" << endl;
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
		//------------2/28 �ק��o---------------
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
			int c = CheckEnergy();/*���@�Ӹ`�I�S�q�h���󦺤`*/
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
				//fout << "�ϰ�" << i + 1 << "���ϰ줺�����ǿ�ӯ� = " << cons[i] / trans_time[i] << endl;
				}
				for (int i = 0; i < 4; i++)
				{
				//fout << "�ϰ�" << i + 1 << "�������ǿ��2�ӯ� = " << consToR2[i] / ToR2_time[i] << endl;
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
				fout << "sink ��" << e << endl; //total�ʥ]��*/
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

				/*�D�z����*/
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

			if (t % CHf == 0) //�C�@�����Ǩ�sink 1��
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

