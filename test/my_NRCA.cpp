#include"common.h"

/*�ܰʹ���ѼƳ]�w*/
#define S_NUM 900 //�P�����`��
#define compression_rate 0.25 //���Y�v �]1�h�S�����Y
#define CH_transmit 120 //CH trans frequency


#define sensing_frequency 360 //�`�Wsensing frequency
// #define SensingRate_type2f 480
// #define SensingRate_type3f 600
#define CH_frequency 120
/*
#define freq_change_switch 0 //0�� 1�} �O�_�n�ϸ�ƶq��M�ɼW���}��
#define b_t 10800 //�jT �C�h�֬�}�@�� �pT �C�@���}�h�֬�
#define s_t 1800
#define bomb_f3 45 //�z��sensing frequency
#define bomb_f4 60
#define bomb_f5 90
*/
struct C{
	double x, y;
};

struct Node
{
	int id, x, y, CH, region;  //
    double energy;//node information
	double dist_to_sink;//dist = distance to sink
	Package receive;
	Package sense;
	Package buffer[NODE_BUFFER2];//buffer in sensor node
};

ofstream fout("NRCA_output.txt");
Node node[S_NUM];
Sink sink;
list<Node> R1_cluster, R2_cluster, R3_cluster, R4_cluster;

double distance(int a, int b){   //�`�Ia�Ǩ�b���Z��
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
        if (nodeAlreadyInCluster) { // �p�G�`�I�w�g�Q��J�F�Y�Ӱϰ줤�A�h���L�����j��
            continue; 
        }

        node[n].id = n;
		node[n].x = rand() % 400 + 1;  //�`�Ix�y��1~400�H����
		node[n].y = rand() % 400 + 1;  //�`�Iy�y��1~400�H����
		node[n].CH = n;
        // node[n].type = rand() % 3 + 1; //sensing rate
		node[n].energy = MAX_energy;
		node[n].dist_to_sink = distance(n, SINKID);  //�Z����sink
		if( node[n].x <= 200 && node[n].y <= 200 ){  //(x,y) = (1~200, 1~200)
			node[n].region = 1;
			R1_cluster.push_back(node[n]);
        }
        else if( node[n].x > 200 && node[n].y <= 200 ){  //(x,y) = (201~400, 1~200)
			node[n].region = 2;
			R2_cluster.push_back(node[n]);
        }        
        else if( node[n].x <= 200 && node[n].y > 200 ){  //(x,y) = (1~200, 201~400)
			node[n].region = 3;
			R3_cluster.push_back(node[n]);
        }        
        else{			//(x,y) = (201~400, 201~400)
			node[n].region = 4;
			R4_cluster.push_back(node[n]);
        }
    }
}

void sink_buffer_init(){
	for (int i = 0; i < SINK_BUFFER_SIZE; i++){
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

double find_max_energy(const list<Node>& Region_cluster){     //��cluster�����̤j��q
	double cluster_max_energy = Region_cluster.front().energy;
	for(const auto& node : Region_cluster){
        if(node.energy > cluster_max_energy){
            cluster_max_energy = node.energy;
        }
	}
	return cluster_max_energy;
}

Node CH_Selection(list<Node>& cluster){
	double cluster_max_energy = find_max_energy(cluster);
	queue<int> CH_cdd; //cdd candidate
	int CH;
	Node selected_CH;
	for(auto& nd : cluster){  //�bcluster���M��̤j�Ѿl��q(En)��node�@���Կ�CH(cch)
		if (nd.energy == cluster_max_energy){
			CH_cdd.push(nd.id);
		}
	}
	CH = CH_cdd.front();
	CH_cdd.pop();
	while(!CH_cdd.empty()){
		if(node[CH_cdd.front()].dist_to_sink < node[CH].dist_to_sink){  //����{��CH�Mcdd�Ĥ@���sink���Z��
			CH = CH_cdd.front();
		}
		CH_cdd.pop();
	}
	for (auto& node : cluster){ //����cluster�����Ҧ��`�ICH��CH
		node.CH = CH;
		if (node.id == CH) {
            selected_CH = node;
        }
	}
	return selected_CH;
}

int CheckEnergy(){   //���`�I������^�Ӹ`�IID�A�S���N��^SinkID
	for(int i = 0 ; i < S_NUM ; i++){
		if( node[i].energy <= 0){
			return i;
			break;
		}
	}
	return SINKID;
}

void transaction(Node trans_node, Node CH_node,int time){
	Packet_Generate(trans_node, time);
	Packet_Deliver(trans_node, CH_node);
	Packet_Receive(CH_node);
}

void Packet_Generate(Node G_Node, int time){ //�`�I�ͦ��ʥ]�����
	total++;
	G_Node.sense.src = G_Node.id;
	G_Node.sense.dst = G_Node.CH;
	G_Node.sense.time = time;
	G_Node.energy -= ProbeEnergy;
}

void Packet_Deliver(Node T_node, Node CH_node){
	int drop_rate = rand() % 100 + 1;
	if( drop_rate > 10 || T_node.id == T_node.CH ){  //10% drop rate�άOCH�ۤv�ͦ����ʥ]��ۤv��buffer
		CH_node.receive.dst = T_node.sense.dst;
		CH_node.receive.src = T_node.sense.src;
		CH_node.receive.time = T_node.sense.time;
	}
	else{    //�ʥ]�ᥢ
		macdrop++;
		CH_node.receive.dst = -1;
		CH_node.receive.src = -1;
		CH_node.receive.data = -1;
		CH_node.receive.time = -1;
	}
	double d_to_CH = distance(T_node.id, T_node.CH);
	if(T_node.id != T_node.CH){
		T_node.energy -= TransmitEnergy + d_to_CH * d_to_CH * AmplifierEnergy;
	}
}


void Packet_Receive(Node CH_node){ //buffer���F�n�ܦ�priority queue �����
	if (CH_node.receive.src != CH_node.CH){ //���O�Ӧۦۤv���~����q
		CH_node.energy -= ReceiveEnergy;
	} //drop�ٺ�O����
	int full = 1;
	for (int a = 0; a < NODE_BUFFER1; a++){ //buffer is not full 50 for self-area-sense 50 for other CH 
		if (CH_node.buffer[a].data == -1)
		{
			CH_node.buffer[a].dst = CH_node.receive.dst;
			CH_node.buffer[a].src = CH_node.receive.src;
			CH_node.buffer[a].time = CH_node.receive.time;
			full = 0;
			break;
		}
	}
	if (full == 1){  //priority queue buffer , �p�G�ʥ]�Qdrop���N���ΤF(-1)
		drop++;
	}
	CH_node.receive.dst = -1;
	CH_node.receive.src = -1;
	CH_node.receive.data = -1;
	CH_node.receive.time = -1;
}

int main(){
	srand((unsigned)time(NULL)); //random seed
	for (int round = 0; round < round_number; round++){
		node_deployed();

		/*initialization*/
		packet_init();
		sink_buffer_init();

		/*firts CH selection*/
		Node R1_CH = CH_Selection(R1_cluster);
		Node R2_CH = CH_Selection(R2_cluster);
		Node R3_CH = CH_Selection(R3_cluster);
		Node R4_CH = CH_Selection(R4_cluster);

		int time = 1;
		bool network_die = false;  //1�N���`�I���`�A0�N��S��

		while( !network_die ){


			int dead_node = CheckEnergy(); //���`�I������^�Ӹ`�IID�A�S���N��^SinkID
			if( dead_node < SINKID ){  //���`�I����
				avg_time += time;
				network_die = true;
				break;
			}
			if( time % sensing_frequency == 0){
				for(auto& node : R1_cluster){
					transaction(node, R1_CH, time);
				}
				for(auto& node : R2_cluster){
					transaction(node, R2_CH, time);
				}
				for(auto& node : R3_cluster){
					transaction(node, R3_CH, time);
				}
				for(auto& node : R4_cluster){
					transaction(node, R4_CH, time);
				}
			}
			if(time % CH_frequency == 0){
				int CH[4];
				CH[0] = R1_CH.CH;
				CH[1] = R2_CH.CH;
				CH[2] = R3_CH.CH;
				CH[3] = R4_CH.CH;
				
				CH_to_Sink(CH[1]);
				CH_to_Region(CH[2], 1);
				CH_to_Region(CH[3], 1);
			}

			time++;
		}

	}
	total /= round_number;
	macdrop /= round_number;
	drop /= round_number;
	avg_t /= round_number;
	fout << "avg_time : " << avg_time << endl;
	fout << "avg_total : " << total << endl;
	fout << "avg_macdrop : " << macdrop << endl;
	fout << "avg_drop : " << drop << endl;
	fout << "avg_PLR : " << (drop + macdrop) / total << endl;
	return 0;
}