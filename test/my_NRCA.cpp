#include"common.h"

/*�ܰʹ���ѼƳ]�w*/
#define S_NUM 900 //�P�����`��
#define compression_rate 0.25 //���Y�v �]1�h�S�����Y
#define CH_transmit 120 //CH trans frequency


#define SensingRate_type1f 360 //�`�Wsensing frequency
#define SensingRate_type2f 480
#define SensingRate_type3f 600
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
	int id, x, y, CH, type, region;  //
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
double avg_time = 0;
double drop = 0;
double macdrop = 0;
double total = 0;

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
        
        if (nodeAlreadyInCluster) {
            continue; // �p�G�`�I�w�g�Q��J�F�Y�Ӱϰ줤�A�h���L�����j��
        }

        node[n].id = n;
		node[n].x = rand() % 400 + 1;  //�`�Ix�y��1~400�H����
		node[n].y = rand() % 400 + 1;  //�`�Iy�y��1~400�H����
		node[n].CH = n;
        node[n].type = rand() % 3 + 1; //sensing rate
		node[n].energy = MAX_energy;
		node[n].dtc = distance(n, SINKID);  //�Z����sink
		if( node[n].x <= 200 && node[n].y <= 200 ){
            //(x,y) = (1~200, 1~200)
			node[n].region = 1;
			R1_cluster.push_back(node[n]);
        }
        else if( node[n].x > 200 && node[n].y <= 200 ){
			//(x,y) = (201~400, 1~200)
			node[n].region = 2;
			R2_cluster.push_back(node[n]);
        }        
        else if( node[n].x <= 200 && node[n].y > 200 ){
            //(x,y) = (1~200, 201~400)
			node[n].region = 3;
			R3_cluster.push_back(node[n]);
        }        
        else{
			//(x,y) = (201~400, 201~400)
			node[n].region = 4;
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
	while (!CH_cdd.empty()){
		if (node[CH_cdd.front()].dtc < node[CH].dtc){
			CH = CH_cdd.front();
		}
		CH_cdd.pop();
	}
	for (auto& nd : cluster){ //start to change CH
		nd.CH = CH;
		if (nd.id == CH) {
            selected_CH = nd;
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

void Packet_Generate(int now, int t){ //generate packet �����
	total++;
	node[now].sense.src = node[now].id;
	node[now].sense.dst = node[now].CH;
	node[now].sense.data = node[now].type;
	node[now].sense.time = t;
	node[now].energy -= ProbeEnergy;
}

void transaction(int )

void SensingRate(int time){
	if( time % SensingRate_type1f == 0){
		for( int i = 0 ; i < S_NUM ; i++){
			if(node[i].type == 3  && node[i].region != 0){
				transaction(i, time);
			}
		}
	}
	if( time % SensingRate_type2f == 0){
		for( int i = 0 ; i < S_NUM ; i++){
			if(node[i].type == 4  && node[i].region != 0){
				transaction(i, time);
			}
		}
	}
	if( time % SensingRate_type3f == 0){
		for( int i = 0 ; i < S_NUM ; i++){
			if(node[i].type == 5  && node[i].region != 0){
				transaction(i, time);
			}
		}
	}


}

int main(){
	srand((unsigned)time(NULL)); //random seed
	for (int round = 0; round < round_number; round++){
		node_deployed();
		packet_init();

		/*Sink initialization*/
		sink.id = SINKID;
		sink_buffer_init(SINK_BUFFER_SIZE);

		/*firts CH selection*/
		Node R1_CH = CH_Selection(R1_cluster);
		Node R2_CH = CH_Selection(R2_cluster);
		Node R3_CH = CH_Selection(R3_cluster);
		Node R4_CH = CH_Selection(R4_cluster);

		int time = 1;
		int network_die = 0;  //1�N���`�I���`�A0�N��S��

		int check_life = CheckEnergy(); //���`�I������^�Ӹ`�IID�A�S���N��^SinkID
		if( check_life < SINKID ){  //���`�I����
			avg_time += time;
			network_die = 1;
			break;
		}

		SensingRate(time);
		

	}
}