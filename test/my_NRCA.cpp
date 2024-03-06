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
list<Node> WSN;

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
	WSN.clear();
    for(int n = 0 ; n < S_NUM ; n++ ){
		// �ˬd�Ӹ`�I�O�_�w�g�Q��J�F�Y�Ӱϰ줤
        bool nodeAlreadyInCluster = false;
        for (const auto& wsn : WSN) {
            if (wsn.id == n) {
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
			WSN.push_back(node[n]);
        }
        else if( node[n].x > 200 && node[n].y <= 200 ){  //(x,y) = (201~400, 1~200)
			node[n].region = 2;
			R2_cluster.push_back(node[n]);
			WSN.push_back(node[n]);
        }        
        else if( node[n].x <= 200 && node[n].y > 200 ){  //(x,y) = (1~200, 201~400)
			node[n].region = 3;
			R3_cluster.push_back(node[n]);
			WSN.push_back(node[n]);
        }        
        else{			//(x,y) = (201~400, 201~400)
			node[n].region = 4;
			R4_cluster.push_back(node[n]);
			WSN.push_back(node[n]);
        }
    }
}

void packet_init(list<Node>& cluster){
	for(auto& node : cluster){
		node.receive.data = -1;
		node.receive.dst = -1;
		node.receive.src = -1;
		node.receive.time = -1;
		node.sense.data = -1;
		node.sense.dst = -1;
		node.sense.src = -1;
		node.sense.time = -1;
		for (int j = 0; j < NODE_BUFFER2; j++){
			node.buffer[j].data = -1;
			node.buffer[j].dst = -1;
			node.buffer[j].src = -1;
			node.buffer[j].time = -1;
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

double find_max_energy(const list<Node>& cluster){     //��cluster�����̤j��q
	double cluster_max_energy = cluster.front().energy;
	for(const auto& node : cluster){
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

int Check_Life(list<Node>& WSN){   //���`�I������^�Ӹ`�IID�A�S���N��^SinkID
	for(auto& wsn : WSN){
		if( wsn.energy <= 0){
			return wsn.id;
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

void CH_to_Sink(Node CH_node){ //�����
	int start = 0;/*sink��buffer�q����}�l�O�Ū�*/
	for (int b = 0; b < SINK_BUFFER_SIZE; b++){
		if (sink.buffer[b].data == -1){
			start = b;
			break;
		}
	}
	if (CH_node.buffer[NODE_BUFFER1].data != -1){    //���O��CH��
		double rate = 0;/*���Y�v0.25*/
		for (int b = NODE_BUFFER1; b < NODE_BUFFER2; b++){
			if (CH_node.buffer[b].data == -1){  //���Ū��N�����~��F
				break;
			}
			rate = b - NODE_BUFFER1 + 1;
			sink.buffer[start].data = CH_node.buffer[b].data;
			sink.buffer[start].dst = CH_node.buffer[b].dst;
			sink.buffer[start].src = CH_node.buffer[b].src;
			sink.buffer[start].time = CH_node.buffer[b].time;
			//fout << "CH ID:" << CH_node.id << " src: " << sink.buffer[start].src << " dst: " << sink.buffer[start].dst << " data: " << sink.buffer[start].data << " time: " << sink.buffer[start].time << " sec" << endl;
			start++;
		}
		//fout << "sink����" << rate << "��" << endl;
		rate = ceil(rate * compression_rate);
		//fout << rate << endl;
		CH_node.energy -= (TransmitEnergy + pow(distance(CH_node.id, SINKID), 2)*AmplifierEnergy)*rate; //�Ndata�X�֤���@���ǰe �ҥH�ӯ�o�˺�(�X�֥���) �]���i���X�֪�size���bpacket���j�p����
																							   //fout << "���O�H���\,�ڪ���q�u��" << CH_node.energy << endl;
																							   //fout << "node : " << CH << "��q��� "<< (TransmitEnergy + pow(distance(CH, SINKID), 2)*AmplifierEnergy)*rate <<" ,�]�����O�H�ǵ�sink" << endl;
		clean(CH_node, NODE_BUFFER1, NODE_BUFFER2); /*�ǧ�����R����*/
	}
	else{      /*�ۤv��*/
		double rate = 0;/*���Y�v0.25*/
		for (int b = 0; b < NODE_BUFFER1; b++){
			if (CH_node.buffer[b].data == -1){ //���Ū��N�����~��F
				break;
			}
			rate = b + 1;
			sink.buffer[start].data = CH_node.buffer[b].data;
			sink.buffer[start].dst = CH_node.buffer[b].dst;
			sink.buffer[start].src = CH_node.buffer[b].src;
			sink.buffer[start].time = CH_node.buffer[b].time;
			//fout << "CH ID:" << CH_node.id << " src: " << sink.buffer[start].src << " dst: " << sink.buffer[start].dst << " data: " << sink.buffer[start].data << " time: " << sink.buffer[start].time << " sec" << endl;
			start++;
		}
		//fout << "sink����" << rate << "��" << endl;
		rate = ceil(rate * compression_rate);
		//fout << rate << endl;
		CH_node.energy -= (TransmitEnergy + pow(distance(CH, SINKID), 2)*AmplifierEnergy)*rate; //�Ndata�X�֤���@���ǰe �ҥH�ӯ�o�˺�(�X�֥���)
																							   //fout << "�ۤv��,�ڪ���q�u��" << CH_node.energy << endl;
																							   //fout << "node : " << CH << "��q��� "<< (TransmitEnergy + pow(distance(CH, SINKID), 2)*AmplifierEnergy)*rate <<" ,�]�����ۤv�ǵ�sink" << endl;
		clean(CH_node, 0, NODE_BUFFER1); /*�ǧ�����R����*/
	}
}

void clean(Node CH_node, int start, int end){
	for (start; start < end; start++){
		CH_node.buffer[start].data = -1;
		CH_node.buffer[start].dst = -1;
		CH_node.buffer[start].src = -1;
		CH_node.buffer[start].time = -1;
	}
}

void CH1_to_CH2(Node CH1, Node CH2, int v){ //���F2�ϥH�~���ϰ쳣���2�ϸ̭���q�̰��� �����
	/*��CH��2��+2�Ϩ�sink���Z���ۥ[�P��Ѿl��q�Ȱ��[�v*/
	double rate = 0;/*���Y�v0.25*/
	if (v == 1)	{
		for (int b = 0; b < NODE_BUFFER1; b++)		{
			if (CH1.buffer[b].data == -1){ //���Ū��N�����~��F
				break;
			}
			rate = b + 1;
			CH2.buffer[NODE_BUFFER1 + b].data = CH1.buffer[b].data;
			CH2.buffer[NODE_BUFFER1 + b].dst = CH1.buffer[b].dst;
			CH2.buffer[NODE_BUFFER1 + b].src = CH1.buffer[b].src;
			CH2.buffer[NODE_BUFFER1 + b].time = CH1.buffer[b].time;
		}
		clean(CH1, 0, NODE_BUFFER1);
		//fout <<"�Ҧ��@����" <<rate << endl;
	}
	if (v == 2){
		for (int b = NODE_BUFFER1; b < NODE_BUFFER2; b++){
			if (CH1.buffer[b].data == -1){  //���Ū��N�����~��F
				break;
			}
			rate = b + 1 - NODE_BUFFER1;
			CH2.buffer[b].data = CH1.buffer[b].data;
			CH2.buffer[b].dst = CH1.buffer[b].dst;
			CH2.buffer[b].src = CH1.buffer[b].src;
			CH2.buffer[b].time = CH1.buffer[b].time;
		}
		clean(CH1, NODE_BUFFER1, NODE_BUFFER2);
		//fout << "�Ҧ��G����" << rate << endl;
	}
	rate = ceil(rate * R);
	CH1.energy -= (TransmitEnergy + pow(distance(CH1.id, CH2.id), 2)*AmplifierEnergy)*rate; //�Ndata�X�֤���@���ǰe �ҥH�ӯ�o�˺�(�X�֥���)
																						  //fout << "node : " << CH1 << "��q��� "<<(TransmitEnergy + pow(distance(CH1, dst), 2)*AmplifierEnergy)*rate<<", �]���ǿ鵹�ϰ�2" << endl;
	CH2.energy -= (ReceiveEnergy)*rate;
	//fout << "node : " << dst << "��q��� "<< (ReceiveEnergy)*rate<<" ,�]���b�ϰ�2����O�����" << endl;
	//fout <<"�ڬO�`�I "<< dst << " ����O�H��" << endl;
	CH2Sink(CH2);
}

int main(){
	srand((unsigned)time(NULL)); //random seed
	for (int round = 0; round < round_number; round++){
		node_deployed();

		/*initialization*/
		packet_init(R1_cluster);
		packet_init(R2_cluster);
		packet_init(R3_cluster);
		packet_init(R4_cluster);
		sink_buffer_init();

		/*firts CH selection*/
		Node R1_CH = CH_Selection(R1_cluster);
		Node R2_CH = CH_Selection(R2_cluster);
		Node R3_CH = CH_Selection(R3_cluster);
		Node R4_CH = CH_Selection(R4_cluster);
		int countround[4] = {0, 0, 0, 0};

		int time = 1;
		bool network_die = false;  //1�N���`�I���`�A0�N��S��

		while( !network_die ){
			if(countround[0] == 0)
				countround[0] = 10;
			if(countround[1] == 0)
				countround[1] = 3;
			if(countround[2] == 0)
				countround[2] = 10;
			if(countround[3] == 0)
				countround[3] = 10;

			/*��|�Ӱϰ쪺�`�I���list WSN*/
			WSN.clear();			
			WSN.insert(WSN.end(), R1_cluster.begin(), R1_cluster.end());
			WSN.insert(WSN.end(), R2_cluster.begin(), R2_cluster.end());
			WSN.insert(WSN.end(), R3_cluster.begin(), R3_cluster.end());
			WSN.insert(WSN.end(), R4_cluster.begin(), R4_cluster.end());

			int dead_node = Check_Life(WSN); //���`�I������^�Ӹ`�IID�A�S���N��^SinkID
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
				CH_to_Sink(R2_CH);
				CH1_to_CH2(R1_CH, R2_CH, 1);
				CH1_to_CH2(R3_CH, R2_CH, 1);
				CH1_to_CH2(R4_CH, R2_CH, 1);

				countround[0]--;
				if(countround[0] == 0)
					CH_Selection(R1_cluster);
				if(countround[1] == 0)
					CH_Selection(R2_cluster);
				if(countround[2] == 0)
					CH_Selection(R3_cluster);
				if(countround[3] == 0)
					CH_Selection(R4_cluster);
			}
			time++;
		}
	}
	total /= round_number;
	macdrop /= round_number;
	drop /= round_number;
	avg_time /= round_number;
	fout << "avg_time : " << avg_time << endl;
	fout << "avg_total : " << total << endl;
	fout << "avg_macdrop : " << macdrop << endl;
	fout << "avg_drop : " << drop << endl;
	fout << "avg_PLR : " << (drop + macdrop) / total << endl;
	return 0;
}