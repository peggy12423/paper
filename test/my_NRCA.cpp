#include"common.h"

/*�ܰʹ���ѼƳ]�w*/
#define S_NUM 900 //�P�����`��
#define compression_rate 0.25 //���Y�v �]1�h�S�����Y
#define CH_transmit 120 //CH trans frequency


#define SensingRate_type1f 360 //�`�Wsensing frequency
#define SensingRate_type2f 480
#define SensingRate_type3f 600
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
	int id, x, y, CH, region, type;  //
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
int R1_S_num = 0, R2_S_num = 0, R3_S_num = 0, R4_S_num = 0;
int R2_start_index = 0, R3_start_index = 0, R4_start_index = 0;
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
        node[n].type = rand() % 3 + 1; //sensing rate
		node[n].energy = MAX_energy;
		node[n].dist_to_sink = distance(n, SINKID);  //�Z����sink
		if( node[n].x <= 200 && node[n].y <= 200 ){  //(x,y) = (1~200, 1~200)
			node[n].region = 3;
			R3_cluster.push_back(node[n]);
			R3_S_num++;
        }
        else if( node[n].x > 200 && node[n].y <= 200 ){  //(x,y) = (201~400, 1~200)
			node[n].region = 4;
			R4_cluster.push_back(node[n]);
			R4_S_num++;
        }        
        else if( node[n].x <= 200 && node[n].y > 200 ){  //(x,y) = (1~200, 201~400)
			node[n].region = 1;
			R1_cluster.push_back(node[n]);
			R1_S_num++;
        }        
        else{			//(x,y) = (201~400, 201~400)
			node[n].region = 2;
			R2_cluster.push_back(node[n]);
			R2_S_num++;
        }
    }
	/*��Ҧ��`�I��Jlist WSN*/
	WSN.clear();			
	WSN.insert(WSN.end(), R1_cluster.begin(), R1_cluster.end());
	WSN.insert(WSN.end(), R2_cluster.begin(), R2_cluster.end());
	WSN.insert(WSN.end(), R3_cluster.begin(), R3_cluster.end());
	WSN.insert(WSN.end(), R4_cluster.begin(), R4_cluster.end());
	
	R2_start_index = R1_S_num;
	R3_start_index = R2_start_index + R2_S_num;
	R4_start_index = R3_start_index + R3_S_num;
}

void special_node_deployed(){
   	R1_cluster.clear();
	R2_cluster.clear();
	R3_cluster.clear();
	R4_cluster.clear();
	WSN.clear();
    int R1 = S_NUM * 0.1;
    int R2 = S_NUM * 0.4;
    int R3 = S_NUM * 0.2;
    int R4 = S_NUM * 0.3;

	int n = 0;
    for( n ; n < R1 ; n++){
        node[n].id = n;
        node[n].x = rand() % 200 + 1;  //�`�Ix�y��1~400�H����
        node[n].y = rand() % 400 + 201;  //�`�Iy�y��1~400�H����
        node[n].CH = n;
        node[n].type = rand() % 2 + 2; //sensing rate 2or3
        node[n].energy = MAX_energy;
        node[n].dist_to_sink = distance(n, SINKID);  //�Z����sink
		node[n].region = 1;
		R1_cluster.push_back(node[n]);
        R1_S_num++;
    }
    for( n ; n < R1+R2 ; n++){
        node[n].id = n;
        node[n].x = rand() % 400 + 201;  //�`�Ix�y��1~400�H����
        node[n].y = rand() % 200 + 201;  //�`�Iy�y��1~400�H����
        node[n].CH = n;
        node[n].type = 1; //sensing rate 1
        node[n].energy = MAX_energy;
        node[n].dist_to_sink = distance(n, SINKID);  //�Z����sink
		node[n].region = 2;
		R2_cluster.push_back(node[n]);
		R2_S_num++;
    }
    for( n ; n < R1+R2+R3 ; n++){
        node[n].id = n;
        node[n].x = rand() % 200 + 1;  //�`�Ix�y��1~400�H����
        node[n].y = rand() % 200 + 1;  //�`�Iy�y��1~400�H����
        node[n].CH = n;
        node[n].type = rand() % 2 + 1; //sensing rate 1or2
        node[n].energy = MAX_energy;
        node[n].dist_to_sink = distance(n, SINKID);  //�Z����sink
		node[n].region = 3;
		R3_cluster.push_back(node[n]);
        R3_S_num++;
    }
    for( n ; n < S_NUM; n++){
        node[n].id = n;
        node[n].x = rand() % 400 + 201;  //�`�Ix�y��1~400�H����
        node[n].y = rand() % 200 + 1;  //�`�Iy�y��1~400�H����
        node[n].CH = n;
        node[n].type = 2; //sensing rate
        node[n].energy = MAX_energy;
        node[n].dist_to_sink = distance(n, SINKID);  //�Z����sink
		node[n].region = 1;
		R4_cluster.push_back(node[n]);
        R4_S_num++;
    }
    /*��Ҧ��`�I��Jlist WSN*/
	WSN.clear();			
	WSN.insert(WSN.end(), R1_cluster.begin(), R1_cluster.end());
	WSN.insert(WSN.end(), R2_cluster.begin(), R2_cluster.end());
	WSN.insert(WSN.end(), R3_cluster.begin(), R3_cluster.end());
	WSN.insert(WSN.end(), R4_cluster.begin(), R4_cluster.end());
	
	R2_start_index = R1_S_num;
	R3_start_index = R2_start_index + R2_S_num;
	R4_start_index = R3_start_index + R3_S_num;
}

void packet_init(list<Node>& wsn){
	for(auto& node : wsn){
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
	sink.id = SINKID;
	for (int i = 0; i < SINK_BUFFER_SIZE; i++){
		sink.buffer[i].data = -1;
		sink.buffer[i].dst = -1;
		sink.buffer[i].src = -1;
		sink.buffer[i].time = -1;
	}
}

double find_max_energy(const list<Node>& WSN, int start_index, int end_index) {
    double cluster_max_energy = 0;
    auto it = next(WSN.begin(), start_index);
    auto end_it = next(WSN.begin(), end_index + 1);
    for (; it != end_it; ++it) {
        if (cluster_max_energy < it->energy) {
            cluster_max_energy = it->energy;
        }
    }
    return cluster_max_energy;
}

int Find_Index(list<Node>& WSN, int nodeID){
	int index = 0;
    for (auto it = WSN.begin(); it != WSN.end(); ++it) {
        if (it->id == nodeID) {
            return index;
        }
        index++;
    }
    // �p�G���������� CH�A��^ -1
    return -1;
}

Node get_node(list<Node>& WSN, int index) {
    if (index >= 0 && index < WSN.size()) {
        auto it = next(WSN.begin(), index);
        return *it;
    } else {
        // �B�z���޶W�X�d�򪺱��p�A�Ҧp�ߥX���`�Ϊ�^�@�ӾA���q�{��
        return Node(); // ��^�@�ӾA���q�{��
    }
}

Node CH_Selection(list<Node>& WSN, int start_index, int end_index) {
    double cluster_max_energy = find_max_energy(WSN, start_index, end_index);
    queue<Node> CH_cdd; // cdd candidate
    Node select_CH;
    
    // �b cluster ���M��̤j�Ѿl��q���`�I�@���Կ� CH
    for (auto it = next(WSN.begin(), start_index); it != next(WSN.begin(), end_index + 1); ++it) {
        if (it->energy == cluster_max_energy) {
            CH_cdd.push(*it);
        }
    }
    
    select_CH = CH_cdd.front();
    int cdd_index = Find_Index(WSN, select_CH.id);
    CH_cdd.pop();

    // ����{�� CH �M cdd ���Ĥ@��� sink ���Z���A��ܳ̾A�X�� CH
    while (!CH_cdd.empty()) {
        int CH_index = Find_Index(WSN, select_CH.id);
        auto it_cdd = next(WSN.begin(), cdd_index);
		auto it_ch = next(WSN.begin(), CH_index);

		if (it_cdd->dist_to_sink < it_ch->dist_to_sink) {
			select_CH = *it_cdd;
		}
        CH_cdd.pop();
    }
    
    // ���� cluster �����Ҧ��`�I�� CH ����ܪ� CH
    for (auto it = next(WSN.begin(), start_index); it != next(WSN.begin(), end_index + 1); ++it) {
        it->CH = select_CH.id;
    }
	return select_CH;
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

int Find_Index(list<Node>& WSN, Node trans_node){
	int index = 0;
    for (auto it = WSN.begin(); it != WSN.end(); ++it) {
        if (it->CH == trans_node.id) {
            return index;
        }
        index++;
    }
    // �p�G���������� CH�A��^ -1
    return -1;
}

void transaction( Node trans_node, int time){
	int CH_WSN_index = Find_Index(WSN, trans_node);
	Packet_Generate(trans_node, time);
	Packet_Deliver(trans_node, WSN[CH_WSN_index]);
	Packet_Receive(WSN[CH_WSN_index]);
}

void Packet_Generate(Node node, int time){ //�`�I�ͦ��ʥ]�����
	total++;
	node.sense.src = node.id;
	node.sense.dst = node.CH;
	node.sense.time = time;
	node.energy -= ProbeEnergy;
}

void Packet_Deliver(Node node, Node CH_node){
	int drop_rate = rand() % 100 + 1;
	if( drop_rate > 10 || node.id == CH_node.id ){  //10% drop rate�άOCH�ۤv�ͦ����ʥ]��ۤv��buffer
		CH_node.receive.dst = node.sense.dst;
		CH_node.receive.src = node.sense.src;
		CH_node.receive.data = node.sense.data;
		CH_node.receive.time = node.sense.time;
	}
	else{    //�ʥ]�ᥢ
		macdrop++;
		CH_node.receive.dst = -1;
		CH_node.receive.src = -1;
		CH_node.receive.data = -1;
		CH_node.receive.time = -1;
	}
	double d_to_CH = distance(node.id, node.CH);
	if(node.id != node.CH){
		node.energy -= TransmitEnergy + d_to_CH * d_to_CH * AmplifierEnergy;
	}
}

void Packet_Receive(Node CH_node){ //buffer���F�n�ܦ�priority queue �����
	if ( CH_node.receive.src != CH_node.CH){ //���O�Ӧۦۤv���~����q
		CH_node.energy -= ReceiveEnergy;
	} //drop�ٺ�O����
	int full = 1;
	for (int a = 0; a < NODE_BUFFER1; a++){ //buffer is not full 50 for self-area-sense 50 for other CH 
		if (CH_node.buffer[a].data == -1)
		{
			CH_node.buffer[a].dst = CH_node.receive.dst;
			CH_node.buffer[a].src = CH_node.receive.src;
			CH_node.buffer[a].data = CH_node.receive.data;
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
		packet_init(WSN);
		sink_buffer_init();

		/*firts CH selection*/
		CH_Selection( 0, R1_S_num-1 );
		CH_Selection( R2_start_index, R3_start_index-1);
		CH_Selection( R3_start_index, R4_start_index-1);
		CH_Selection( R4_start_index, S_NUM-1);
		int R1_CHid = WSN[0].CH;
		int R2_CHid = WSN[R2_start_index].CH;
		int R3_CHid = WSN[R3_start_index].CH;
		int R4_CHid = WSN[R4_start_index].CH;

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

			/*��|�Ӱϰ쪺CH�`�I�bWSN�����ޭ�*/
			int CH1_index = Find_Index(WSN, R1_CHid);
			int CH2_index = Find_Index(WSN, R2_CHid);	
			int CH3_index = Find_Index(WSN, R3_CHid);
			int CH4_index = Find_Index(WSN, R4_CHid);

			int dead_node = Check_Life(WSN); //���`�I������^�Ӹ`�IID�A�S���N��^SinkID
			if( dead_node < SINKID ){  //���`�I����
				avg_time += time;
				network_die = true;
				break;
			}
			if( time % SensingRate_type1f == 0){
				for(auto& node : WSN){
					if( node.type == 1 ){
						transaction(node, time);
					}
				}
			}
			if( time % SensingRate_type2f == 0){
				for(auto& node : WSN){
					if( node.type == 2){
						transaction(node, time);
					}
				}
			}
			if( time % SensingRate_type3f == 0){
				for(auto& node : WSN){
					if( node.type == 3){
						transaction(node, time);
					}
				}
			}

				for(auto& node : R3_cluster){
					transaction(node, R3_CH, time);
				}
				for(auto& node : R4_cluster){
					transaction(node, R4_CH, time);
				}
			}
			if(time % CH_frequency == 0){
				CH_to_Sink(WSN[]);
				CH1_to_CH2(R1_CH, R2_CH, 1);
				CH1_to_CH2(R3_CH, R2_CH, 1);
				CH1_to_CH2(R4_CH, R2_CH, 1);

				countround[0]--;
				if(countround[0] == 0)
					CH_Selection(0, R1_S_num-1 );
				if(countround[1] == 0)
					CH_Selection(R2_start_index, R3_start_index-1);
				if(countround[2] == 0)
					CH_Selection(R3_start_index, R4_start_index-1 );
				if(countround[3] == 0)
					CH_Selection(R4_start_index, S_NUM-1);
			}
			time++;
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