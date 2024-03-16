#include"common.h"
#include <iostream>

#define compression_rate 0.25 //���Y�v �]1�h�S�����Y
#define CH_transmit 120 //CH trans frequency
#define SensingRate_type1f 360 //�`�Wsensing frequency
#define SensingRate_type2f 480
#define SensingRate_type3f 600
#define CH_frequency 120

struct Node{
	int id, x, y, CH, region, type;  //
    double energy;//node information
	double dist_to_sink;//dist = distance to sink
	Package receive;
	Package sense;
	Package buffer[NODE_BUFFER2];//buffer in sensor node
};
/*�ܰʹ���ѼƳ]�w*/
int S_NUM = 200; //�P�����`��

double avg_time = 0;
double macdrop = 0;
double drop = 0;
double total = 0;
double received = 0;
double cccount = 0;

Node node[1000];
Sink sink;
list<Node> R1_cluster, R2_cluster, R3_cluster, R4_cluster;
int R1_S_num = 0, R2_S_num = 0, R3_S_num = 0, R4_S_num = 0;
int R2_start_index = 0, R3_start_index = 0, R4_start_index = 0;
list<Node> WSN;

void print_WSN(const list<Node>& wsn) {
	int i = 0;
    for (const auto& node : wsn) {
		cout << "[" << i << "]  " 
        << "Region: " << node.region <<", " << "CH: " << node.CH << ", " 
		<< "Node ID: " << node.id << ", "
        << "Energy: " << node.energy << ", "
        << "(X, Y): " << node.x << ", "<< node.y << endl;
		i++;
    }
}

double distance(int node1_x, int node1_y, int node2_x, int node2_y){
	return sqrt(pow(abs(node1_x - node2_x), 2) + pow(abs(node1_y - node2_y), 2));
}

void round_init(){
	WSN.clear();
	R1_cluster.clear();
	R2_cluster.clear();
	R3_cluster.clear();
	R4_cluster.clear();
	R1_S_num = 0;
	R2_S_num = 0;
	R3_S_num = 0;
	R4_S_num = 0;
}

void node_deployed(){
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
		node[n].energy = MAX_energy ;
		node[n].dist_to_sink = distance(node[n].x, node[n].y, SINK_X, SINK_Y );  //�Z����sink
        if( node[n].x <= 200 && node[n].y > 200 ){  //(x,y) = (1~200, 201~400)
			node[n].region = 1;
			R1_cluster.push_back(node[n]);
			R1_S_num++;
        }
		else if( node[n].x > 200 && node[n].y > 200){			//(x,y) = (201~400, 201~400)
			node[n].region = 2;
			R2_cluster.push_back(node[n]);
			R2_S_num++;
        }        
		else if( node[n].x <= 200 && node[n].y <= 200 ){  //(x,y) = (1~200, 1~200)
			node[n].region = 3;
			R3_cluster.push_back(node[n]);
			R3_S_num++;
        }
        else{  //(x,y) = (201~400, 1~200)
			node[n].region = 4;
			R4_cluster.push_back(node[n]);
			R4_S_num++;
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
   	int R1 = S_NUM * 0.1;
    int R2 = S_NUM * 0.4;
    int R3 = S_NUM * 0.2;
    int R4 = S_NUM * 0.3;

	int n = 0;
    for( n ; n < R1 ; n++){
        node[n].id = n;
        node[n].x = rand() % 200 + 1;  //�`�Ix�y��1~400�H����
        node[n].y = rand() % 200 + 201;  //�`�Iy�y��1~400�H����
        node[n].CH = n;
        node[n].type = rand() % 2 + 2; //sensing rate 2or3
        node[n].energy = MAX_energy ;
        node[n].dist_to_sink = distance(node[n].x, node[n].y, SINK_X, SINK_Y );  //�Z����sink
		node[n].region = 1;
		R1_cluster.push_back(node[n]);
        R1_S_num++;
    }
    for( n ; n < R1+R2 ; n++){
        node[n].id = n;
        node[n].x = rand() % 200 + 201;  //�`�Ix�y��1~400�H����
        node[n].y = rand() % 200 + 201;  //�`�Iy�y��1~400�H����
        node[n].CH = n;
        node[n].type = 1; //sensing rate 1
        node[n].energy = MAX_energy ;
        node[n].dist_to_sink = distance(node[n].x, node[n].y, SINK_X, SINK_Y );  //�Z����sink
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
        node[n].energy = MAX_energy ;
        node[n].dist_to_sink = distance(node[n].x, node[n].y, SINK_X, SINK_Y );  //�Z����sink
		node[n].region = 3;
		R3_cluster.push_back(node[n]);
        R3_S_num++;
    }
    for( n ; n < S_NUM; n++){
        node[n].id = n;
        node[n].x = rand() % 200 + 201;  //�`�Ix�y��1~400�H����
        node[n].y = rand() % 200 + 1;  //�`�Iy�y��1~400�H����
        node[n].CH = n;
        node[n].type = rand() % 3 + 1; //sensing rate
        node[n].energy = MAX_energy ;
        node[n].dist_to_sink = distance(node[n].x, node[n].y, SINK_X, SINK_Y );  //�Z����sink
		node[n].region = 4;
		R4_cluster.push_back(node[n]);
        R4_S_num++;
    }
    /*��Ҧ��`�I��Jlist WSN*/
	WSN.insert(WSN.end(), R1_cluster.begin(), R1_cluster.end());
	WSN.insert(WSN.end(), R2_cluster.begin(), R2_cluster.end());
	WSN.insert(WSN.end(), R3_cluster.begin(), R3_cluster.end());
	WSN.insert(WSN.end(), R4_cluster.begin(), R4_cluster.end());
	
	R2_start_index = R1_S_num;
	R3_start_index = R2_start_index + R2_S_num;
	R4_start_index = R3_start_index + R3_S_num;
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

int CH_Selection(list<Node>& WSN, int start_index, int end_index) {
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
    CH_cdd.pop();

    // ����{�� CH �M cdd ���Ĥ@��� sink ���Z���A��ܳ̾A�X�� CH
    while (!CH_cdd.empty()) {
        if (CH_cdd.front().dist_to_sink < select_CH.dist_to_sink) {
        	select_CH = CH_cdd.front();
    	}
    	CH_cdd.pop();
    }
    
    // ���� cluster �����Ҧ��`�I�� CH ����ܪ� CH
    for (auto it = next(WSN.begin(), start_index); it != next(WSN.begin(), end_index + 1); ++it) {
        it->CH = select_CH.id;
    }
	return select_CH.id;
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

void Packet_Generate(Node& node, int time){ //�`�I�ͦ��ʥ]�����
	total++;
	node.sense.src = node.id;
	node.sense.dst = node.CH;
	node.sense.data = node.type;
	node.sense.time = time;
	node.energy -= ProbeEnergy;
	// cout << "Node " << node.id << "�ͦ���� " << node.sense.data << " at " << time << "   �ǳưe��" << node.sense.dst << endl;
	// cout << "node " << node.id << "�ͦ��ʥ]�]����q��֬�" << node.energy << endl;
}

void Packet_Deliver(Node& node, Node& CH_node){
	int drop_rate = rand() % 100 + 1;
	double d = distance(node.x, node.y, CH_node.x, CH_node.y);
	CH_node.receive.dst = node.sense.dst;
	CH_node.receive.src = node.sense.src;
	CH_node.receive.data = node.sense.data;
	CH_node.receive.time = node.sense.time;
	if( drop_rate <= 10 && node.id != CH_node.id ){  //�ᥢ�ʥ]�B�ǰe�ʥ]�`�I���OCH�ۤv
		macdrop++;
		node.energy -= TransmitEnergy + d*d*AmplifierEnergy;  // //macdrop�᭫�e�Y��e�F
	}
	else if( drop_rate > 10 && node.id != CH_node.id ){    //����ʥ]�B�ǰe�ʥ]�`�I���OCH�ۤv
		node.energy -= TransmitEnergy + d*d*AmplifierEnergy;
		// cout << "node " << node.id << " �]�ǰe��CH�A��q��� " << TransmitEnergy +d*d*AmplifierEnergy << endl;
		// cout << "node " << node.id << " �ǿ�ʥ]�]����q��֬�" << node.energy << endl;
	}
}

void Packet_Receive(Node& CH_node) { //buffer���F�n�ܦ�priority queue �����
    if (CH_node.receive.src != CH_node.id) { //���O�Ӧۦۤv���~����q
        CH_node.energy -= ReceiveEnergy;
    }
    cccount++;
    int full_index = -1; // ��l�� full_index

    for (int a = 0; a < NODE_BUFFER1; a++) { // CH����ʥ]���Jbuffer[0~99]
        if (CH_node.buffer[a].data == -1) {   // �p�G buffer[a] �٬O�Ū��N�� receive �ʥ]��i�h
            CH_node.buffer[a].dst = CH_node.receive.dst;
            CH_node.buffer[a].src = CH_node.receive.src;
            CH_node.buffer[a].data = CH_node.receive.data;
            CH_node.buffer[a].time = CH_node.receive.time;
            received++;
            full_index = a;
            // cout << CH_node.receive.src << " -->" << CH_node.receive.dst << "   cccount: " << cccount << endl;
            // cout << "CH " << CH_node.id << "   BFull index : " << full_index << "    Received: " << received << endl;
            break;
        }
    }
	// cout << "buffer[99]: " << CH_node.buffer[99].data << endl;
    if (full_index == -1) { // �p�G�Ҧ��� buffer[a] �����O�Ū��Afull_index ���M����l�� -1
        // cout << CH_node.receive.src << "--->" << CH_node.id<<endl;
		drop++;
        // cout << "-----buffer[99]: "<< CH_node.buffer[99].data << endl;
        // cout << "cccount: " << cccount <<  "      ##############DROP###########   Drop: " << drop <<endl;
    } else {
        // �p�G���\��J�ʥ]�A�h������ƪ�����A�]���ڭ̤w�g�����F�ʥ]���B�z
        return;
    }

    CH_node.receive.dst = -1;
    CH_node.receive.src = -1;
    CH_node.receive.data = -1;
    CH_node.receive.time = -1;
}

void transaction( Node& trans_node, int time, Node& R1_ch, Node& R2_ch, Node& R3_ch, Node& R4_ch ){
	if( trans_node.CH == R1_ch.id ){
		Packet_Generate( trans_node, time );
		Packet_Deliver( trans_node, R1_ch );
		Packet_Receive( R1_ch );
	}
	else if( trans_node.CH == R2_ch.id ){
		Packet_Generate( trans_node, time );
		Packet_Deliver( trans_node, R2_ch );
		Packet_Receive( R2_ch );
	}
	else if( trans_node.CH == R3_ch.id ){
		Packet_Generate( trans_node, time );
		Packet_Deliver( trans_node, R3_ch );
		Packet_Receive( R3_ch );
	}
	else{
		Packet_Generate( trans_node, time );
		Packet_Deliver( trans_node, R4_ch );
		Packet_Receive( R4_ch );
	}
}

void clean(Node& CH_node, int start, int end){
	for (start; start < end; start++){
		CH_node.buffer[start].data = -1;
		CH_node.buffer[start].dst = -1;
		CH_node.buffer[start].src = -1;
		CH_node.buffer[start].time = -1;
	}
}

void CH_to_Sink(Node& CH_node){ //R4�Ǩ�Sink
	int start = 0;    //sink��buffer�q����}�l�O�Ū�
	for( int b = 0; b < SINK_BUFFER_SIZE; b++ ){
		if( sink.buffer[b].data == -1){
			start = b;
			break;
		}
	}
	if( CH_node.buffer[NODE_BUFFER1].data != -1 ){    //���O��CH��
		double rate = 0;/*���Y�v0.25*/
		for(int b = NODE_BUFFER1; b < NODE_BUFFER2; b++){
			if( CH_node.buffer[b].data == -1 ){  //���Ū��N�����~��F
				break;
			}
			rate = b - NODE_BUFFER1 + 1;
			sink.buffer[start].data = CH_node.buffer[b].data;
			sink.buffer[start].dst = CH_node.buffer[b].dst;
			sink.buffer[start].src = CH_node.buffer[b].src;
			sink.buffer[start].time = CH_node.buffer[b].time;
			// cout << "CH ID:" << CH_node.id << " src: " << sink.buffer[start].src << " dst: " << sink.buffer[start].dst << " data: " << sink.buffer[start].data << " time: " << sink.buffer[start].time << " sec" << endl;
			start++;
		}
		rate = ceil(rate * compression_rate);
		double CH_to_S = distance(CH_node.x, CH_node.y, SINK_X, SINK_Y);
		CH_node.energy -= (TransmitEnergy + CH_to_S*CH_to_S*AmplifierEnergy)*rate; //�Ndata�X�֤���@���ǰe �ҥH�ӯ�o�˺�(�X�֥���) �]���i���X�֪�size���bpacket���j�p����
		clean( CH_node, NODE_BUFFER1, NODE_BUFFER2); /*�ǧ�����R����*/
	}
	else{      //�ۤv�ϰ쪺���Y�ʥ]�Ǩ�Sink
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
			// cout << "CH ID:" << CH_node.id << " src: " << sink.buffer[start].src << " dst: " << sink.buffer[start].dst << " data: " << sink.buffer[start].data << " time: " << sink.buffer[start].time << " sec" << endl;
			start++;
		}
		rate = ceil(rate * compression_rate);
		double CH_to_S = distance(CH_node.x, CH_node.y, SINK_X, SINK_Y);
		CH_node.energy -= (TransmitEnergy + CH_to_S*CH_to_S*AmplifierEnergy)*rate; //�Ndata�X�֤���@���ǰe �ҥH�ӯ�o�˺�(�X�֥���) �]���i���X�֪�size���bpacket���j�p����
		clean( CH_node, 0, NODE_BUFFER1); /*�ǧ�����R����*/
	}
}

void CH_to_CH4(Node& CH, Node& CH4){ //���F2�ϥH�~���ϰ쳣���2�ϸ̭���q�̰��� �����
	/*��CH��2��+2�Ϩ�sink���Z���ۥ[�P��Ѿl��q�Ȱ��[�v*/
	double rate = 0;/*���Y�v0.25*/
	for (int b = 0; b < NODE_BUFFER1; b++){  //CH�Ǩ�CH4
		if ( CH.buffer[b].data == -1){ //���Ū��N�����~��F
			break;
		}
		rate = b + 1;
		CH4.buffer[NODE_BUFFER1 + b].data = CH.buffer[b].data;
		CH4.buffer[NODE_BUFFER1 + b].dst = CH.buffer[b].dst;
		CH4.buffer[NODE_BUFFER1 + b].src = CH.buffer[b].src;
		CH4.buffer[NODE_BUFFER1 + b].time = CH.buffer[b].time;
	}
	clean(CH, 0, NODE_BUFFER1);
	rate = ceil(rate * compression_rate);
	CH.energy -= (TransmitEnergy + pow(distance(CH.x, CH.y, CH4.x, CH4.y), 2)*AmplifierEnergy)*rate; //�Ndata�X�֤���@���ǰe �ҥH�ӯ�o�˺�(�X�֥���)
																						  //cout << "node : " << CH << "��q��� "<<(TransmitEnergy + pow(distance(CH.x, CH1.y, CH4.x, CH4.y), 2)*AmplifierEnergy)*rate<<", �]���ǿ鵹�ϰ�2" << endl;
	CH4.energy -= (ReceiveEnergy)*rate;
	CH_to_Sink(CH4);
}

int main(){
	ofstream fout("normal_NRCA.txt");
	streambuf *coutbuf = cout.rdbuf();
	cout.rdbuf(fout.rdbuf());
	cout << "NRCA" << endl;

	for( S_NUM ; S_NUM <= end_S_NUM ; S_NUM+=100 ){
		srand((unsigned)time(NULL)); //random seed
		for (int round = 0; round < round_number; round++){
			// cout << "----------------ROUND " << round +1 << "-----------------" <<endl;
			round_init();
			node_deployed();
			// special_node_deployed();
			
			/*initialization*/
			packet_init(WSN);
			sink_buffer_init();

			/*firts CH selection*/
			int R1_CH = CH_Selection( WSN, 0, R2_start_index-1 );
			int R2_CH = CH_Selection( WSN, R2_start_index, R3_start_index-1);
			int R3_CH = CH_Selection( WSN, R3_start_index, R4_start_index-1);
			int R4_CH = CH_Selection( WSN, R4_start_index, S_NUM-1);
			// cout << "CH: " << R1_CH << "  " << R2_CH << "  " << R3_CH << "  " << R4_CH <<endl;

			/*��|�Ӱϰ쪺CH�`�I�bWSN�����ޭ�*/
			int CH1_index = Find_Index(WSN, R1_CH);
			int CH2_index = Find_Index(WSN, R2_CH);	
			int CH3_index = Find_Index(WSN, R3_CH);	
			int CH4_index = Find_Index(WSN, R4_CH);	

			auto CH1_it = next( WSN.begin(), CH1_index);
			Node& CH1 = *CH1_it;
			auto CH2_it = next( WSN.begin(), CH2_index);
			Node& CH2 = *CH2_it;
			auto CH3_it = next( WSN.begin(), CH3_index);
			Node& CH3 = *CH3_it;
			auto CH4_it = next( WSN.begin(), CH4_index);
			Node& CH4 = *CH4_it;
			// cout << "CH confirm: " << CH1.id << "  " << CH2.id << "  " << CH3.id << "  " << CH4.id <<endl;

			int countround[4] = {0, 0, 0, 0};

			int time = 1;
			bool network_die = false;  //1�N���`�I���`�A0�N��S��

			while( !network_die ){
				if(countround[0] == 0)
					countround[0] = 10;
				if(countround[1] == 0)
					countround[1] = 10;
				if(countround[2] == 0)
					countround[2] = 10;
				if(countround[3] == 0)
					countround[3] = 3;

				int dead_node = Check_Life(WSN); //���`�I������^�Ӹ`�IID�A�S���N��^SinkID
				if( dead_node < SINKID ){  //���`�I����
					avg_time += time;
					network_die = true;
					break;
				}
				if( time % SensingRate_type1f == 0){
					for(auto& node : WSN){
						if( node.type == 1 ){
							transaction( node, time, CH1, CH2, CH3, CH4 );
						}
					}
				}
				if( time % SensingRate_type2f == 0){
					for(auto& node : WSN){
						if( node.type == 2){
							transaction( node, time, CH1, CH2, CH3, CH4 );
						}
					}
				}
				if( time % SensingRate_type3f == 0){
					for(auto& node : WSN){
						if( node.type == 3){
							transaction( node, time, CH1, CH2, CH3, CH4 );
						}
					}
				}

				if( time % CH_frequency == 0){
					CH_to_Sink(CH4);
					CH_to_CH4(CH3, CH4);
					CH_to_CH4(CH1, CH4);
					CH_to_CH4(CH2, CH4);

					countround[0]--;
					if(countround[0] == 0)
						R1_CH = CH_Selection( WSN, 0, R2_start_index-1 );
					countround[1]--;
					if(countround[1] == 0)
						R2_CH = CH_Selection( WSN, R2_start_index, R3_start_index-1);
					countround[2]--;
					if(countround[2] == 0)
						R3_CH = CH_Selection( WSN, R3_start_index, R4_start_index-1 );
					countround[3]--;
					if(countround[3] == 0)
						R4_CH = CH_Selection( WSN, R4_start_index, S_NUM-1);
					/*��|�Ӱϰ쪺CH�`�I�bWSN�����ޭ�*/
					CH1_index = Find_Index(WSN, R1_CH);
					CH2_index = Find_Index(WSN, R2_CH);	
					CH3_index = Find_Index(WSN, R3_CH);	
					CH4_index = Find_Index(WSN, R4_CH);	

					auto CH1_it = next( WSN.begin(), CH1_index);
					Node& CH1 = *CH1_it;
					auto CH2_it = next( WSN.begin(), CH2_index);
					Node& CH2 = *CH2_it;
					auto CH3_it = next( WSN.begin(), CH3_index);
					Node& CH3 = *CH3_it;
					auto CH4_it = next( WSN.begin(), CH4_index);
					Node& CH4 = *CH4_it;
				}
				time++;
			}
			// cout <<"ROUND " << round+1 << "   OVER"<< endl;
		}
		drop /= round_number;
		total /= round_number;
		received /= round_number;
		macdrop /= round_number;
		avg_time /= round_number;
		cccount /= round_number;
		cout << "------------ Sensors " << S_NUM << " ------------" << endl;
		cout << "avg_time: " << avg_time << endl;
		cout << "package total : " << total << endl;
		// cout << "cccount: " << cccount << endl;
		cout << "mac_drop : " << macdrop << endl;
		cout << "overflow_drop :" << drop << endl;
		cout << "received : " << received <<endl;
		cout << "packet loss rate : " << (macdrop+drop) / total << endl << endl;
	}
	
	cout.rdbuf(coutbuf);
	fout.close();
	return 0;
}