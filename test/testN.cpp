#include"common.h"

// struct C{
// 	double x, y;
// };

ofstream fout("normal_result.txt");
Sink sink;
Node node[S_NUM];
list<Node> WSN, R1_cluster, R2_cluster, R3_cluster, R4_cluster;
int R1_S_num = 0, R2_S_num = 0, R3_S_num = 0, R4_S_num = 0;
int R2_start_index = 0, R3_start_index = 0, R4_start_index = 0;

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
		node[n].energy = rand() % 1000 +1 ;
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
        node[n].y = rand() % 200 + 201;  //�`�Iy�y��1~400�H����
        node[n].CH = n;
        node[n].type = rand() % 2 + 2; //sensing rate 2or3
        node[n].energy = rand() % 1000 +1 ;
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
        node[n].energy = rand() % 1000 +1 ;
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
        node[n].energy = rand() % 1000 +1 ;
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
        node[n].type = 2; //sensing rate
        node[n].energy = rand() % 1000 +1 ;
        node[n].dist_to_sink = distance(node[n].x, node[n].y, SINK_X, SINK_Y );  //�Z����sink
		node[n].region = 4;
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

Node get_node(list<Node>& WSN, int index) {
    if (index >= 0 && index < WSN.size()) {
        auto it = next(WSN.begin(), index);
        return *it;
    } else {
        // �B�z���޶W�X�d�򪺱��p�A�Ҧp�ߥX���`�Ϊ�^�@�ӾA���q�{��
        return Node(); // ��^�@�ӾA���q�{��
    }
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
	return select_CH.id;
}

void print_WSN(const list<Node>& wsn) {
	int i = 0;
    for (const auto& node : wsn) {
		fout << "[" << i << "]  " 
        << "Region: " << node.region <<", " << "CH: " << node.CH << ", " 
		<< "Node ID: " << node.id << ", "
        << "Energy: " << node.energy << ", "
        << "(X, Y): " << node.x << ", "<< node.y << endl;
		i++;
    }
}

void print_CH(Node Region_CH , int Region_S_num, int CH_index){
		fout << "�`�I�ơG"  << Region_S_num << endl;
		fout << "CH id: "  << Region_CH.id << endl;
		fout << "WSN_index: " << CH_index << endl;
		fout << "CH energy: " << Region_CH.energy << endl; 
		fout << endl;
}

int main() {
	srand((unsigned)time(NULL)); // �]�m���P���ؤl��
    for (int round = 0; round < round_number; round++){
		fout<<endl<<"--------------ROUND "<< round+1 <<"------------------"<<endl;
		round_init();

		// special_node_deployed();
		node_deployed();
		
		/*initialization*/
		packet_init(WSN);
		sink_buffer_init();


		/*firts CH selection*/
		int R1_CH = CH_Selection( WSN, 0, R1_S_num-1 );
		int R2_CH = CH_Selection( WSN, R2_start_index, R3_start_index-1);
		int R3_CH = CH_Selection( WSN, R3_start_index, R4_start_index-1);
		int R4_CH = CH_Selection( WSN, R4_start_index, S_NUM-1);

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

		fout << "[REGION 1]"  << "R1_CH: " << R1_CH << endl;
		print_CH(CH1, R1_S_num, CH1_index);
		fout << "[REGION 2]" << "R2_CH: " << R2_CH << endl;
		print_CH(CH2, R2_S_num, CH2_index);
		fout << "[REGION 3]" << "R3_CH: " << R3_CH << endl;
		print_CH(CH3, R3_S_num, CH3_index);
		fout << "[REGION 4]" << "R4_CH: " << R4_CH << endl;
		print_CH(CH4, R4_S_num, CH4_index);

		print_WSN(WSN);

	}
}
