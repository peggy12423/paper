#include"common.h"

#define S_NUM 100 //感測器總數

struct Node
{
	int id, x, y, CH, type, region1, RegionID;  //RegionID用於紀錄該節點在這個region內的ID
	double energy;//node information
	Package receive;
	Package sense;
	Package buffer[NODE_BUFFER2];//buffer in sensor node
	double dtc;//dist = distance to sink
};
Node node[S_NUM];
Sink sink;
list<Node> R1_cluster, R2_cluster, R3_cluster, R4_cluster;

double distance(int a, int b){
	if (b != SINKID){
		return sqrt(pow(abs(node[a].x - node[b].x), 2) + pow(abs(node[a].y - node[b].y), 2));
	}
	else{ //SINK
		return sqrt(pow(abs(node[a].x - SINK_X), 2) + pow(abs(node[a].y - SINK_Y), 2));
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
		// node[n].energy = MAX_energy;
		node[n].energy = rand() % 400+1;

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

double find_max_energy(const list<Node>& Region_cluster){     //!energy的預扣
	double cluster_max_energy = Region_cluster.front().energy;
	for(const auto& node : Region_cluster){
        if(node.energy > cluster_max_energy){
            cluster_max_energy = node.energy;
        }
	}
	return cluster_max_energy;
}

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

void printCluster(const list<Node>& cluster) {
    for (const auto& node : cluster) {
        cout << "Node ID: " << node.id << ", "
                  << "Energy: " << node.energy << ", "
                  << "(X, Y): " << node.x << ", "<< node.y << endl;
    }
}

int main() {
	srand((unsigned)time(NULL)); // 設置不同的種子值
    for (int round = 0; round < round_number; round++){
		cout<<endl<<"--------------ROUND "<< round+1 <<"------------------"<<endl;
		node_deployed();
		cout<<"[REGION 1]"<<endl;
		printCluster(R1_cluster);
		cout<<"[REGION 2]"<<endl;
		printCluster(R2_cluster);
		cout<<"[REGION 3]"<<endl;
		printCluster(R3_cluster);
		cout<<"[REGION 4]"<<endl;
		printCluster(R4_cluster);

		packet_init();

		/*sink initialization*/
		sink.id = SINKID;
		sink_buffer_init(SINK_BUFFER_SIZE);

		/*firts CH selection*/
		Node R1_CH = CH_Selection(R1_cluster);

		cout << "[CHs in R1]" << endl;
		cout << "Node ID: " << R1_CH.id << ", energy: " << R1_CH.energy << endl;

		Node R2_CH = CH_Selection(R2_cluster);
		cout << "[CHs in R2]" << endl;
		cout << "Node ID: " << R2_CH.id << ", energy: " << R2_CH.energy << endl;

		Node R3_CH = CH_Selection(R3_cluster);
		cout << "[CHs in R3]" << endl;
		cout << "Node ID: " << R3_CH.id << ", energy: " << R3_CH.energy << endl;
		
		Node R4_CH = CH_Selection(R4_cluster);
		cout << "[CHs in R4]" << endl;
		cout << "Node ID: " << R4_CH.id << ", energy: " << R4_CH.energy << endl;

	}
}
