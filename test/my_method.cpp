#include"common.h"

/*變動實驗參數設定*/
#define S_NUM 900 //感測器總數
#define compression_rate 0.25 //壓縮率 設1則沒有壓縮
#define CH_transmit 120 //CH trans frequency
#define sensing_frequency_type1 360
#define sensing_frequency_type2 480
#define sensing_frequency_type3 720

struct C{
	double x, y;
};

struct Node
{
	int id, x, y, CH, region;  //
    double energy = 0.0;//node information
	double dist_to_sink = 0.0;//dist = distance to sink
	Package receive;
	Package sense;
	Package buffer[NODE_BUFFER2];//buffer in sensor node
};

ofstream fout("NRCA_output.txt");
Node node[S_NUM];
Sink sink;
list<Node> R1_cluster, R2_cluster, R3_cluster, R4_cluster;
list<Node> WSN;

double distance(int a, int b){   //節點a傳到b的距離
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
		/* 檢查該節點是否已經被放入了某個區域中*/
        bool nodeAlreadyInCluster = false;
        for (const auto& wsn : WSN) {
            if (wsn.id == n) {
                nodeAlreadyInCluster = true;
                break;
            }
        }
        if (nodeAlreadyInCluster) { // 如果節點已經被放入了某個區域中，則跳過本次迴圈
            continue; 
        }

        node[n].id = n;
		node[n].x = rand() % 400 + 1;  //節點x座標1~400隨機值
		node[n].y = rand() % 400 + 1;  //節點y座標1~400隨機值
		node[n].CH = n;
        node[n].type = rand() % 3 + 1; //sensing rate
		node[n].energy = MAX_energy;
		node[n].dist_to_sink = distance(n, SINKID);  //距離區sink
		if( node[n].x <= 200 && node[n].y <= 200 ){  //(x,y) = (1~200, 1~200)
			node[n].region = 3;
			R3_cluster.push_back(node[n]);
			WSN.push_back(node[n]);
        }
        else if( node[n].x > 200 && node[n].y <= 200 ){  //(x,y) = (201~400, 1~200)
			node[n].region = 4;
			R4_cluster.push_back(node[n]);
			WSN.push_back(node[n]);
        }        
        else if( node[n].x <= 200 && node[n].y > 200 ){  //(x,y) = (1~200, 201~400)
			node[n].region = 1;
			R1_cluster.push_back(node[n]);
			WSN.push_back(node[n]);
        }        
        else{			//(x,y) = (201~400, 201~400)
			node[n].region = 2;
			R2_cluster.push_back(node[n]);
			WSN.push_back(node[n]);
        }
    }
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

    bool nodeAlreadyInCluster = false;
    for (const auto& wsn : WSN) {
        if (wsn.id == n) {
            nodeAlreadyInCluster = true;
            break;
        }
    }
    if (nodeAlreadyInCluster) { // 如果節點已經被放入了某個區域中，則跳過本次迴圈
        continue; 
    }
    for( int n = 0 ; n < R1 ; n++){
        node[n].id = n;
        node[n].x = rand() % 200 + 1;  //節點x座標1~400隨機值
        node[n].y = rand() % 400 + 201;  //節點y座標1~400隨機值
        node[n].CH = n;
        node[n].type = rand() % 2 + 2; //sensing rate 2or3
        node[n].energy = MAX_energy;
        node[n].dist_to_sink = distance(n, SINKID);  //距離區sink
		node[n].region = 1;
		R1_cluster.push_back(node[n]);
		WSN.push_back(node[n]);
    }
    for( int n = 0; n < R2 ; n++){
        node[n].id = n;
        node[n].x = rand() % 400 + 201;  //節點x座標1~400隨機值
        node[n].y = rand() % 200 + 201;  //節點y座標1~400隨機值
        node[n].CH = n;
        node[n].type = 1; //sensing rate 1
        node[n].energy = MAX_energy;
        node[n].dist_to_sink = distance(n, SINKID);  //距離區sink
		node[n].region = 2;
		R2_cluster.push_back(node[n]);
		WSN.push_back(node[n]);
    }
    for( int n = 0 ; n < R3 ; n++){
        node[n].id = n;
        node[n].x = rand() % 200 + 1;  //節點x座標1~400隨機值
        node[n].y = rand() % 200 + 1;  //節點y座標1~400隨機值
        node[n].CH = n;
        node[n].type = rand() % 2 + 1; //sensing rate 1or2
        node[n].energy = MAX_energy;
        node[n].dist_to_sink = distance(n, SINKID);  //距離區sink
		node[n].region = 3;
		R3_cluster.push_back(node[n]);
		WSN.push_back(node[n]);
    }
    for( int n = 0; n < R4; n++){
        node[n].id = n;
        node[n].x = rand() % 400 + 201;  //節點x座標1~400隨機值
        node[n].y = rand() % 200 + 1;  //節點y座標1~400隨機值
        node[n].CH = n;
        node[n].type = 2; //sensing rate
        node[n].energy = MAX_energy;
        node[n].dist_to_sink = distance(n, SINKID);  //距離區sink
		node[n].region = 1;
		R4_cluster.push_back(node[n]);
		WSN.push_back(node[n]);
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

double avg_energy(list<Node>& cluster, int node_num){
    double total_energy = 0.0;
    for(auto& nd : cluster){
        total_energy += nd.energy;
    }
    double avg_energy = total / node_num;
    return avg_energy;
}

double node_density(list<Node>& cluster){
    int node_count = 0;
    for(auto& nd : cluster){
        node_count++;
    }
    double node_density = node_count / 40000;
    return node_density;
}

int main(){
    srand((unsigned)time(NULL));
    for(int round = 0; round < round_number; round++){
        node_deployed();
        // special_node_deployed();

    }
}