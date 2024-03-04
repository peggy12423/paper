#include "common.h"

/*變動實驗參數設定*/
#define S_NUM 1000 //感測器總數
#define R 0.25 //壓縮率 設1則沒有壓縮
#define type3f 360 //常規sensing frequency
#define type4f 480
#define type5f 600
#define CHf 120 //CH trans frequency
#define freq_change_switch 1 //0關 1開 是否要使資料量突然暴增的開關
#define b_t 10800 //大T 每多少秒開一次 小T 每一次開多少秒
#define s_t 1800
#define bomb_f3 45 //爆炸sensing frequency
#define bomb_f4 60
#define bomb_f5 90
/**/

Node node[S_NUM];
Sink sink;
sink.id = SINKID;
int R2 = S_NUM / 4;
int R3 = S_NUM / 2;
int R4 = S_NUM * 0.75;

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


void node_density(){
    
}

void packet_init(){
	for (int a = 0; a < S_NUM; a++){
		node[a].receive.data = -1;
		node[a].receive.dst = -1;
		node[a].receive.src = -1;
		node[a].receive.time = -1;
		node[a].sense.data = -1;
		node[a].sense.dst = -1;
		node[a].sense.src = -1;
		node[a].sense.time = -1;
		for (int b = 0; b < 100; b++){
			node[a].buffer[b].data = -1;
			node[a].buffer[b].dst = -1;
			node[a].buffer[b].src = -1;
			node[a].buffer[b].time = -1;
		}
	}
}
void neighbor_init(){
	for (int i = 0; i < S_NUM; i++){
		for (int j = 0; j < S_NUM; j++){
			if (i != j && distance(i, j) <= trans_dis){
				node[i].neighbor[node[i].non] = j;
				node[i].non++;
			}
		}
	}
}
int main(){
    for( int rn = 0 ; rn < round_number ; rn++){
        node_deployed();
        packet_init();
        neighbor_init();
    }
}