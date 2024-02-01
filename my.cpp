#include "common.h"

/*�ܰʹ���ѼƳ]�w*/
#define S_NUM 1000 //�P�����`��
#define R 0.25 //���Y�v �]1�h�S�����Y
#define type3f 360 //�`�Wsensing frequency
#define type4f 480
#define type5f 600
#define CHf 120 //CH trans frequency
#define freq_change_switch 1 //0�� 1�} �O�_�n�ϸ�ƶq��M�ɼW���}��
#define b_t 10800 //�jT �C�h�֬�}�@�� �pT �C�@���}�h�֬�
#define s_t 1800
#define bomb_f3 45 //�z��sensing frequency
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
    for(int n = 0 ; n < S_NUM ; n++ ){
        node[n].id = node;
		node[n].energy = MAX_energy;
		node[n].visited = 0;
		node[n].non = 0;
        node[n].type = rand() % 3 + 3;//3 4 5
		node[n].x = rand() % 400 + 1;
		node[n].y = rand() % 400 + 1;
        if( node[n].x <= 200 && node[n].y <= 200 ){
            node[n].region1 = 1;
        }
        else if( node[n].x > 200 && node[n].y <= 200 ){
            node[n].region1 = 2;
        }        
        else if( node[n].x <= 200 && node[n].y > 200 ){
            node[n].region1 = 3;
        }        
        else{
            node[n].region1 = 4;
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
    for( int rn = 0 ; rn < roundnumber ; rn++){
        node_deployed();
        packet_init();
        neighbor_init();
    }
}