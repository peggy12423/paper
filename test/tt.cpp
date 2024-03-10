#include <iostream>

using namespace std;
int add(int a, int b) {
    return a + b;
}

int main(){
    int i=1, j=2;
    int sum = add(i, i+j+j);
    cout<<"SUM: "<<sum;
    return 0;
}