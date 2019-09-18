#include <iostream>
#include <math.h>
using namespace std; 
#include <vector>

class node{
public:
    int age;
    };

int main(){
	vector<vector<node*>> arr(20, vector<node*>(20));
	arr[10][10] = new node();
	for(int i = 0; i<20; i++){
		for (int j = 0; j<20; j++){
			if (arr[i][j]){
				cout << arr[i][j] << endl;
			}
		}
	}



	return 0;
}
	