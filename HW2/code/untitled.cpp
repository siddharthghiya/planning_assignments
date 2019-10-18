#include <math.h>
#include <iostream>
using namespace std;
#include <queue>
#include <vector>
#include <unordered_map>
#include <random>

template<class T, size_t N> 
struct std::hash<std::array<T, N>> {
    auto operator() (const std::array<T, N>& key) const {
        std::hash<T> hasher;
        size_t result = 0;   // I would still seed this.
        for(size_t i = 0; i < N; ++i) {
            result = (result << 1) ^ hasher(key[i]); // ??
        }
        return result;
    }
};


class node{
public:
	int numofDOFs;
	double* angles;
	node* parent;
	int t = 0;
	node(int dof);
	double cost = 0;
	vector<node*> neighbors;
};

int main(){
	node* what;
	unordered_map<array<double,3>, node*>  test;
	array<double, 3> test_key;
	test_key[0] = 0;
	test_key[1] = 1;
	test_key[2] = 2;
	// cout << test_key[1] << endl;
	cout << test.size() << endl;
	test[test_key] = what;
	cout << test.size() << endl;
	return 0;
}