#include <math.h>
#include <mex.h>
#include <iostream>
using namespace std;
#include <queue>
#include <vector>
#include <unordered_map>
#include <random>


#define PI 3.141592654
#define NEAR_RADIUS 1
#define MAX_ITERATIONS 100000
#define K 5
#define STEP_SIZE PI/180
#define DOF 5

//borrowed from https://codereview.stackexchange.com/questions/171999/specializing-stdhash-for-stdarray
template<class T, size_t N> 
struct std::hash<std::array<T, N>> {
    auto operator() (const std::array<T, N>& key) const -> std::size_t
    {
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

struct CompareG{
    bool operator()(const node* lhs, const node* rhs) const
    {
        return (lhs->cost) > (rhs->cost);
    }
};

class graph{
public:
	vector<node*> nearestNeighbors;
	unordered_map<array<double,DOF>, node*>  nodesPtrMap; //an unordered map to map from array of configuration to the pointer of that particular configuration.
	int numofDOFs;
	double* map;
	int x_size;
	int y_size;
	graph(int numofDOFs_, double* map_, int x_size_, int y_size_);
	void local_connect(node* qPtr);
	void add_node(node* qNewPtr);
	void add_edges(node* qNewPtr);
	double compute_distance(node* q1Ptr, node* q2Ptr);
	int collision_free(node* q1Ptr, node* q2Ptr);
	void astar_path(node* q1Ptr, node* q2Ptr);
};

int IsValidArmConfiguration(double* angles, int numofDOFs, double*	map, int x_size, int y_size);
node* sampleValidNode(int numofDOFs, double*	map, int x_size, int y_size);
void plannerPRM(double* map, int x_size, int y_size, double* armstart_anglesV_rad, double* armgoal_anglesV_rad, int numofDOFs, double*** plan, int* planlength);

