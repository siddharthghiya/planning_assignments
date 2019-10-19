// #ifndef RRTheader_H
// #define RRTheader_H
#include <math.h>
#include <mex.h>
#include <iostream>
using namespace std;
#include <queue>
#include <vector>
#include <unordered_map>
#include <random>

#define PI 3.141592654
#define P_G 0.2
#define MAX_EPSILON 10
#define MAX_STEP_SIZE PI/90
#define NEAR_RADIUS PI/3
#define MAX_ITERATIONS_RRT 500000
#define MAX_ITERATIONS_PRM 30000
#define DOF 5
#define K 5

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
	int iden;
};

struct CompareG{
    bool operator()(const node* lhs, const node* rhs) const
    {
        return (lhs->cost) > (rhs->cost);
    }
};

class tree{
public:
	vector<node*> nodesPtrList;
	vector<node*> nearPtrList; //for RRTStar, to get a list of nearby configurations.
	int numofDOFs;
	double* map;
	int x_size;
	int y_size;

	tree(node* startQPtr, int numofDOFs_, double* map_, int x_size_, int y_size_);
	double compute_distance(node* node1Ptr, node* node2Ptr);
	int max_steps(node* qNearPtr, node* qRandPtr);
	void set_increment_direction(node* qNearPtr, node* qRandPtr, double* direction);
	node* nearest_neighbor(node* qRandPtr);
	int extend(node* qRandPtr, bool infinitEpsilon);
	void repair();
	// node* sampleNode(int numofDOFs);
};

class graph{
public:
	vector<node*> nodesPtrList; //an unordered map to map from array of configuration to the pointer of that particular configuration.
	int numofDOFs;
	double* map;
	int x_size;
	int y_size;

	graph(int numofDOFs_, double* map_, int x_size_, int y_size_);
	bool local_connect(node* qPtr);
	void add_node(node* qNewPtr);
	void add_edges(node* qNewPtr);
	double compute_distance(node* q1Ptr, node* q2Ptr);
	int collision_free(node* q1Ptr, node* q2Ptr);
	bool astar_path(node* q1Ptr, node* q2Ptr);
};

node* sampleNode(int numofDOFs);
node* sampleValidNode(int numofDOFs, double*	map, int x_size, int y_size);
int IsValidArmConfiguration(double* angles, int numofDOFs, double*	map, int x_size, int y_size);
void plannerRRT(double*	map, int x_size, int y_size, double* armstart_anglesV_rad, double* armgoal_anglesV_rad, int numofDOFs, double*** plan, int* planlength);
void plannerRRTConnect(double* map, int x_size, int y_size, double* armstart_anglesV_rad, double* armgoal_anglesV_rad, int numofDOFs, double*** plan, int* planlength);
void plannerRRTStar(double* map, int x_size, int y_size, double* armstart_anglesV_rad, double* armgoal_anglesV_rad, int numofDOFs, double*** plan, int* planlength);
void plannerPRM(double* map, int x_size, int y_size, double* armstart_anglesV_rad, double* armgoal_anglesV_rad, int numofDOFs, double*** plan, int* planlength);