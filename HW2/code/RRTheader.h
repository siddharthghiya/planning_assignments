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

#define P_G 0.1
#define MAX_EPSILON 4
#define PI 3.141592654
#define MAX_STEP_SIZE PI/20
#define NEAR_RADIUS PI/180

class node{
public:
	int numofDOFs;
	double* angles;
	node* parent;
	int t = 0;
	node(int dof);
	double cost = 0;
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
	node* sampleNode(int numofDOFs);
};

node* sampleNode(int numofDOFs);
int IsValidArmConfiguration(double* angles, int numofDOFs, double*	map, int x_size, int y_size);
void plannerRRT(double*	map, int x_size, int y_size, double* armstart_anglesV_rad, double* armgoal_anglesV_rad, int numofDOFs, double*** plan, int* planlength);
void plannerRRTConnect(double* map, int x_size, int y_size, double* armstart_anglesV_rad, double* armgoal_anglesV_rad, int numofDOFs, double*** plan, int* planlength);
void plannerRRTStar(double* map, int x_size, int y_size, double* armstart_anglesV_rad, double* armgoal_anglesV_rad, int numofDOFs, double*** plan, int* planlength);