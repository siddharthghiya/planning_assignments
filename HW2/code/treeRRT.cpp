#include "RRTheader.h"

node::node(int dof){
	numofDOFs = dof;
	angles = new double[numofDOFs];
}

// initialise by adding the start configuration to the vector list.
tree::tree(node* startQPtr, int numofDOFs_, double* map_, int x_size_, int y_size_){
	nodesPtrList.push_back(startQPtr);
	numofDOFs = numofDOFs_;
	map = map_;
	x_size = x_size_;
	y_size = y_size_;
}

// distance between two points
double tree::compute_distance(node* node1Ptr, node* node2Ptr){
	double distance = 0;
	double tempDistance;
	for (int i=0; i<numofDOFs; i++){
		tempDistance = (node2Ptr->angles[i] - node1Ptr->angles[i]);
		distance += tempDistance*tempDistance;
	}
	distance = sqrt(distance);

	return distance;
}

//compute maximum steps before goal.
int tree::max_steps(node* qNearPtr, node* qRandPtr){
	int steps;
	double distance = 0;
	distance = compute_distance	(qNearPtr, qRandPtr);
	steps = (int)(distance/(MAX_STEP_SIZE));
	return steps;
}

//find the increment direction
void tree::set_increment_direction(node* qNearPtr, node* qRandPtr, double* direction){
	double distance;
	distance = compute_distance(qNearPtr, qRandPtr);
	for (int i=0; i<numofDOFs; i++){
		direction[i] = (qRandPtr->angles[i] - qNearPtr->angles[i])/distance;
	}
}

// return the nearest configuration in the tree to the given randomly sampled configuration 
node* tree::nearest_neighbor(node* qRandPtr){
	double minDistance = 0;
	double distance;
	node* qNearPtr;
	node* temp;
	for(int i=0; i<nodesPtrList.size(); i++){
		temp = nodesPtrList[i];
		distance = compute_distance(qRandPtr, temp);
		if (minDistance){
			if (distance < minDistance){
				minDistance = distance;
				qNearPtr = temp;
			}
		}
		else{
			qNearPtr = temp;
			minDistance = distance;
		}
	}
	return qNearPtr;
}

// given a configuration, find the closest configuration already in the tree and then extend it by epsilon in the direction of qRandPtr to find qNew. Add it to the tree. if the point added to the tree is same as qRandPtr, return true, else false.
int tree::extend(node* qRandPtr, bool infinitEpsilon){
	//important declerations
	int epsilon = 0;
	double angles[numofDOFs] = {0};
	node* qNewPtr = new node(numofDOFs);
	int reached = 0;
	int steps = 0;
	double direction[numofDOFs] = {0};

	//find the nearest point in tree and set angles to qNearPtr`s angles.
	node* qNearPtr = nearest_neighbor(qRandPtr);
	for (int j=0; j<numofDOFs; j++){
		angles[j] = qNearPtr->angles[j];
	}

	//find the steps till we have to execute the extend operation.
	if (!infinitEpsilon){
		epsilon = min(max_steps(qNearPtr, qRandPtr), MAX_EPSILON);
	}
	else{
		epsilon = max_steps(qNearPtr, qRandPtr);
	}

	//set the increment direction	
	set_increment_direction(qNearPtr, qRandPtr, direction);

	//loop through epsilon steps, to check if the line connecting two configurations is valid or not.
	for (int i=0; i<epsilon; i++){
		// increment current angles by a small step size.
		for (int j=0; j<numofDOFs; j++){
			angles[j] += (MAX_STEP_SIZE)*direction[j];
		}
		
		//check if the current angles configuration is valid. If invalid, break loop.
		if (!IsValidArmConfiguration(angles, numofDOFs, map, x_size, y_size)){
			break;
		}

		steps++;
	}

	//if epsilon == max_steps(qNearPtr, qRandPtr) and if qRandPtr is a valid configuration, change the angles to the qRand's angles. 
	if ((steps == max_steps(qNearPtr, qRandPtr)) && (IsValidArmConfiguration(qRandPtr->angles, numofDOFs, map, x_size, y_size))){
		//if extend, dont create a new pointer. if connect, create a new pointer.
		if (!infinitEpsilon){
			qNewPtr = qRandPtr;
		}
		else{
			//declare qNewPtr to have the same configuration as angles and add it to the vector of points.
			for (int j=0; j<numofDOFs; j++){
				qNewPtr->angles[j] = angles[j];
			}
		}
		//reached the qRandPtr.
		reached = 2;	
	}

	else{
		//if the last configuration is invalid, go to the last valid configuration.
		if (!IsValidArmConfiguration(angles, numofDOFs, map, x_size, y_size)){
			for (int j=0; j<numofDOFs; j++){
				angles[j] -= (MAX_STEP_SIZE)*direction[j];
			}
			//trapped.
			reached = 1;
		}

		else{
			//took epsilon steps in direction of qRandPtr.
			reached = 0;
		}

		//declare qNewPtr to have the same configuration as angles and add it to the vector of points.
		for (int j=0; j<numofDOFs; j++){
			qNewPtr->angles[j] = angles[j];
		}
	}

	//only add if we make som eprogress towards qRandPtr.
	if (steps != 0){
		qNewPtr->parent = qNearPtr;
		qNewPtr->t = qNearPtr->t + 1;
		qNewPtr->cost = qNearPtr->cost + compute_distance(qNewPtr, qNearPtr);

		nodesPtrList.push_back(qNewPtr);
	}

	return reached;
}

//for the last added configuration to the tree, repair the graph.
void tree::repair(){
	//important intialisations
	node* qLastPtr = nodesPtrList.back();
	int distance = 0;
	node* temp;
	node* qNearestPtr;
	int epsilon;
	double direction[numofDOFs] = {0};
	int steps = 0;
	double angles[numofDOFs] = {0};

	//clear last nearPtrList.
	nearPtrList.clear();

	//set the list of nearest configurations in nearPtrList. Also, if we find a node within the nearRadius, from which reaching qLastPtr is of smaller cost, set it as the parent of qLastPtr.
	for(int i=0; i<(nodesPtrList.size()-1); i++){
		temp = nodesPtrList[i];
		distance = compute_distance(qLastPtr, temp);
		if (distance<NEAR_RADIUS){
			nearPtrList.push_back(temp);
			
			//check if cost(temp)+distance(temp, qLastPtr) < qLastPtr->cost. If true, set temp as qNearestPtr if the path from temp to qNearestPtr is collision free.
			if ((temp->cost + distance) < qLastPtr->cost){
				//collision check.

				//set angles to temps angles.
				for (int j=0; j<numofDOFs; j++){
					angles[j] = temp->angles[j];
				}

				//set epsilon
				epsilon = max_steps(qLastPtr, temp);

				//set the direction.
				set_increment_direction(temp, qLastPtr, direction);

				steps = 0;
				//loop through epsilon steps, to check if the line connecting two configurations is valid or not.
				for (int i=0; i<epsilon; i++){
					// increment current angles by a small step size.
					for (int j=0; j<numofDOFs; j++){
						angles[j] += (MAX_STEP_SIZE)*direction[j];
					}
					
					//check if the current angles configuration is valid. If invalid, break loop.
					if (!IsValidArmConfiguration(angles, numofDOFs, map, x_size, y_size)){
						break;
					}

					steps++;
				}

				if (steps == epsilon){
					qNearestPtr = temp;
					qLastPtr->cost = qNearestPtr->cost + distance;
					qLastPtr->t = qNearestPtr->t + 1;
					qLastPtr->parent = qNearestPtr;
				}
			}
		}
	}

	//go through all the nearest configurations. skip if we come across qNearestPtr. If the nearest neighbor have smaller cost through qNearPtr, set qNearPtr as it's parent.
	for(int i=0; i<nearPtrList.size(); i++){
		temp = nodesPtrList[i];
		if (temp == qNearestPtr){
			continue;
		}
		distance = compute_distance(qLastPtr, temp);
		if ((qLastPtr->cost + distance < temp->cost)){
			//collision check.
			//set angles to temps angles.
			for (int j=0; j<numofDOFs; j++){
				angles[j] = qLastPtr->angles[j];
			}

			//set epsilon
			epsilon = max_steps(qLastPtr, temp);

			//set the direction.
			set_increment_direction(qLastPtr, temp, direction);

			steps = 0;
			//loop through epsilon steps, to check if the line connecting two configurations is valid or not.
			for (int i=0; i<epsilon; i++){
				// increment current angles by a small step size.
				for (int j=0; j<numofDOFs; j++){
					angles[j] += (MAX_STEP_SIZE)*direction[j];
				}
				
				//check if the current angles configuration is valid. If invalid, break loop.
				if (!IsValidArmConfiguration(angles, numofDOFs, map, x_size, y_size)){
					break;
				}

				steps++;
			}

			if(steps == epsilon){
				temp->cost = qLastPtr->cost + distance;
				temp->t = qLastPtr->t + 1;
				temp->parent = qLastPtr;
			}
		}
	}
}

//randomly sample a configuration. with p_g, sample goal configuration and with probability (1-p_g) sample a random configuration.
node* sampleNode(int numofDOFs){
	node* temp = new node(numofDOFs);
	for(int i=0; i<numofDOFs; i++){
		temp->angles[i] = 2*PI*((double) rand() / (RAND_MAX));
	}
	return temp;
}