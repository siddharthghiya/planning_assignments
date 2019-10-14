#include <stdio.h> 
#include <iostream>
using namespace std;

#define PI 3.141592654
#define MAX_STEP_SIZE PI/20

class node{
   node(int dof){
      int numofDOFs;
      numofDOFs = dof;
      double angles[numofDOFs];
      node* parent;  
   }
};
  
// Driver program 
int main() 
{
   double distance = 4.6;
   cout << MAX_STEP_SIZE << endl;
   cout << (int)(distance/(MAX_STEP_SIZE)) << endl;
} 