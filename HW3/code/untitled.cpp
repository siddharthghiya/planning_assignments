// C++ program to distinct permutations of the string 
#include <bits/stdc++.h> 
#include <list>
using namespace std; 
 

void generate_permutations(){
	
}

// Driver code 
int main() 
{ 
    unordered_map<string, string> randommap;
    randommap["a"] = "A";
    randommap["b"] = "B";
    randommap["c"] = "C";

    unordered_map<string, string> hula = randommap;
    cout << hula["a"] << endl;
    cout << hula["b"] << endl;
} 