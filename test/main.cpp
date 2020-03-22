#include <iostream>
#include <string>
//#include "octree.h"
//#include "./../src/octree.cpp"
//#include "collider.h"
#include "collisionDetection.h"
#include "profiler.h"

void printTree(ocNode<detectionResult>* t , std::string str) {
	if (t != nullptr) {

		for (unsigned char i = 0; i < 8; ++i) {
			printTree(((*t).nodes[i]),str + char(i+'0'));
		}
		std::cout << (int)t->data <<" layer = "<<str<< std::endl;;
	}
}

int main() {
	startProfiling();
	
	cuboidCollider c1(vec3d(110, 0, 0), vec3d(10, 10, 10));
	cuboidCollider c3(vec3d(0, 0, 0), vec3d(1, 1, 1));
	std::vector<collider*> cPtr;
	cPtr.push_back(&c1);
	cPtr.push_back(&c3);
	compoundCollider cComp(cPtr);
	sphereCollider c2(0.5,vec3d(0.5,0.5,0.5));
	vec3d val = calcCOM(&cComp, 5);
	std::cout <<val.x<<" , "<<val.y<<" , "<<val.z<< std::endl;
	endProfiling();
	system("pause");
	return 0;
}