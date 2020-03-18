#include <iostream>
#include "collider.h"

#include "profiler.h"


int main() {

	startProfiling();
	{
		profileFunc();
	sphereCollider sc;
	sc.center = vec3d(4, -5, 6);
	sc.rad = 10;

	vec3d lower, upper;
	sc.boundingBox(lower, upper);

	std::cout << lower.x << " , " << lower.y << " , " << lower.z << std::endl;
	std::cout << upper.x << " , " << upper.y << " , " << upper.z << std::endl;

	std::cout << "hello world 0" << std::endl;
	}
	endProfiling();
	int x;
	std::cin >> x;


	return 0;
}