#pragma once

#include"octree.h"
#include"collider.h"


#define defaultLayers 10

enum class detectionResult{
	NFD = 0 ,//[NFD]needs further division
	nonColliding = 1 ,
	colliding = 2,
	nonCollidingNFD = 3 ,
	collidingNFD = 4 
};

ocNode<detectionResult>* createCollisionTree(collider* c1, collider* c2, unsigned char layers = defaultLayers);
