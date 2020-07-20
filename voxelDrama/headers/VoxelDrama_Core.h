#pragma once
#include "interactions.h"
#include "collisionReaction.h"
#include "collisionSeparator.h"

#include <iostream>//debugging

class physicalWorld {
private:
	std::vector<collider*> staticColls;
	std::vector<collider*> colls;
	std::vector<dynamicProperties> staticDP;
	std::vector<dynamicProperties> DP;
	void applyGlobalForce();
public:
	double* deltaTime = nullptr;
	vec3d globalForce = vec3d(0,0,0);

	void addBody(collider* c, dynamicProperties* dp = nullptr);
	void addStaticBody(collider* c, dynamicProperties* dp = nullptr);
	collider* getBodyCol(size_t index);
	collider* getStaticBodyCol(size_t index);
	dynamicProperties* getBodyDP(size_t index);
	dynamicProperties* getStaticBodyDP(size_t index);
	size_t noBodies();
	size_t noStaticBodies();

	void update();
	void updatePositions();

};