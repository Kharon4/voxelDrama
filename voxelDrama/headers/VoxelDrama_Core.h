#pragma once
#include "interactions.h"
#include "collisionReaction.h"
#include "collisionSeparator.h"

class physicalWorld {
private:
	std::vector<collider*> staticColls;
	std::vector<collider*>colls;
	std::vector<dynamicProperties> staticDP;
	std::vector<dynamicProperties> DP;

public:
	double* deltaTime = nullptr;

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