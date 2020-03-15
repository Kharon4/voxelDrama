#pragma once

#include "math/vec3.h"

enum intersectionType { outside = 0,overlap = 1, inside = 2 };

class collider {
public:
	virtual void boundingBox(vec3d& lower, vec3d& upper) {};
	virtual intersectionType inside(vec3d pt) { return intersectionType::outside; };
};


class sphereCollider : public collider {
public:
	vec3d center;
	double rad;
	void boundingBox(vec3d& lower, vec3d& upper);
	intersectionType inside(vec3d pt);
};