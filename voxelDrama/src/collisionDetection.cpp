#include "./../headers/collisionDetection.h"

#include <iostream> //for debugging

#define min(a,b) ((a<b)?a:b)
#define max(a,b) ((a>b)?a:b)

#define debugVec vec3d(-10000,-10000,-10000)

bool collidingREC(collider * c1 , collider * c2 ,vec3d pt , vec3d step , unsigned char layers) {
	if (layers == 0)return false;
	vec3d stepX(step.x*2, 0, 0);
	vec3d stepY(0, step.y*2, 0);
	vec3d stepZ(0, 0, step.z*2);
	vec3d PT = pt - step;
	if (c1->inside(PT) && c2->inside(PT))return true;
	PT += stepX;
	if (c1->inside(PT) && c2->inside(PT))return true;
	PT += stepY;
	if (c1->inside(PT) && c2->inside(PT))return true;
	PT -= stepX;
	if (c1->inside(PT) && c2->inside(PT))return true;
	PT += stepZ;
	if (c1->inside(PT) && c2->inside(PT))return true;
	PT += stepX;
	if (c1->inside(PT) && c2->inside(PT))return true;
	PT -= stepY;
	if (c1->inside(PT) && c2->inside(PT))return true;
	PT -= stepX;
	if (c1->inside(PT) && c2->inside(PT))return true;
	PT = pt - step;
	step /= 2;
	if (collidingREC(c1, c2, PT, step, layers - 1))return true;
	PT += stepX;
	if (collidingREC(c1, c2, PT, step, layers - 1))return true;
	PT += stepY;
	if (collidingREC(c1, c2, PT, step, layers - 1))return true;
	PT -= stepX;
	if (collidingREC(c1, c2, PT, step, layers - 1))return true;
	PT += stepZ;
	if (collidingREC(c1, c2, PT, step, layers - 1))return true;
	PT += stepX;
	if (collidingREC(c1, c2, PT, step, layers - 1))return true;
	PT -= stepY;
	if (collidingREC(c1, c2, PT, step, layers - 1))return true;
	PT -= stepX;
	if (collidingREC(c1, c2, PT, step, layers - 1))return true;
	return false;
}

bool colliding(collider* c1, collider* c2, unsigned char layers) {
	vec3d lower, upper;
	{
		vec3d l0, l1, u0, u1;
		(*c1).boundingBox(l0, u0);
		(*c2).boundingBox(l1, u1);
		lower.x = max(l0.x, l1.x);
		lower.y = max(l0.y, l1.y);
		lower.z = max(l0.z, l1.z);
		upper.x = min(u0.x, u1.x);
		upper.y = min(u0.y, u1.y);
		upper.z = min(u0.z, u1.z);
	}

	if (lower.x > upper.x)return false;
	if (lower.y > upper.y)return false;
	if (lower.z > upper.z)return false;
	vec3d pt = (lower + upper) / 2;
	if ((*c1).inside(pt) && (*c2).inside(pt))return true;

	vec3d step = (upper - lower) / 4;
	return collidingREC(c1, c2, pt,step, layers - 1);
}


vec3d getAColPTREC(collider* c1, collider* c2, vec3d pt, vec3d step, unsigned char layers, bool* colResult) {
	if (layers == 0) {
		if (colResult != nullptr)*colResult = false;
		return debugVec;
	}
	vec3d stepX(step.x * 2, 0, 0);
	vec3d stepY(0, step.y * 2, 0);
	vec3d stepZ(0, 0, step.z * 2);
	vec3d PT = pt - step;
	if (c1->inside(PT) && c2->inside(PT)) {
		if(colResult!=nullptr)*colResult = true;
		return PT;
	}
	PT += stepX;
	if (c1->inside(PT) && c2->inside(PT)) {
		if (colResult != nullptr)*colResult = true;
		return PT;
	}
	PT += stepY;
	if (c1->inside(PT) && c2->inside(PT)) {
		if (colResult != nullptr)*colResult = true;
		return PT;
	}
	PT -= stepX;
	if (c1->inside(PT) && c2->inside(PT)) {
		if (colResult != nullptr)*colResult = true;
		return PT;
	}
	PT += stepZ;
	if (c1->inside(PT) && c2->inside(PT)) {
		if (colResult != nullptr)*colResult = true;
		return PT;
	}
	PT += stepX;
	if (c1->inside(PT) && c2->inside(PT)) {
		if (colResult != nullptr)*colResult = true;
		return PT;
	}
	PT -= stepY;
	if (c1->inside(PT) && c2->inside(PT)) {
		if (colResult != nullptr)*colResult = true;
		return PT;
	}
	PT -= stepX;
	if (c1->inside(PT) && c2->inside(PT)) {
		if (colResult != nullptr)*colResult = true;
		return PT;
	}
	bool tempCol;
	vec3d rVal;
	PT = pt - step;
	step /= 2;
	rVal = getAColPTREC(c1, c2, PT, step, layers - 1, &tempCol);
	if(tempCol) {
		if (colResult != nullptr)*colResult = true;
		return rVal;
	}
	PT += stepX;
	rVal = getAColPTREC(c1, c2, PT, step, layers - 1, &tempCol);
	if (tempCol) {
		if (colResult != nullptr)*colResult = true;
		return rVal;
	}
	PT += stepY;
	rVal = getAColPTREC(c1, c2, PT, step, layers - 1, &tempCol);
	if (tempCol) {
		if (colResult != nullptr)*colResult = true;
		return rVal;
	}
	PT -= stepX;
	rVal = getAColPTREC(c1, c2, PT, step, layers - 1, &tempCol);
	if (tempCol) {
		if (colResult != nullptr)*colResult = true;
		return rVal;
	}
	PT += stepZ;
	rVal = getAColPTREC(c1, c2, PT, step, layers - 1, &tempCol);
	if (tempCol) {
		if (colResult != nullptr)*colResult = true;
		return rVal;
	}
	PT += stepX;
	rVal = getAColPTREC(c1, c2, PT, step, layers - 1, &tempCol);
	if (tempCol) {
		if (colResult != nullptr)*colResult = true;
		return rVal;
	}
	PT -= stepY;
	rVal = getAColPTREC(c1, c2, PT, step, layers - 1, &tempCol);
	if (tempCol) {
		if (colResult != nullptr)*colResult = true;
		return rVal;
	}
	PT -= stepX;
	rVal = getAColPTREC(c1, c2, PT, step, layers - 1, &tempCol);
	if (tempCol) {
		if (colResult != nullptr)*colResult = true;
		return rVal;
	}
	

	if (colResult != nullptr)*colResult = false;
	return debugVec;
}

vec3d getAColPT(collider* c1, collider* c2, unsigned char layers, bool * colResult) {
	vec3d lower, upper;
	{
		vec3d l0, l1, u0, u1;
		(*c1).boundingBox(l0, u0);
		(*c2).boundingBox(l1, u1);
		lower.x = max(l0.x, l1.x);
		lower.y = max(l0.y, l1.y);
		lower.z = max(l0.z, l1.z);
		upper.x = min(u0.x, u1.x);
		upper.y = min(u0.y, u1.y);
		upper.z = min(u0.z, u1.z);
	}

	if (colResult != nullptr) {
		if (lower.x > upper.x)*colResult = false;
		if (lower.y > upper.y)*colResult = false;
		if (lower.z > upper.z)*colResult = false;
	}

	vec3d pt = (lower + upper) / 2;
	if ((*c1).inside(pt) && (*c2).inside(pt)) {
		if (colResult != nullptr)*colResult = true;
		return pt;
	}

	vec3d step = (upper - lower) / 4;
	return getAColPTREC(c1, c2, pt, step, layers - 1, colResult);
}
