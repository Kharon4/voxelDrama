#pragma once

#include "vec3.cuh"
#include "linearMath.cuh"
#include "transform.cuh"
#include <vector>

enum intersectionType { outside = 0,overlap = 1, inside = 2 };

class collider {
public:
	manipulation3d::transformd M;//modify
	manipulation3d::transformd MR;//modify ony rotate
	manipulation3d::transformd MT;//modify only translate
	virtual void boundingBox(vec3d& lower, vec3d& upper) {};
	virtual intersectionType inside(vec3d pt) { return intersectionType::outside; };
	virtual vec3d getNormal(vec3d pt) { return vec3d(1, 0, 0); }//unit vector pointing outward
};


class sphereCollider : public collider {
public:
	vec3d center;
	double rad;

	sphereCollider(double Rad = 1, vec3d Center = vec3d(0,0,0));
	void boundingBox(vec3d& lower, vec3d& upper);
	intersectionType inside(vec3d pt);
	vec3d getNormal(vec3d pt);
};

class cuboidCollider : public collider {//grid alligned
public:
	vec3d center;
	vec3d dim;

	cuboidCollider(vec3d Center = vec3d(0, 0, 0), vec3d Dim = vec3d(1, 1, 1));
	void boundingBox(vec3d& lower, vec3d& upper);
	intersectionType inside(vec3d pt);
	vec3d getNormal(vec3d pt);
};

class capsuleCollider : public collider {
public:
	double r;
	vec3d a, b;//ends
	capsuleCollider(vec3d A = vec3d(0,0,1), vec3d B = vec3d(0,0,-1), double rad = 1);
	void boundingBox(vec3d& lower, vec3d& upper);
	intersectionType inside(vec3d pt);
	vec3d getNormal(vec3d pt);
};

class meshCollider : public collider {//convex hull with normals outward
private:
	vec3d* DRs = nullptr;
	vec3d* PTs = nullptr;

public:
	meshCollider(const meshCollider&) = delete;
	meshCollider& operator=(const meshCollider&) = delete;
	meshCollider(std::vector<linearMath::planed>* Planes, vec3d center = vec3d(0,0,0));
	~meshCollider();

	void boundingBox(vec3d& lower, vec3d& upper);
	intersectionType inside(vec3d pt);
	vec3d getNormal(vec3d pt);
};

class compoundCollider : public collider {
private:
	std::vector<collider *>colls;
public:
	compoundCollider(const compoundCollider&) = delete;
	compoundCollider& operator=(const compoundCollider&) = delete;
	compoundCollider(std::vector<collider *> colliders);
	~compoundCollider() { ; }

	void boundingBox(vec3d& lower, vec3d& upper);
	intersectionType inside(vec3d pt);
	vec3d getNormal(vec3d pt);
};

