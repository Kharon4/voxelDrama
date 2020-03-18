#include "./../headers/collider.h"


//sphere collider
sphereCollider::sphereCollider(double Rad, vec3d Center) {
	rad = Rad;
	center = Center;
	modify.push_back(&center);
}

void sphereCollider::boundingBox(vec3d& lower, vec3d& upper) {
	vec3d val(rad, rad, rad);
	upper = vec3d::add(center, val);
	lower = vec3d::subtract(center, val);
}

intersectionType sphereCollider::inside(vec3d pt) {
	double r2 = rad * rad;
	double r = vec3d::subtract(pt, center).mag2();
	if (r < r2)return intersectionType::inside;
	if (r == r2)return intersectionType::overlap;
	return intersectionType::outside;
}


//cuboid collider
void cuboidCollider::boundingBox(vec3d& lower, vec3d& upper) {
	lower = vec3d::subtract(center, dim);
	upper = vec3d::add(center, dim);
}

intersectionType cuboidCollider::inside(vec3d pt) {
	vec3d Lower, Upper;
	boundingBox(Lower, Upper);
	if (pt.x<Lower.x || pt.y<Lower.y || pt.z<Lower.z || pt.x>Upper.x || pt.y>Upper.y || pt.z>Upper.z)
		return intersectionType::outside;
	if (pt.x==Lower.x || pt.y==Lower.y || pt.z==Lower.z || pt.x==Upper.x || pt.y==Upper.y || pt.z==Upper.z)
		return intersectionType::overlap;
	return intersectionType::inside;

}

cuboidCollider::cuboidCollider(vec3d Center, vec3d Dim) {
	center = Center;
	dim = Dim;
	modify.push_back(&center);
}


//capsule collider

capsuleCollider::capsuleCollider(vec3d A, vec3d B, double rad) {
	a = A;
	b = B;
	r = rad;
	modify.push_back(&a);
	modify.push_back(&b);
}

#define min(x,y) ((x<y)?x:y)
#define max(x,y) ((x>y)?x:y)

void capsuleCollider::boundingBox(vec3d& lower, vec3d& upper) {
	sphereCollider temp(r, a);
	temp.boundingBox(lower, upper);
	vec3d l1, u1;
	temp.center = b;
	temp.boundingBox(l1, u1);
	lower.x = min(lower.x, l1.x);
	lower.y = min(lower.y, l1.y);
	lower.z = min(lower.z, l1.z);
	upper.x = max(upper.x, u1.x);
	upper.y = max(upper.y, u1.y);
	upper.z = max(upper.z, u1.z);
}

intersectionType capsuleCollider::inside(vec3d pt) {
	linearMathD::line ln(a, vec3d::subtract(b, a));
	double dist = linearMathD::distance(pt, ln);
	if (dist > r)return intersectionType::outside;
	if (dist < r) {
		//check for line
		if (vec3d::dot(vec3d::subtract(pt, a), vec3d::subtract(b, a)) > 0) {
			if (vec3d::dot(vec3d::subtract(pt, b), vec3d::subtract(a, b)) > 0)
				return intersectionType::inside;
		}

		//check spheres
		sphereCollider temp(r, a);
		intersectionType rval = temp.inside(pt);
		if (rval != intersectionType::outside)return rval;
		temp.center = b;
		return temp.inside(pt);
	}
	return intersectionType::overlap;
}


//mesh collider

meshCollider::meshCollider(std::vector<linearMathD::plane>* Planes) {
	DRs = new vec3d[(*Planes).size()];
	PTs = new vec3d[(*Planes).size()];
	for (size_t i = 0; i < Planes->size(); ++i) {
		DRs[i] = (*Planes)[i].getDr();
		PTs[i] = (*Planes)[i].getPt();
		modify.push_back(PTs + i);
		modifyR.push_back(DRs + i);
	}
}

meshCollider::~meshCollider() {
	if (DRs != nullptr)delete[] DRs;
	if (PTs != nullptr)delete[] PTs;
}

void meshCollider::boundingBox(vec3d& lower, vec3d& upper) {
	lower = PTs[0];
	upper = PTs[0];
	for (size_t i = 0; i < modify.size(); ++i) {
		if (lower.x > PTs[i].x)lower.x = PTs[i].x;
		if (lower.y > PTs[i].y)lower.y = PTs[i].y;
		if (lower.z > PTs[i].z)lower.z = PTs[i].z;
		if (upper.x < PTs[i].x)upper.x = PTs[i].x;
		if (upper.y < PTs[i].y)upper.y = PTs[i].y;
		if (upper.z < PTs[i].z)upper.z = PTs[i].z;
	}
}

intersectionType meshCollider::inside(vec3d pt) {
	intersectionType rval = intersectionType::inside;
	for (size_t i = 0; i < modify.size(); ++i) {
		double val = vec3d::dot(vec3d::subtract(pt, PTs[i]), DRs[i]);
		if (val > 0)return intersectionType::outside;
		else if (val == 0)rval = intersectionType::overlap;
	}
	return rval;
}