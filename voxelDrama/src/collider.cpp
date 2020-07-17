#include "collider.h"

//sphere collider
sphereCollider::sphereCollider(double Rad, vec3d Center) {
	rad = Rad;
	center = Center;
	M.CS.setOrigin(center);
	MR.CS.setOrigin(center);
	MT.CS.setOrigin(center);
	MT.addVec(center, &center);
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

vec3d sphereCollider::getNormal(vec3d pt) {
	return vec3d::normalizeRaw_s(pt - center);
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
	dim = vec3d::multiply(Dim,0.5);
	M.CS.setOrigin(center);
	MR.CS.setOrigin(center);
	MT.CS.setOrigin(center);
	MT.addVec(center, &center);
}

vec3d cuboidCollider::getNormal(vec3d pt) {
	vec3d lower , upper;
	boundingBox(lower, upper);
	unsigned char minPlane = 0;
	double min = pt.x - lower.x;
	double temp;
	temp = upper.x - pt.x;
	if (temp < min) {
		min = temp;
		minPlane = 1;
	}
	temp = pt.y - lower.y;
	if (temp < min) {
		min = temp;
		minPlane = 2;
	}
	temp = upper.y - pt.y;
	if (temp < min) {
		min = temp;
		minPlane = 3;
	}
	temp = pt.z - lower.z;
	if (temp < min) {
		min = temp;
		minPlane = 4;
	}
	temp = upper.z - pt.z;
	if (temp < min) {
		minPlane = 5;
	}
	switch (minPlane)
	{
	case 0:
		return vec3d(-1, 0, 0);
	case 1:
		return vec3d(1, 0, 0);
	case 2:
		return vec3d(0, -1, 0);
	case 3:
		return vec3d(0, 1, 0);
	case 4:
		return vec3d(0, 0, -1);
	default:
		return vec3d(0, 0, 1);
	}

}

//capsule collider

capsuleCollider::capsuleCollider(vec3d A, vec3d B, double rad) {
	a = A;
	b = B;
	r = rad;
	vec3d center = (a + b) / 2;
	M.CS.setOrigin(center);
	MR.CS.setOrigin(center);
	MT.CS.setOrigin(center);
	M.addVec(a, &a);
	M.addVec(b, &b);
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
	linearMath::lined ln(a, vec3d::subtract(b, a));
	double dist = linearMath::distance(pt, ln);
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

vec3d capsuleCollider::getNormal(vec3d pt) {
	vec3d DR = b - a;
	double component = vec3d::componentRaw_s((pt - a), DR);
	if (component < 0)
		return vec3d::normalizeRaw_s(pt - a);
	if (component * component > DR.mag2())
		return vec3d::normalizeRaw_s(pt - b);
	else return vec3d::normalizeRaw_s((pt - a) - vec3d::dot(pt - a,DR) * (DR) / (DR).mag2());
}

//mesh collider

meshCollider::meshCollider(std::vector<linearMath::planed>* Planes,vec3d center) {
	DRs = new vec3d[(*Planes).size()];
	PTs = new vec3d[(*Planes).size()];
	M.CS.setOrigin(center);
	MR.CS.setOrigin(center);
	MT.CS.setOrigin(center);
	for (size_t i = 0; i < Planes->size(); ++i) {
		DRs[i] = (*Planes)[i].getDr();
		PTs[i] = (*Planes)[i].getPt();
		M.addVec(PTs[i], PTs + i);
		MR.addVec(DRs[i], DRs + i);
	}
}

meshCollider::~meshCollider() {
	if (DRs != nullptr)delete[] DRs;
	if (PTs != nullptr)delete[] PTs;
}

void meshCollider::boundingBox(vec3d& lower, vec3d& upper) {
	lower = PTs[0];
	upper = PTs[0];
	for (size_t i = 0; i < (*M.getData()).size(); ++i) {
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
	for (size_t i = 0; i < (*M.getData()).size(); ++i) {
		double val = vec3d::dot(vec3d::subtract(pt, PTs[i]), DRs[i]);
		if (val > 0)return intersectionType::outside;
		else if (val == 0)rval = intersectionType::overlap;
	}
	return rval;
}

vec3d meshCollider::getNormal(vec3d pt) {
	if ((*M.getData()).size())return vec3d(0,0,0);
	size_t meshId = 0;
	double min = abs(linearMath::aDistance(pt,linearMath::planed(PTs[0],DRs[0])));
	for (size_t i = 1; i < (*M.getData()).size(); ++i) {
		double dist = abs(linearMath::aDistance(pt, linearMath::planed(PTs[i], DRs[i])));
		if (dist < min) {
			min = dist;
			meshId = i;
		}
	}
	return DRs[meshId];
}

//compound collider
compoundCollider::compoundCollider(std::vector<collider*> colliders) {
	for (size_t i = 0; i < colliders.size(); ++i) {
		colls.push_back(colliders[i]);
		for (size_t j = 0; j < (*(*colliders[i]).M.getData()).size(); ++j) {
			M.addVec((*colliders[i]->M.getData())[j], &(*colliders[i]->M.getData())[j]);
		}
		for (size_t j = 0; j < (*(*colliders[i]).MR.getData()).size(); ++j) {
			MR.addVec((*colliders[i]->MR.getData())[j], &(*colliders[i]->MR.getData())[j]);
		}
		for (size_t j = 0; j < (*(*colliders[i]).MT.getData()).size(); ++j) {
			MT.addVec((*colliders[i]->MT.getData())[j], &(*colliders[i]->MT.getData())[j]);
		}
	}
}

void compoundCollider::boundingBox(vec3d& lower, vec3d& upper) {
	if (colls.size() == 0)return;
	colls[0]->boundingBox(lower, upper);
	vec3d L, U;
	for (size_t i = 1; i < colls.size(); ++i) {
		colls[i]->boundingBox(L, U);
		lower.x = min(L.x, lower.x);
		lower.y = min(L.y, lower.y);
		lower.z = min(L.z, lower.z);
		upper.x = max(U.x, upper.x);
		upper.y = max(U.y, upper.y);
		upper.z = max(U.z, upper.z);
	}
}

intersectionType compoundCollider::inside(vec3d pt) {
	intersectionType rVal = intersectionType::outside;
	intersectionType temp;
	for (size_t i = 0; i < colls.size(); ++i) {
		temp = colls[i]->inside(pt);
		if (temp == intersectionType::inside)return temp;
		if(temp!= intersectionType::outside)rVal = temp;
	}
	return rVal;
}

vec3d compoundCollider::getNormal(vec3d pt) {
	for (size_t i = 0; i < colls.size(); ++i) {
		if (colls[i]->inside(pt)) {
			return colls[i]->getNormal(pt);
		}
	}
	return vec3d(0, 0, 0);
}