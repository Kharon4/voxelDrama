#include "./../headers/collider.h"
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