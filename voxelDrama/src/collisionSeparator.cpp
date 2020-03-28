#include "./../headers/collisionSeparator.h"


vec3d getMaxSeparationVector(collider* c1, collider* c2, vec3d axis /*dir in which c2 must be moved*/) {//only c2 will be moved
	//get total displacement
	vec3d l1, u1, l2, u2;
	c1->boundingBox(l1, u1);
	c2->boundingBox(l2, u2);
	double t = -1;
	
	if (axis.x < 0)t = (u2.x - l1.x) / axis.x;
	else if(axis.x >0)t = (u1.x - l2.x) / axis.x;

	if (axis.y < 0) {
		double temp = (u2.y - l1.y) / axis.y;
		if (temp < t || t < 0)t = temp;
	}
	else if (axis.y > 0) {
		double temp = (u1.y - l2.y) / axis.y;
		if (temp < t || t < 0)t = temp;
	}

	if (axis.z < 0) {
		double temp = (u2.z - l1.z) / axis.z;
		if (temp < t || t < 0)t = temp;
	}
	else if (axis.z > 0) {
		double temp = (u1.z - l2.z) / axis.z;
		if (temp < t || t < 0)t = temp;
	}

	return axis * t;
}

vec3d separateTillLastColl(collider* c1, collider* c2, vec3d axis, vec3d& OUTfinalSep, unsigned char layers) {
	vec3d maxSep = getMaxSeparationVector(c1, c2, axis);
	vec3d lastUncollided = maxSep + c2->M.CS.getOrigin();
	vec3d lastCollided = c2->M.CS.getOrigin();
	vec3d colPt = getAColPT(c1, c2);
	bool result;
	maxSep /= 2;
	c2->M.CS.setOrigin(c2->M.CS.getOrigin() + maxSep);
	c2->MT.CS.setOrigin(c2->MT.CS.getOrigin() + maxSep);
	c2->M.update();
	c2->MT.update();
	for (unsigned char i = 0; i < layers; ++i) {
		maxSep /= 2;
		if (colliding(c1, c2)) {
			colPt = getAColPT(c1, c2);
			lastCollided = c2->M.CS.getOrigin();
			c2->M.CS.setOrigin(c2->M.CS.getOrigin() + maxSep);
			c2->MT.CS.setOrigin(c2->MT.CS.getOrigin() + maxSep);
		}
		else {
			lastUncollided = c2->M.CS.getOrigin();
			c2->M.CS.setOrigin(c2->M.CS.getOrigin() - maxSep);
			c2->MT.CS.setOrigin(c2->MT.CS.getOrigin() - maxSep);
		}
		c2->M.update();
		c2->MT.update();
	}
	c2->M.CS.setOrigin(lastCollided);
	c2->MT.CS.setOrigin(lastCollided);
	c2->M.update();
	c2->MT.update();
	OUTfinalSep = lastUncollided;
	return colPt;
}

void performLastSep(collider* c2, vec3d finalSep) {
	c2->M.CS.setOrigin(finalSep);
	c2->MT.CS.setOrigin(finalSep);
	c2->M.update();
	c2->MT.update();
}