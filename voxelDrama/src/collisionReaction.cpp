#include "./../headers/collisionReaction.h"

inline vec3d calculatePtVel(kineticProperties kp, vec3d pt) {
	return kp.vel + (kp.angularVel * (pt - kp.COM));
}

#define MAX(a,b) ((a>b)?a:b)
#define MIN(a,b) ((a<b)?a:b)

physicalMaterial getCollphysicalMat(physicalMaterial p1 , physicalMaterial p2) {
	physicalMaterial rVal;
	combinationTechnique c;
	bool firstChosen = true;
	if (p1.materialIndex > p2.materialIndex) c = p1.CT;
	else if (p1.materialIndex < p2.materialIndex) { 
		firstChosen = false;
		c = p2.CT; 
	}
	else {
		if (p1.CT == p2.CT) { 
			if (p1.CT == combinationTechnique::otherVal)return defaultPMat;
			c = p1.CT; 
		}
		else if (p1.CT == combinationTechnique::otherVal) {
			return p2;
		}
		else if (p2.CT == combinationTechnique::otherVal) {
			return p1;
		}
		else if (p1.CT == combinationTechnique::thisVal) {
			return p1;
		}
		else if (p2.CT == combinationTechnique::thisVal) {
			return p2;
		}
		else {
			c = combinationTechnique::average;
		}
	}

	switch (c)
	{
	case combinationTechnique::average:
		rVal.coeffLFriction = (p1.coeffLFriction + p2.coeffLFriction) / 2;
		rVal.coeffRFriction = (p1.coeffRFriction + p2.coeffRFriction) / 2;
		rVal.coeffRestitution = (p1.coeffRestitution + p2.coeffRestitution) / 2;
		break;
	case combinationTechnique::greaterVal:
		rVal.coeffLFriction = MAX(p1.coeffLFriction , p2.coeffLFriction);
		rVal.coeffRFriction = MAX(p1.coeffRFriction , p2.coeffRFriction);
		rVal.coeffRestitution = MAX(p1.coeffRestitution , p2.coeffRestitution);
		break;
	case combinationTechnique::lowerVal:
		rVal.coeffLFriction = MIN(p1.coeffLFriction, p2.coeffLFriction);
		rVal.coeffRFriction = MIN(p1.coeffRFriction, p2.coeffRFriction);
		rVal.coeffRestitution = MIN(p1.coeffRestitution, p2.coeffRestitution);
		break;
	case combinationTechnique::thisVal:
		if (firstChosen)return p1;
		else return p2;
		break;
	case combinationTechnique::otherVal:
		if (firstChosen)return p2;
		else return p1;
		break;
	default:
		break;
	}
	return rVal;
}


inline vec3d getNormal(collider* c1, collider* c2, vec3d pt) {//wrt c2
	return ((c2->getNormal(pt) - c1->getNormal(pt)) / 2);
}

inline vec3d getColParallel(vec3d normal, vec3d v1, vec3d v2) {//wrt c2
	return vec3d::normalize((v1-v2) - (vec3d::dot((v1 - v2), normal) * normal)) ;
}

void calculateNewVel(collider* c1, collider* c2, dynamicProperties* d1, dynamicProperties* d2, vec3d collPt) {
	
}