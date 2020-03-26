#include "./../headers/collisionReaction.h"

inline vec3d calculatePtVel(kineticProperties kp, vec3d pt) {
	return kp.vel + (kp.angularVel * (pt - kp.COM));
}

void calculateNewVel(collider* c1, collider* c2, dynamicProperties* d1, dynamicProperties* d2, vec3d collPt) {

}