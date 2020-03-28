#include "interactions.h"

void applyImpulse(dynamicProperties d, vec3d force, vec3d pt) {
	d.kP.vel += force / d.mass;
	d.kP.angularVel += vec3d::cross(pt - d.kP.COM, force) / d.TOI;
}


void applyImpulse(dynamicProperties d, vec3d force) {
	d.kP.vel += force / d.mass;
}


void applyAngularImpulse(dynamicProperties d, vec3d angularImpulse) {
	d.kP.angularVel += angularImpulse / d.TOI;
}


vec3d calculateAngularImpulse(dynamicProperties d, vec3d force, vec3d pt) {
	return vec3d::cross(pt - d.kP.COM, force) / d.TOI;
}