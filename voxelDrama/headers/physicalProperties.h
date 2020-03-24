#pragma once
#include "collider.h";

enum class combinationTechnique{average , greaterVal , lowerVal , thisVal , otherVal};

struct physicalMaterial{
	size_t materialIndex = 0;//higher wins
	combinationTechnique CT = combinationTechnique::average;
	float coeffRestitution;
	float coeffLFriction;
	float coeffRFriction;
};

struct tensorOfInertia {
	vec3d IX;
	vec3d IY;
	vec3d IZ;
};

struct kineticProperties {
	vec3d COM;
	vec3d vel;
	vec3d angularVel;
};

struct dynamicProperties {
	kineticProperties kP;
	physicalMaterial pMat;
	double mass;
	tensorOfInertia TOI;
};

vec3d calculateUniformCOM(collider* c, unsigned char detail = 20);
tensorOfInertia calculateUniformInertia(collider* c, double Mass, vec3d about, unsigned char detail = 20);