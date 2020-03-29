#pragma once
#include "collider.h";

enum class combinationTechnique{average, greaterVal , lowerVal , thisVal , otherVal};

struct physicalMaterial{
	size_t materialIndex = 0;//higher wins
	combinationTechnique CT = combinationTechnique::average;
	float coeffRestitution = 0.75;
	float coeffLFriction = 0.5;
	float coeffRFriction = 0;
};

static const physicalMaterial defaultPMat = {0,combinationTechnique::average,0.5f,0.5f,0.0f };

struct tensorOfInertia {
	vec3d IX;
	vec3d IY;
	vec3d IZ;
	vec3d IXInv;
	vec3d IYInv;
	vec3d IZInv;
};

vec3d operator *(tensorOfInertia t, vec3d v);
vec3d operator *(vec3d v, tensorOfInertia t);
vec3d operator /(tensorOfInertia t, vec3d v);
vec3d operator /(vec3d v ,tensorOfInertia t);


struct kineticProperties {
	vec3d COM;
	vec3d vel;
	vec3d angularVel;
};

struct dynamicProperties {
	kineticProperties kP;
	physicalMaterial pMat;
	double mass = 1;
	tensorOfInertia TOI;
};

vec3d calculateUniformCOM(collider* c, unsigned char detail = 20);
tensorOfInertia calculateUniformInertia(collider* c, double Mass, vec3d about, unsigned char detail = 20 ,bool* err = nullptr);