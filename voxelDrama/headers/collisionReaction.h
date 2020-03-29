#pragma once
#include "physicalProperties.h"

void calculateNewVel(collider* c1, collider* c2, dynamicProperties *d1, dynamicProperties *d2, vec3d collPt , bool STATIC = false);//at max first body can be made static
