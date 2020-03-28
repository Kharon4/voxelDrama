#pragma once

#include "physicalProperties.h"
#include "collisionDetection.h"


#define defaultLayers 3
vec3d separateTillLastColl(collider* c1, collider* c2, vec3d axis, vec3d & OUTfinalSep, unsigned char layers = defaultLayers);//return last collision pt

void performLastSep(collider* c2, vec3d finalSep);