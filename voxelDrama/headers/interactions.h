#pragma once

#include "physicalProperties.h"

void applyImpulse(dynamicProperties d, vec3d force, vec3d pt);

void applyImpulse(dynamicProperties d, vec3d force);

void applyAngularImpulse(dynamicProperties d, vec3d angularImpulse);

vec3d calculateAngularImpulse(dynamicProperties d, vec3d force, vec3d pt);