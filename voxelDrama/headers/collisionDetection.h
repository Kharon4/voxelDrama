#pragma once

#include"collider.h"

#define defaultLayers 5 // dont make it greater than 8

bool colliding(collider* c1, collider* c2, unsigned char layers = defaultLayers);

vec3d getAColPT(collider* c1, collider* c2, unsigned char layers = defaultLayers, bool* colResult = nullptr);
