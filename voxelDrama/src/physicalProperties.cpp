#include "./../headers/physicalProperties.h"



vec3d calculateUniformCOM(collider* c, unsigned char detail) {
	unsigned long long mass = 0;
	vec3d lower, upper;
	c->boundingBox(lower, upper);
	vec3d rVal = vec3d(0, 0, 0);
	vec3d PT = lower;
	vec3d step = (upper - lower) / detail;

	for (int i = 0; i <= detail; ++i) {
		PT.y = lower.y;
		for (int j = 0; j <= detail; ++j) {
			PT.x = lower.x;
			for (int k = 0; k <= detail; ++k) {
				if (c->inside(PT)) {
					rVal += PT;
					mass++;
				}
				PT.x += step.x;
			}
			PT.y += step.y;
		}
		PT.z += step.z;
	}

	if (mass != 0)rVal /= mass;
	return rVal;
}

bool completeTOI(tensorOfInertia& TOI) {//finds inverse
	
	//diagonals of the adjoint matrix
	double det1 = TOI.IY.y * TOI.IZ.z - TOI.IY.z * TOI.IZ.y;
	TOI.IXInv.x = det1;//A11
	TOI.IYInv.y = TOI.IX.x * TOI.IZ.z - TOI.IX.z * TOI.IZ.x;//A22
	TOI.IZInv.z = TOI.IX.x * TOI.IY.y - TOI.IX.y * TOI.IY.x;//A33
	
	//other elements
	double det2 = TOI.IY.x * TOI.IZ.z - TOI.IY.z * TOI.IZ.x;
	TOI.IXInv.y = det2;//A12
	TOI.IYInv.x = TOI.IXInv.y;//A21
	double det3 = TOI.IY.x * TOI.IZ.y - TOI.IY.y * TOI.IZ.x;
	TOI.IXInv.z = det3;//A13
	TOI.IZInv.x = TOI.IXInv.z;//A31
	TOI.IYInv.z = TOI.IX.x * TOI.IZ.y - TOI.IX.y * TOI.IZ.x;//A23
	TOI.IZInv.y = TOI.IYInv.z;//A32


	//det of matrix;
	double det = TOI.IX.x * det1 - TOI.IX.y * det2 + TOI.IX.z * det3;
	if (det == 0)return true;
	TOI.IXInv /= det;
	TOI.IYInv /= det;
	TOI.IZInv /= det;
	return false;
}

tensorOfInertia calculateUniformInertia(collider* c, double Mass, vec3d about, unsigned char detail , bool * err) {
	unsigned long long mass = 0;
	vec3d lower, upper;
	c->boundingBox(lower, upper);
	vec3d PT = lower;
	vec3d step = (upper - lower) / detail;

	tensorOfInertia TOI;
	TOI.IX = vec3d(0, 0, 0);
	TOI.IY = vec3d(0, 0, 0);
	TOI.IZ = vec3d(0, 0, 0);

	for (int i = 0; i <= detail; ++i) {
		PT.y = lower.y;
		for (int j = 0; j <= detail; ++j) {
			PT.x = lower.x;
			for (int k = 0; k <= detail; ++k) {
				if (c->inside(PT)) {
					vec3d disp = PT - about;
					TOI.IX.x += disp.y * disp.y + disp.z * disp.z;
					TOI.IY.y += disp.x * disp.x + disp.z * disp.z;
					TOI.IZ.z += disp.y * disp.y + disp.x * disp.x;
					TOI.IX.y -= disp.y * disp.x;
					TOI.IX.z -= disp.z * disp.x;
					TOI.IY.z -= disp.y * disp.z;
					mass++;
				}
				PT.x += step.x;
			}
			PT.y += step.y;
		}
		PT.z += step.z;
	}

	if (mass != 0) {
		TOI.IX.x /= mass;
		TOI.IY.y /= mass;
		TOI.IZ.z /= mass;
		TOI.IX.y /= mass;
		TOI.IX.z /= mass;
		TOI.IY.z /= mass;
		TOI.IX.x *= Mass;
		TOI.IY.y *= Mass;
		TOI.IZ.z *= Mass;
		TOI.IX.y *= Mass;
		TOI.IX.z *= Mass;
		TOI.IY.z *= Mass;
	}
	TOI.IY.x = TOI.IX.y;
	TOI.IZ.x = TOI.IX.z;
	TOI.IZ.y = TOI.IY.z;

	if (completeTOI(TOI)) {
		if (err != nullptr)(*err) = true;
	}
	else {
		if (err != nullptr)(*err) = false;
	}
	return TOI;
}

