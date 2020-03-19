#include "./../headers/collisionDetection.h"


//defination of inside = all are inside || overlapping
//defination of outside = all outside || overlapping
//any other combo means finer detail

detectionResult detect(collider* col, vec3d lower, vec3d upper , detectionResult *forced = nullptr) {
	unsigned char in = 0, out = 0;
	vec3d checkPt = lower;//000 - 0
	if (col->inside(checkPt) == intersectionType::outside)out++;
	if (col->inside(checkPt) == intersectionType::inside)in++;
	checkPt.x = upper.x;//001 - 1
	if (col->inside(checkPt) == intersectionType::outside)out++;
	if (col->inside(checkPt) == intersectionType::inside)in++;
	checkPt.y = upper.y;//011 - 3
	if (col->inside(checkPt) == intersectionType::outside)out++;
	if (col->inside(checkPt) == intersectionType::inside)in++;
	checkPt.z = upper.z;//111 - 7
	if (col->inside(checkPt) == intersectionType::outside)out++;
	if (col->inside(checkPt) == intersectionType::inside)in++;
	checkPt.y = lower.y;//101 - 5
	if (col->inside(checkPt) == intersectionType::outside)out++;
	if (col->inside(checkPt) == intersectionType::inside)in++;
	checkPt.x = lower.x;//100 - 4
	if (col->inside(checkPt) == intersectionType::outside)out++;
	if (col->inside(checkPt) == intersectionType::inside)in++;
	checkPt.y = upper.y;//110 - 6
	if (col->inside(checkPt) == intersectionType::outside)out++;
	if (col->inside(checkPt) == intersectionType::inside)in++;
	checkPt.z = lower.z;//010 - 2
	if (col->inside(checkPt) == intersectionType::outside)out++;
	if (col->inside(checkPt) == intersectionType::inside)in++;

	if (out == 0)return detectionResult::colliding;
	if (in == 0)return detectionResult::nonColliding;
	if (forced!=nullptr) {
		if (out > in)return detectionResult::nonCollidingNFD;
		return detectionResult::collidingNFD;
	}
	return detectionResult::NFD;
}

ocNode<detectionResult>* createCollisionTreeR
(collider* c1, collider* c2, unsigned char currentLayer , vec3d lower , vec3d upper ,intersectionType * drs ) {
	ocNode<detectionResult>* rval = new ocNode<detectionResult>;
	
	//calculate current situation
	unsigned char col = 0, nonCol = 0;
	for (unsigned char i = 0; i < 8; ++i) {
		if (drs[i]==intersectionType::inside)col++;
		else if(drs[i]==intersectionType::outside) nonCol++;
	}
	if (nonCol == 0) {
		rval->data = detectionResult::colliding;
		return rval;
	}
	else if (col == 0) {
		rval->data = detectionResult::nonColliding;
		return rval;
	}
	if (currentLayer == 0) {
		if (nonCol > col)
			rval->data = detectionResult::nonCollidingNFD;
		else
			rval->data = detectionResult::collidingNFD;
		return rval;
	}
	rval->data = detectionResult::NFD;
	//calculate rest of the pts
	intersectionType nDrs[27];//x + y*3 + z*9
	nDrs[0] = drs[0];
	nDrs[2] = drs[1];
	nDrs[6] = drs[2];
	nDrs[8] = drs[3];
	nDrs[18] = drs[4];
	nDrs[20] = drs[5];
	nDrs[24] = drs[6];
	nDrs[26] = drs[7];
	vec3d step = vec3d::multiply(vec3d::subtract(upper, lower), 0.5);
	//lower layer
	vec3d pt = lower;
	pt.x += step.x;
	nDrs[1] = (intersectionType)(c1->inside(pt) && c2->inside(pt));
	pt.x = lower.x;
	pt.y += step.y;
	nDrs[3] = (intersectionType)(c1->inside(pt) && c2->inside(pt));
	pt.x += step.x;
	nDrs[4] = (intersectionType)(c1->inside(pt) && c2->inside(pt));
	pt.x += step.x;
	nDrs[5] = (intersectionType)(c1->inside(pt) && c2->inside(pt));
	pt.x -= step.x;
	pt.y = upper.y;
	nDrs[7] = (intersectionType)(c1->inside(pt) && c2->inside(pt));
	//upper layer
	pt = lower;
	pt.z = upper.z;
	pt.x += step.x;
	nDrs[19] = (intersectionType)(c1->inside(pt) && c2->inside(pt));
	pt.x = lower.x;
	pt.y += step.y;
	nDrs[21] = (intersectionType)(c1->inside(pt) && c2->inside(pt));
	pt.x += step.x;
	nDrs[22] = (intersectionType)(c1->inside(pt) && c2->inside(pt));
	pt.x += step.x;
	nDrs[23] = (intersectionType)(c1->inside(pt) && c2->inside(pt));
	pt.x -= step.x;
	pt.y = upper.y;
	nDrs[25] = (intersectionType)(c1->inside(pt) && c2->inside(pt));
	//middle layer
	pt.z -= step.z;
	pt.y = lower.y;
	for (unsigned char i = 0; i < 2; ++i) {
		pt.x = lower.x;
		for (unsigned char j = 0; j < 2; ++j) {
			nDrs[8 + j + i * 3] = (intersectionType)(c1->inside(pt) && c2->inside(pt));
			pt.x += step.x;
		}
		pt.y += step.y;
	}
	//save children by calling their procedures
	drs[0] = nDrs[0];
	drs[1] = nDrs[1];
	drs[2] = nDrs[3];
	drs[3] = nDrs[4];
	drs[4] = nDrs[9];
	drs[5] = nDrs[10];
	drs[6] = nDrs[12];
	drs[7] = nDrs[13];
	pt = lower;
	rval->nodes[0] = createCollisionTreeR(c1, c2, currentLayer - 1, pt, vec3d::add(pt, step), drs);
	drs[0] = nDrs[1];
	drs[1] = nDrs[2];
	drs[2] = nDrs[4];
	drs[3] = nDrs[5];
	drs[4] = nDrs[10];
	drs[5] = nDrs[11];
	drs[6] = nDrs[13];
	drs[7] = nDrs[14];
	pt.x += step.x;
	rval->nodes[1] = createCollisionTreeR(c1, c2, currentLayer - 1, pt, vec3d::add(pt, step), drs);
	drs[0] = nDrs[4];
	drs[1] = nDrs[5];
	drs[2] = nDrs[7];
	drs[3] = nDrs[8];
	drs[4] = nDrs[13];
	drs[5] = nDrs[14];
	drs[6] = nDrs[16];
	drs[7] = nDrs[17];
	pt.y += step.y;
	rval->nodes[3] = createCollisionTreeR(c1, c2, currentLayer - 1, pt, vec3d::add(pt, step), drs);
	drs[0] = nDrs[3];
	drs[1] = nDrs[4];
	drs[2] = nDrs[6];
	drs[3] = nDrs[7];
	drs[4] = nDrs[12];
	drs[5] = nDrs[13];
	drs[6] = nDrs[15];
	drs[7] = nDrs[16];
	pt.x -= step.x;
	rval->nodes[2] = createCollisionTreeR(c1, c2, currentLayer - 1, pt, vec3d::add(pt, step), drs);
	drs[0] = nDrs[12];
	drs[1] = nDrs[13];
	drs[2] = nDrs[15];
	drs[3] = nDrs[16];
	drs[4] = nDrs[21];
	drs[5] = nDrs[22];
	drs[6] = nDrs[24];
	drs[7] = nDrs[25];
	pt.z += step.z;
	rval->nodes[6] = createCollisionTreeR(c1, c2, currentLayer - 1, pt, vec3d::add(pt, step), drs);
	drs[0] = nDrs[13];
	drs[1] = nDrs[14];
	drs[2] = nDrs[16];
	drs[3] = nDrs[17];
	drs[4] = nDrs[22];
	drs[5] = nDrs[23];
	drs[6] = nDrs[25];
	drs[7] = nDrs[26];
	pt.x += step.x;
	rval->nodes[7] = createCollisionTreeR(c1, c2, currentLayer - 1, pt, vec3d::add(pt, step), drs);
	drs[0] = nDrs[10];
	drs[1] = nDrs[11];
	drs[2] = nDrs[13];
	drs[3] = nDrs[14];
	drs[4] = nDrs[19];
	drs[5] = nDrs[20];
	drs[6] = nDrs[22];
	drs[7] = nDrs[23];
	pt.y -= step.y;
	rval->nodes[5] = createCollisionTreeR(c1, c2, currentLayer - 1, pt, vec3d::add(pt, step), drs);
	drs[0] = nDrs[9];
	drs[1] = nDrs[10];
	drs[2] = nDrs[12];
	drs[3] = nDrs[13];
	drs[4] = nDrs[18];
	drs[5] = nDrs[19];
	drs[6] = nDrs[21];
	drs[7] = nDrs[22];
	pt.x -= step.x;
	rval->nodes[4] = createCollisionTreeR(c1, c2, currentLayer - 1, pt, vec3d::add(pt, step), drs);

}

ocNode<detectionResult>* createCollisionTree(collider* c1, collider* c2, unsigned char layers) {
	vec3d lower, upper;
	{
		//get bounding boxes
		vec3d l1, u1, l2, u2;
		c1->boundingBox(l1, u1);
		c2->boundingBox(l2, u2);

		//find boundging box intersections
		lower.x = ((l1.x > l2.x) ? l1.x : l2.x);
		lower.y = ((l1.y > l2.y) ? l1.y : l2.y);
		lower.z = ((l1.z > l2.z) ? l1.z : l2.z);
		upper.x = ((u1.x < u2.x) ? u1.x : u2.x);
		upper.y = ((u1.y < u2.y) ? u1.y : u2.y);
		upper.z = ((u1.z < u2.z) ? u1.z : u2.z);
	}

	//calculate initial points
	intersectionType data[8];
	vec3d step = vec3d::multiply(vec3d::subtract(upper, lower),0.5);
	vec3d pt = lower;
	for (unsigned char i = 0; i < 2; ++i) {
		pt.x = lower.x;
		pt.y = lower.y;
		for (unsigned char j = 0; j < 2; ++j) {
			pt.x = lower.x;
			for (unsigned char k = 0; k < 2; ++k) {
				data[calcTreePos(k, j, i)] = (intersectionType(c1->inside(pt) && c2->inside(pt)));
				pt.x += step.x;
			}
			pt.y += step.y;
		}
		pt.z += step.z;
	}
	return createCollisionTreeR(c1, c2, layers, lower, upper, data);

}
