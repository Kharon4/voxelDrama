#include "VoxelDrama_Core.h"

void physicalWorld::addBody(collider* c, dynamicProperties* dp) {
	colls.push_back(c);
	dynamicProperties D;
	if (dp != nullptr) {
		D = *dp;
	}
	else {
		D.kP.COM = calculateUniformCOM(c);
		D.TOI = calculateUniformInertia(c, 1, D.kP.COM);
	}
	DP.push_back(D);
}

void physicalWorld::addStaticBody(collider* c, dynamicProperties* dp) {
	staticColls.push_back(c);
	dynamicProperties D;
	if (dp != nullptr) {
		D = *dp;
	}
	else {
		D.kP.COM = calculateUniformCOM(c);
		D.TOI = calculateUniformInertia(c, 1, D.kP.COM);
	}
	staticDP.push_back(D);
}

collider* physicalWorld::getBodyCol(size_t index) { return colls[index]; }
collider* physicalWorld::getStaticBodyCol(size_t index) { return staticColls[index]; }
dynamicProperties* physicalWorld::getBodyDP(size_t index) { return & DP[index]; }
dynamicProperties* physicalWorld::getStaticBodyDP(size_t index) { return &staticDP[index]; }
size_t physicalWorld::noBodies() { return colls.size(); }
size_t physicalWorld::noStaticBodies() { return staticColls.size(); }



void physicalWorld::updatePositions() {
	for (size_t i = 0; i < colls.size(); ++i) {
		//DP[i].kP.COM = colls[i]->M.CS.getOrigin();
		DP[i].kP.COM += DP[i].kP.vel * (*deltaTime);
		colls[i]->MR.CS.addRotationAboutAxis(DP[i].kP.angularVel * (*deltaTime));
		colls[i]->M.CS.addRotationAboutAxis(DP[i].kP.angularVel * (*deltaTime));
		colls[i]->MT.CS.setOrigin(colls[i]->MT.CS.getOrigin()+ (DP[i].kP.vel * (*deltaTime)));
		colls[i]->M.CS.setOrigin(colls[i]->M.CS.getOrigin()+ (DP[i].kP.vel * (*deltaTime)));
		colls[i]->MR.update();
		colls[i]->MT.update();
		colls[i]->M.update();
	}
	for (size_t i = 0; i < staticColls.size(); ++i) {
		staticDP[i].kP.COM = staticColls[i]->M.CS.getOrigin();
		staticDP[i].kP.COM += staticDP[i].kP.vel * (*deltaTime);
		staticColls[i]->MR.CS.addRotationAboutAxis(staticDP[i].kP.angularVel * (*deltaTime));
		staticColls[i]->M.CS.addRotationAboutAxis(staticDP[i].kP.angularVel * (*deltaTime));
		staticColls[i]->MT.CS.setOrigin(staticDP[i].kP.COM);
		staticColls[i]->M.CS.setOrigin(staticDP[i].kP.COM);
		staticColls[i]->MR.update();
		staticColls[i]->MT.update();
		staticColls[i]->M.update();
	}
}

void physicalWorld::applyGlobalForce() {
	for (size_t i = 0; i < DP.size(); ++i) {
		DP[i].kP.vel += globalForce * (*deltaTime);
	}
}

void physicalWorld::update() {
	applyGlobalForce();
	//update initia pos
	updatePositions();
	
	//calculate collisions

	//static dynamic collisions
	for (size_t i = 0; i < staticColls.size(); ++i) {
		for (size_t j = 0; j < colls.size(); ++j) {
			//check for collision
			if (colliding(staticColls[i], colls[j])) {
				//resolve collision
				//get resolving axis
				vec3d axis = staticDP[i].kP.vel-DP[j].kP.vel;
				if (vec3d::isNUL(axis))axis = DP[j].kP.COM - staticDP[i].kP.COM;
				if (vec3d::isNUL(axis))axis = vec3d(1, 0, 0);
				vec3d finalSep;
				vec3d collPt = separateTillLastColl(staticColls[i], colls[j], axis, finalSep);
				//calculate reaction
				calculateNewVel(staticColls[i], colls[j], &staticDP[i], &DP[j], collPt, true);
				//final separation
				performLastSep(colls[j],finalSep);
				std::cout << "static coll Detected : " << i << " , " << j << std::endl;
			}
		}
	}

	//dynamic dynamic collisions
	for (size_t i = 0; i < colls.size(); ++i) {
		for (size_t j = i + 1; j < colls.size(); ++j) {
			if (colliding(colls[i], colls[j])) {
				//resolve collision
				//get resolving axis
				vec3d axis = DP[i].kP.vel - DP[j].kP.vel;
				if (vec3d::isNUL(axis))axis = DP[j].kP.COM - DP[i].kP.COM;
				if (vec3d::isNUL(axis))axis = vec3d(1, 0, 0);
				vec3d finalSep;
				vec3d collPt = separateTillLastColl(colls[i], colls[j], axis, finalSep);
				//calculate reaction
				calculateNewVel(colls[i], colls[j], &DP[i], &DP[j], collPt, false);
				//final separation
				performLastSep(colls[j], finalSep);
				std::cout << "dynamic coll Detected : " << i << " , " << j << std::endl;
			}
		}
	}
}