#include <iostream>
#include <string>
#include "collisionDetection.h"
#include "profiler.h"
//#include "consoleGraphics/renderer.h"
//#include "consoleGraphics/consoleController.h"
//#include "consoleGraphics/input.h"
#include "win32WindowingSystem.h"
#include "rendering.cuh"
#include "parsing/parsingAlgos/obj.h"

#include "VoxelDrama_Core.h"

//#define tostring(x) #x
//#define objLocation(x) tostring(./../submodules/surrealRT/test/res/x.obj)

bool updateCam(manipulation3d::transformf& t, manipulation3d::transformf& tNr) {
	input::update();
	double speed = 0.1;
	double rSpeed = 0.001;
	if (input::pressed['X'])return false;
	if (input::isDown[VK_SPACE])speed *= 4;
	if (input::isDown['E']) {
		t.CS.addRelativePos(vec3d(0, 0, speed));
	}
	if (input::isDown['Q']) {
		t.CS.addRelativePos(vec3d(0, 0, -speed));
	}
	if (input::isDown['W']) {
		t.CS.addRelativePos(vec3d(speed, 0, 0));
	}
	if (input::isDown['S']) {
		t.CS.addRelativePos(vec3d(-speed, 0, 0));
	}
	if (input::isDown['D']) {
		t.CS.addRelativePos(vec3d(0, -speed, 0));
	}
	if (input::isDown['A']) {
		t.CS.addRelativePos(vec3d(0, speed, 0));
	}
	if (input::pressed['M']) {
		input::lockX = input::mouseX;
		input::lockY = input::mouseY;
		input::lock = !input::lock;
	}
	if (input::lock){
		input::hideCursor();
		vec3d rotationStep(-input::changeX, -input::changeY, 0);
		t.CS.setAngle(t.CS.getAngle() + (rotationStep * rSpeed));
		tNr.CS.setAngle(tNr.CS.getAngle() + (rotationStep * rSpeed));
	}
	input::showCursor();
	t.update();
	tNr.update();
	return true;
}


int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow){
	enableConsole();
	unsigned short xRes=720, yRes=480;
	camera cam(vec3f(0,-1,0),xRes,yRes,vec3f(0,0,0),vec3f(1,0,0),vec3f(0,0,((float)yRes)/xRes));
	window dWindow(hInstance,nCmdShow,L"voxelDrama",xRes,yRes);
	manipulation3d::transformf t;
	manipulation3d::transformf tNr;

	t.CS.setOrigin(cam.vertex);
	t.CS.setAngle(vec3d(math3D_pi / 2, 0, 0));
	tNr.CS.setAngle(vec3d(math3D_pi / 2, 0, 0));

	t.addVec(cam.sc.screenCenter, &cam.sc.screenCenter);
	t.addVec(cam.vertex, &cam.vertex);
	tNr.addVec(cam.sc.halfUp, &cam.sc.halfUp);
	tNr.addVec(cam.sc.halfRight, &cam.sc.halfRight);
	
	//setup colliders
	double scale = 2; //dia
	//sphereCollider c1(scale / 2, vec3d(0, 0, 0));
	cuboidCollider c1(vec3d(0, 0, 0), vec3d(scale, scale, scale));
	sphereCollider c2(scale / 2 ,vec3d(0, 0, 10));

	//graphical setup
	const unsigned int estimatedNoFaces = 500;
	commonMemory<meshShaded> Mesh(estimatedNoFaces);
	loadBlankModel(Mesh.getHost(), estimatedNoFaces);
	long int totalFacesLoaded = 0;

	//load graphical bodies
	shadedSolidColCPU obj1Shader(color(125, 10, 10), color(0.75, 0.25, 0.25), vec3f(-1, 0, 0));
	shadedSolidColCPU obj2Shader(color(10, 125, 125), color(0.75, 0.25, 0.25), vec3f(-1, 0, 0));
	long int facesLoadedB1 = loadModel(Mesh.getHost() + totalFacesLoaded , estimatedNoFaces - totalFacesLoaded, "./../submodules/surrealRT/test/res/cube.obj",obj1Shader.getGPUPtr(),loadAxisExchange::xzy);
	if (facesLoadedB1 < 0) { std::cout << "error loading Model\n"; system("pause"); return 0; }
	else totalFacesLoaded += facesLoadedB1;
	vec3d* sp1V = new vec3d[3 * facesLoadedB1];//sphere vertices
	for (long int i = 0; i < facesLoadedB1; ++i) {
		c1.M.addVec(Mesh.getHost()[i].M.pts[0], sp1V + i * 3);
		c1.M.addVec(Mesh.getHost()[i].M.pts[1], sp1V + i * 3+1);
		c1.M.addVec(Mesh.getHost()[i].M.pts[2], sp1V + i * 3+2);
	}
	c1.M.update();
	long int facesLoadedB2 = loadModel(Mesh.getHost() + totalFacesLoaded, estimatedNoFaces - totalFacesLoaded, "./../submodules/surrealRT/test/res/icoSphere.obj", obj2Shader.getGPUPtr(), loadAxisExchange::xzy);
	if (facesLoadedB2 < 0) { std::cout << "error loading Model\n"; system("pause"); return 0; }
	else totalFacesLoaded += facesLoadedB2;
	vec3d* sp2V = new vec3d[3 * facesLoadedB2];//sphere vertices
	for (long int i = 0; i < facesLoadedB2; ++i) {
		c2.M.addVec(Mesh.getHost()[i + facesLoadedB1].M.pts[0] + c2.M.CS.getOrigin(), sp2V + i * 3);
		c2.M.addVec(Mesh.getHost()[i + facesLoadedB1].M.pts[1] + c2.M.CS.getOrigin(), sp2V + i * 3 + 1);
		c2.M.addVec(Mesh.getHost()[i + facesLoadedB1].M.pts[2] + c2.M.CS.getOrigin(), sp2V + i * 3 + 2);
	}
	c2.M.update();
	//graphical world setup
	graphicalWorldADV world(&Mesh,xRes,yRes,3,3);
	
	//physical world setup
	physicalWorld pWorld;
	pWorld.deltaTime = &(input::deltaTime);
	pWorld.globalForce = vec3d(0, 0, -9.8);
	pWorld.addStaticBody(&c1);
	//pWorld.getBodyDP(0)->kP.vel = vec3d(0, 0, 1);
	//pWorld.getStaticBodyDP(0)->mass = 2;
	pWorld.getStaticBodyDP(0)->kP.angularVel = vec3d(0, 0, 1);
	
	pWorld.addBody(&c2);
	pWorld.getBodyDP(0)->kP.angularVel = vec3d(0, 0, 0);
	//pWorld.getBodyDP(0)->kP.vel = vec3d(0, 0, -5);

	pWorld.getBodyDP(0)->pMat.coeffRestitution = 1;
	pWorld.getStaticBodyDP(0)->pMat.coeffRestitution = 0;
	
	

	std::cout << "total faces loaded = " << totalFacesLoaded << " / " << estimatedNoFaces << std::endl;
	
	input::asyncGetch();
	//loop
	while (!dWindow.isWindowClosed() && updateCam(t, tNr)) {
		world.render(cam, dWindow.data, [&dWindow]() {dWindow.update(); });
		pWorld.update();
		for (long int i = 0; i < facesLoadedB1; ++i) {
			Mesh.getHost()[i].M.pts[0] = sp1V[3 * i + 0];
			Mesh.getHost()[i].M.pts[1] = sp1V[3 * i + 1];
			Mesh.getHost()[i].M.pts[2] = sp1V[3 * i + 2];
		}
		for (long int i = 0; i < facesLoadedB2; ++i) {
			Mesh.getHost()[i + facesLoadedB1].M.pts[0] = sp2V[3 * i + 0];
			Mesh.getHost()[i + facesLoadedB1].M.pts[1] = sp2V[3 * i + 1];
			Mesh.getHost()[i + facesLoadedB1].M.pts[2] = sp2V[3 * i + 2];
		}

	}
	

	delete[] sp1V;
	delete[] sp2V;
	return 0;
}