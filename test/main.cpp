#include <iostream>
#include <string>
#include "collisionDetection.h"
#include "profiler.h"
#include "math3D/rotation.h"
#include "consoleGraphics/renderer.h"
#include "consoleGraphics/consoleController.h"
#include "consoleGraphics/input.h"
#include "physicalProperties.h"
solidCharShader defaultShader(color::getColor(0.5,0,0.5));
solidCharShader collpt(color::getColor(1, 0, 0));



void updateCam(manipulation3dD::transform& t, manipulation3dD::transform& tNr) {
	input::get()->update();
	double speed = 0.1;
	double rSpeed = 0.001;
	if (input::get()->isDown['E']) {
		t.addRelativePos(vec3d(0, 0, speed));
	}
	if (input::get()->isDown['Q']) {
		t.addRelativePos(vec3d(0, 0, -speed));
	}
	if (input::get()->isDown['W']) {
		t.addRelativePos(vec3d(speed, 0, 0));
	}
	if (input::get()->isDown['S']) {
		t.addRelativePos(vec3d(-speed, 0, 0));
	}
	if (input::get()->isDown['D']) {
		t.addRelativePos(vec3d(0, -speed, 0));
	}
	if (input::get()->isDown['A']) {
		t.addRelativePos(vec3d(0, speed, 0));
	}
	if (input::get()->pressed['M']) {
		input::get()->lockX = input::get()->mouseX;
		input::get()->lockY = input::get()->mouseY;
		input::get()->lock = !input::get()->lock;
	}
	if (input::get()->lock){
		input::get()->hideCursor();
		vec3d rotationStep(-input::get()->changeX, -input::get()->changeY, 0);
		t.CS.setAngle(t.CS.getAngle() + (rotationStep * rSpeed));
		tNr.CS.setAngle(tNr.CS.getAngle() + (rotationStep * rSpeed));
	}
	input::get()->showCursor();
	t.update();
	tNr.update();
}

int main() {
	startProfiling();

	//setup graphics
	cam camera;
	camera.sc.xPixels = 200;
	camera.sc.yPixels = 64;

	manipulation3dD::transform t;
	manipulation3dD::transform tNr;

	t.CS.setOrigin(camera.vertex);
	t.CS.setAngle(vec3d(manipulation3dD::pi / 2, 0, 0));
	tNr.CS.setAngle(vec3d(manipulation3dD::pi / 2, 0, 0));

	t.addVec(camera.sc.TopLeft, &camera.sc.TopLeft);
	t.addVec(camera.vertex, &camera.vertex);
	tNr.addVec(camera.sc.down, &camera.sc.down);
	tNr.addVec(camera.sc.right, &camera.sc.right);
	std::vector<std::vector<CHAR_INFO>>data;
	{
		mesh::triangle T(vec3d(0, 0, 0), vec3d(10, 0, 0), vec3d(10, 10, 0), &defaultShader);
		_globalWorld.triangles.push_back(T);
	}
	{
		mesh::triangle T(vec3d(0, 0, 0), vec3d(0, 10, 0), vec3d(10, 10, 0), &defaultShader);
		_globalWorld.triangles.push_back(T);
	}
	consoleController::get(camera.sc.xPixels, camera.sc.yPixels, L"ConsoleGraphics - by AbhishekKhurana");
	
	//setup colliders
	cuboidCollider c1(vec3d(0, 0, 0), vec3d(10, 10, 10));
	sphereCollider c2(10,vec3d(10, 0, 0));
	vec3d COM = calculateUniformCOM(&c2);
	std::cout << COM.x << " , " << COM.y << " , " << COM.z<<std::endl;
	tensorOfInertia TOI;
	TOI = calculateUniformInertia(&c2, 1, COM);
	std::cout << TOI.IX.x << " , " << TOI.IX.y << " , " << TOI.IX.z << std::endl;
	std::cout << TOI.IY.x << " , " << TOI.IY.y << " , " << TOI.IY.z << std::endl;
	std::cout << TOI.IZ.x << " , " << TOI.IZ.y << " , " << TOI.IZ.z << std::endl;

	std::cout << colliding(&c1, &c2)<<std::endl;
	std::cout << getAColPT(&c1, &c2).x << " , " << getAColPT(&c1, &c2).y <<" , "<< getAColPT(&c1, &c2).z << std::endl;
	system("pause");

	//loop
	while (1) {
		data = _globalWorld.render(camera.sc, camera.vertex, color::getColor(0, 0, 0));
		consoleController::get()->draw(&data);
		updateCam(t, tNr);
	}
	
	endProfiling();
	system("pause");
	return 0;
}