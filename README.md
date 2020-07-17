# voxelDrama
A basic physics engine written in cpp
![logo](https://raw.githubusercontent.com/Kharon4/voxelDrama/master/branding/voxelDramaLogo.png)

## voxelDrama is deterministic 3D physics engine written in cpp. Targeted at windows x86 , x64 platforms.
*c++ standard library is the only external dependency for the project hence it can be compiled for virtually any platform.*

### Current features :
- [x] Multiple types of colliders for collision detection
- [x] Automatic calculation of physical properties
	- Center of mass
	- Tensor of Inertia and its inverse
- [x] Inbuilt system for managing vectors other than the ones required for collider
- [x] Basic physics simulation of rigid bodies
	- Collision Detection (discrete space, discrete time) 
	- Collision Reaction
	- Collision separation
- [x] Priority based system for determination of collision properties

### Future Goals :
- [ ] Continuous time collision detection
- [ ] Implementation of joints
- [ ] Arbitrary constraints
- [ ] Hardware acceleration using CUDA


## Powered by [math3D v1.0-16.7.20](https://github.com/Kharon4/math3D/releases/tag/v1.0-16.7.20)
![math3D logo](https://raw.githubusercontent.com/Kharon4/math3D/master/branding/logo.png)

## Tested With [surrealRT v2.1-16.7.20](https://github.com/Kharon4/surrealRT/releases/tag/v2.1-16.7.20)

 Older versions Tested using [consoleGraphics](https://github.com/Kharon4/consoleGraphics#consolegraphics)
![consoleGraphics logo](https://raw.githubusercontent.com/Kharon4/consoleGraphics/master/branding/logo.PNG)

