#pragma once

enum class treePos {
	xyz = 0,
	Xyz = 1,
	xYz = 2,
	XYz = 3,
	xyZ = 4,
	XyZ = 5,
	xYZ = 6,
	XYZ = 7
};

unsigned char calcTreePos(bool x, bool y, bool z);

template <typename T>
class ocNode {//octree node
private:
	void deleteTree();
public:
	T data;
	ocNode<T>* nodes[8];
	ocNode();
	~ocNode();
};