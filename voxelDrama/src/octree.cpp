#pragma once
#include "./../headers/octree.h"

template<typename T>
ocNode<T>::ocNode() {
	for (unsigned char i = 0; i < 8; ++i)
		nodes[i] = nullptr;
}

template<typename T>
void ocNode<T>::deleteTree(){
	for (unsigned char i = 0; i < 8; ++i) {
		if (nodes[i] != nullptr)
		{
			delete nodes[i];
		}
	}	
}

template<typename T>
ocNode<T>::~ocNode() {
	deleteTree();
}

unsigned char calcTreePos(bool x, bool y, bool z) {
	unsigned char rVal = 0;
	if (x)rVal |= 1 << 0;
	if (y)rVal |= 1 << 1;
	if (z)rVal |= 1 << 2;
	return rVal;
}