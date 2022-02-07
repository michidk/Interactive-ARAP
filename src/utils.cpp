#include <iostream>
#include "utils.h"

// returns the closest vertex of the triangle to the baycentric coordinates bc
int getClosestVertex(Eigen::MatrixXi triangle, Eigen::Vector3f &bc)
{
	if (bc(0) > bc(1) && bc(0) > bc(2))
	{
		return triangle(0);
	}
	else if (bc(1) > bc(0) && bc(1) > bc(2))
	{
		return triangle(1);
	}
	else if (bc(2) > bc(1) && bc(2) > bc(0))
	{
		return triangle(2);
	}
	else
	{
		std::cout << "couldnt decide for vertex" << std::endl;
		return triangle(1);
	}
}

/**
 * Print information about input modes
 */
void printUserInfo()
{
	std::cout << "\nFollowing interaction Modes are available:\n";
	std::cout << "\t[key]\t[action]\n";
	PRINTINFO("1", "Toggle fixed vertices selection mode. Activated at start of application")
	PRINTINFO("2", "Toggle fixed faces selection mode.")
	PRINTINFO("3", "Toggle handle vertices selection mode.")
	PRINTINFO("4", "Toggle ARAP deformation mode. Move selected handle by dragging to mouse position while left MB is pressed.")
	PRINTINFO("5", "Toggle viewer only mode. Rotate object only.")
	std::cout << std::endl;
}

double computeAngleBetween(Eigen::Vector3d a, Eigen::Vector3d b)
{
	auto c = a.cross(b).norm();
	return atan2f(c, a.dot(b));
}
