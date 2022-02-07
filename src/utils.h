#pragma once

#include <Eigen/Dense>

#define PRINT(x) std::cout << x << std::endl;
#define CONTAINS(x, y) (x.find(y) != x.end())
#define PRINTINFO(key, info) (std::cout << "\t[" << key "]" \
                                        << "\t\t" << info << "\n");

int getClosestVertex(Eigen::MatrixXi triangle, Eigen::Vector3f &bc);
void printUserInfo();
double computeAngleBetween(Eigen::Vector3d a, Eigen::Vector3d b);
