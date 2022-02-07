#pragma once

#include <list>
#include <set>
#include <Eigen/Dense>
#include <Eigen/Sparse>

class ARAP
{
public:
	ARAP(Eigen::MatrixXd &vertices, Eigen::MatrixXi &faces);

	Eigen::MatrixXd deform(std::set<int> &fixedPoints, std::set<int> &handles, Eigen::Vector3d &move);
	Eigen::MatrixXd getWeightMatrix();
	Eigen::MatrixXd getSystemMatrix();

private:
	Eigen::MatrixXd vertices;
	const Eigen::MatrixXi faces;
	const size_t vertCount;
	const size_t faceCount;

	std::vector<int> constraints;	// fixed vertrices + fixed handles + their neighbors
	std::vector<int> freeVariables; // contains all unfixed variables
	Eigen::MatrixXd weightMatrix;
	std::vector<std::vector<int>> neighborsMatrix;
	std::vector<Eigen::Matrix3d> cellRotations;

	Eigen::MatrixXd systemMatrix;
	Eigen::MatrixXd bMatrix;
	// Original system matrix from the initialisation of a deform step.
	Eigen::MatrixXd initialSystemMatrix;

	Eigen::SparseMatrix<double> sparseMatrix;
	std::vector<Eigen::MatrixXd> guessedP;

	// Displaced p_prime vertices (in the equation system)
	Eigen::MatrixXd displacedVertices;

	Eigen::MatrixXi computeNeighborAdjacencyMatrix();
	std::vector<std::vector<int>> createNeighbors(Eigen::MatrixXi &neighbor_adjacency);
	void fillConstraints(std::set<int> &fixedPoints, std::set<int> &handles);
	void fillFreeVariables();
	Eigen::MatrixXd buildWeightMatrix();
	void buildSystemMatrix();
	void updateSystemMatrix(std::set<int> &fixedPoints, std::set<int> &handles);
	void calcB(std::set<int> &fixedPoints, std::set<int> &handles, Eigen::Vector3d &move);
	void estimateRotation();
	Eigen::Matrix3d estimateRotationForCell(int vertexId);
	Eigen::MatrixXd estimateTranslation();
	double computeEnergy();
};
