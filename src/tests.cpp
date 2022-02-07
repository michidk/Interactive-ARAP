#define CATCH_CONFIG_MAIN

#include <catch.hpp>
#include <arap.h>
#include <igl/readOFF.h>

TEST_CASE("Check Weight matrix", "[weight-matrix-check][simple-two-faces]")
{
	/* Comparison between the two methods for computing the weights:
		no voronoi
		0, 0.5, -4.37114e-08, 0.5
		0.5, 0, 0.5, 0
		-4.37114e-08, 0.5, 0, 0.5
		0.5, 0, 0.5, 0

		with voronoi					<--- more accurate
		0, 0.5, -2.22045e-16, 0.5
		0.5, 0, 0.5, 0
		-2.22045e-16, 0.5, 0, 0.5
		0.5, 0, 0.5, 0
	 */
	// build 2 faces and compute cotan within
	Eigen::MatrixXd vertices(4,3);
	Eigen::MatrixXi faces(2, 3);
	Eigen::MatrixXd solution(4, 4);
	solution.row(0) = Eigen::Vector4d(0, 0.5, 0, 0.5);
	solution.row(1) = Eigen::Vector4d(0.5, 0, 0.5, 0);
	solution.row(2) = Eigen::Vector4d(0, 0.5, 0, 0.5);
	solution.row(3) = Eigen::Vector4d(0.5, 0, 0.5, 0);
	vertices.row(0) = Eigen::Vector3d(0,0,0);
	vertices.row(1) = Eigen::Vector3d(1,0,0);
	vertices.row(2) = Eigen::Vector3d(1,1,0);
	vertices.row(3) = Eigen::Vector3d(0,1,0);
	faces.row(0) = Eigen::Vector3i(0,1,2);
	faces.row(1) = Eigen::Vector3i(0,2,3);

	// initializing also creates the weights matrix
	ARAP *arap = new ARAP(vertices, faces);
	Eigen::MatrixXd weights = arap->getWeightMatrix();
	for (int i = 0; i < 4; i++){
		std::cout << weights(i,0) << ", " << weights(i,1) << ", " << weights(i,2) << ", " << weights(i,3) << std::endl;
	}
	REQUIRE(weights.isApprox(solution, 1e-4));
}

TEST_CASE("Check System matrix", "[system-matrix-check]")
{
	Eigen::MatrixXd vertices(4,3);
	Eigen::MatrixXi faces(2, 3);
	Eigen::MatrixXd solution(4, 4);
	vertices.row(0) = Eigen::Vector3d(0,0,0);
	vertices.row(1) = Eigen::Vector3d(1,0,0);
	vertices.row(2) = Eigen::Vector3d(1,1,0);
	vertices.row(3) = Eigen::Vector3d(0,1,0);
	faces.row(0) = Eigen::Vector3i(0,1,2);
	faces.row(1) = Eigen::Vector3i(0,2,3);

	//system matrix contains weights
	solution.row(0) = Eigen::Vector4d(1,-0.5, 0, -0.5);
	solution.row(1) = Eigen::Vector4d(-0.5,1, -0.5, 0);
	solution.row(2) = Eigen::Vector4d(0,-0.5, 1, -0.5);
	solution.row(3) = Eigen::Vector4d(-0.5,0, -0.5, 1);

	ARAP *arap = new ARAP(vertices, faces);
	Eigen::MatrixXd system = arap->getSystemMatrix();
	for (int i = 0; i < 4; i++){
		std::cout << system(i,0) << ", " << system(i,1) << ", " << system(i,2) << ", " << system(i,3) << std::endl;
	}
	REQUIRE(system.isApprox(solution, 1e-4));
}
