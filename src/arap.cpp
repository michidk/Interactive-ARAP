#include <Eigen/StdVector>
#include <Eigen/Eigenvalues>
#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>

#include <igl/point_mesh_squared_distance.h>

#include <list>
#include <set>

#include "arap.h"
#include "utils.h"

#define MEASURE_PERF false		 // turn on performance measurments
#define USE_OPENMP true			 // use OpenMP (faster)
#define SPARSE true				 // use sparse matrices (faster)
#define VORONOI_WEIGHTS true	 // computing weights using voronoi area is more accurate
#define FILTER_SMALL_VALUES true // "remove" very small values caused by computational errors from equation (required for unit testing)
#define ITERATIONS 40			 // ARAP interations
#define ENERGY_THRESHOLD 1e-1	 // early stopping using energy threshold

#if MEASURE_PERF
#include <chrono>
#endif

ARAP::ARAP(Eigen::MatrixXd &vertices, Eigen::MatrixXi &faces)
	: vertices(vertices), faces(faces), faceCount(faces.rows()), vertCount(vertices.rows())
{
	PRINT("Initializing ARAP")

#if MEASURE_PERF
	std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
#endif

	cellRotations = std::vector<Eigen::Matrix3d>(vertices.rows());
	bMatrix = Eigen::MatrixXd::Zero((long)vertCount, 3);
	displacedVertices = vertices.replicate<1, 1>();

	Eigen::MatrixXi adjacencyMatrix = computeNeighborAdjacencyMatrix();
	neighborsMatrix = createNeighbors(adjacencyMatrix);

	weightMatrix = buildWeightMatrix();
	buildSystemMatrix();

#if MEASURE_PERF
	std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
	std::cout << "\tInitialization time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << " ms" << std::endl;
#endif

	PRINT("ARAP initialized")
}

/**
 * Compute a as rigid as possible deformation.
 * @param fixedPoints fixed handles
 * @param handle selected movable handle ID, in moved position (moved by the user)
 * @param handlePosition position of the movable handle
 * @return deformed vertices
 */
Eigen::MatrixXd ARAP::deform(std::set<int> &fixedPoints, std::set<int> &handles, Eigen::Vector3d &move)
{
#if MEASURE_PERF
	std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
#endif

	displacedVertices = vertices.replicate<1, 1>();
	// We want to add our displaced positions from the moved handle into this matrix
	for (int handle : handles)
	{
		Eigen::Vector3d handlePos = vertices.row(handle);
		displacedVertices.row(handle) = handlePos - move;
	}

	fillConstraints(fixedPoints, handles);
	fillFreeVariables();
	updateSystemMatrix(fixedPoints, handles);

	double ePrev = 10;
	Eigen::MatrixXd result;
	for (auto i = 0; i < ITERATIONS; i++)
	{
		PRINT("ARAP iteration " << i)
		estimateRotation();				   //assume position is fixed and compute rotation
		calcB(fixedPoints, handles, move); //assume rotation is fixed and compute position
		result = estimateTranslation();

		double e = computeEnergy();
		if (std::abs((ePrev - e)) < ENERGY_THRESHOLD)
		{
			std::cout << "Current Energy: " << e << " Prev. Energy: " << ePrev << " diff: " << (ePrev - e) << std::endl;
			break;
		}
		ePrev = e;
	}

	// calculate movement amount (just debug info)
	double displacement = 0.0;
#if USE_OPENMP
#pragma omp parallel for default(none) shared(result) reduction(+ \
																: displacement)
#endif
	for (auto i = 0; i < vertCount; i++)
	{
		Eigen::Vector3d newPos = result.row(i);
		Eigen::Vector3d oldPos = vertices.row(i);
		displacement += (newPos - oldPos).norm();
	}
	PRINT("TOTAL DISPLACEMENT: " << displacement)
	this->vertices = result;
#if MEASURE_PERF
	std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
	std::cout << "\tDeform time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << " ms" << std::endl;
#endif
	return result;
}

/**
 * Compute the neighbor matrix
 * @return Adjacency Matrix with values in [0,1] set to 1 where two vertices are connected.
 */
Eigen::MatrixXi ARAP::computeNeighborAdjacencyMatrix()
{
	// Get direct neighbors of one vertex. Store that in matrix containing vertices as col and neighbor-vertices as row
	Eigen::MatrixXi neighbor_adjacency(vertCount, vertCount);
#if USE_OPENMP
#pragma omp parallel for default(none) shared(neighbor_adjacency)
#endif
	for (int i = 0; i < faces.rows(); i++)
	{
		Eigen::MatrixXi tri = faces.row(i);
		int v_1 = tri(0);
		int v_2 = tri(1);
		int v_3 = tri(2);
		// adjacency is shown in rows where it is set to 1
		neighbor_adjacency(v_1, v_2) = 1;
		neighbor_adjacency(v_1, v_3) = 1;
		neighbor_adjacency(v_2, v_1) = 1;
		neighbor_adjacency(v_2, v_3) = 1;
		neighbor_adjacency(v_3, v_1) = 1;
		neighbor_adjacency(v_3, v_2) = 1;
	}
	return neighbor_adjacency;
}

/**
 * Creates the neighbor matrix
 * @param vertices
 * @param neighbor_adjacency
 * @return 2-dim vector: contains per VertexID a vector with its neighbors (as VertexIDs). Access Neighbors via neighbors(vertexID).
 */
std::vector<std::vector<int>> ARAP::createNeighbors(Eigen::MatrixXi &neighbor_adjacency)
{
	// adjacency matrix for neighboring vertices. row/col is the index of a vertex in the matrix vertices.
	// fill in vector with neighbors in there
	std::vector<std::vector<int>> neighbors;
	neighbors.resize(vertCount);
#if USE_OPENMP
#pragma omp parallel for default(none) shared(neighbors, neighbor_adjacency)
#endif
	for (int i = 0; i < vertCount; i++)
	{
		for (int j = 0; j < vertCount; j++)
		{
			if (neighbor_adjacency(i, j) == 1)
			{
				neighbors[i].push_back(j); // add vertex id
			}
		}
	}
	return neighbors;
}

/**
 * Add handles, fixed points and neighbors to constraints vector.
 * @param fixedPoints
 * @param handles
 */
void ARAP::fillConstraints(std::set<int> &fixedPoints, std::set<int> &handles)
{
	constraints = std::vector<int>();
	constraints.reserve(handles.size() + fixedPoints.size());
	for (auto handle : handles)
	{
		constraints.push_back(handle);
		// we don't want to fix the neighbors here, as they will stay in unmoved position in our implementation.
	}
	for (auto fixed : fixedPoints)
	{
		constraints.push_back(fixed);
		for (auto neighbor : neighborsMatrix[fixed])
		{
			constraints.push_back(neighbor);
		}
	}
	std::sort(constraints.begin(), constraints.end());
}

/**
 * Fill in all free variables that are not in constraints
 */
void ARAP::fillFreeVariables()
{
	freeVariables = std::vector<int>();
	int constraints_begin = constraints[0];
	freeVariables.reserve(constraints_begin);
	for (int vID = 0; vID < vertCount; vID++)
	{
		// constraints are sorted, so all lower ids than the beginning one can be inserted directly
		if (vID < constraints_begin)
		{
			freeVariables.push_back(vID);
			continue;
		}
		if (std::find(constraints.begin(), constraints.end(), vID) != constraints.end())
		{
			// value is constraint
			continue;
		}
		freeVariables.push_back(vID);
	}
}

/**
 * Weight Computation using cotan. Cotan is computed over the area of the face.
 * w_ij = 0.5 * (cot(a) + cot(b))
 * @return sparse weight Matrix where weights are stored per edge (i,j) = (j,i)
 */
Eigen::MatrixXd ARAP::buildWeightMatrix()
{
	// basically do a laplace beltrami cotan step
	// Code inspiration: https://github.com/alfonsoros88/PCLBO/blob/cdfcd845795b64ee84d6f7d8c89c553f4f90054e/src/meshlbo.cpp#L32
	Eigen::MatrixXd weights(vertCount, vertCount);
	weights.setZero();
	// compute 1/2 cot(a) per edge in triangle and add that up. Weight is only stored per edge -> edge covers 2 triangles.
	// Always add up 2 cots per edge
#if USE_OPENMP
#pragma omp parallel for default(none) shared(weights)
#endif
	for (int i = 0; i < faceCount; i++)
	{
		int v1ID = faces(i, 0);
		int v2ID = faces(i, 1);
		int v3ID = faces(i, 2);
		// Get vertices (x,y,z)
		Eigen::Vector3d v1 = vertices.row((v1ID));
		Eigen::Vector3d v2 = vertices.row((v2ID));
		Eigen::Vector3d v3 = vertices.row((v3ID));

		double alpha, beta, gamma, cot_1_2, cot_2_3, cot_1_3, diag;
#if VORONOI_WEIGHTS
		// Computation of weights implementation oriented at:
		// Mario Botsch and Olga Sorkine. "On Linear Variational Surface Deformation Methods "
		// https://igl.ethz.ch/projects/deformation-survey/

		// compute squared norm of edges
		double norm1 = std::sqrt((v2 - v1).squaredNorm());
		double norm2 = std::sqrt((v3 - v2).squaredNorm());
		double norm3 = std::sqrt((v1 - v3).squaredNorm());

		// compute area
		double s = 0.5 * (norm1 + norm2 + norm3);
		double area = std::sqrt(s * (s - norm1) * (s - norm2) * (s - norm3));
		area = 1.0 / (4.0 * area);
		diag = area;
		// compute cot
		alpha = (-norm1 * norm1 + norm2 * norm2 + norm3 * norm3) * area;
		beta = (norm1 * norm1 - norm2 * norm2 + norm3 * norm3) * area;
		gamma = (norm1 * norm1 + norm2 * norm2 - norm3 * norm3) * area;

		cot_1_2 = 0.5 * alpha;
		cot_2_3 = 0.5 * beta;
		cot_1_3 = 0.5 * gamma;
#else
		// other approach, not based tan: Has little less accuracy than voronoi.
		diag = 1;

		beta = computeAngleBetween(v3 - v1, v2 - v1);
		gamma = computeAngleBetween(v1 - v2, v3 - v2);
		alpha = computeAngleBetween(v2 - v3, v1 - v3);

		// compute cotan  1/tan(a) * 0.5 = 0.5/tan(a)
		cot_1_2 = 0.5 / tan(alpha);
		cot_2_3 = 0.5 / tan(beta);
		cot_1_3 = 0.5 / tan(gamma);
#endif

#if FILTER_SMALL_VALUES
		//filter for small error values to set them to 0
		if (cot_1_2 < 1e-8)
			cot_1_2 = 0;
		if (cot_1_3 < 1e-8)
			cot_1_3 = 0;
		if (cot_2_3 < 1e-8)
			cot_2_3 = 0;
#endif
		// Setting the diagonal values is required for energy computation.
		weights(v1ID, v1ID) += diag;
		weights(v2ID, v2ID) += diag;
		weights(v3ID, v3ID) += diag;

		// write weights symmetrical in edges matrix
		weights(v1ID, v2ID) += cot_1_2;
		weights(v2ID, v1ID) += cot_1_2;

		weights(v2ID, v3ID) += cot_2_3;
		weights(v3ID, v2ID) += cot_2_3;

		weights(v1ID, v3ID) += cot_1_3;
		weights(v3ID, v1ID) += cot_1_3;
	}
	return weights;
}

/**
 * initial calculation of lefthand side of LES
 */
void ARAP::buildSystemMatrix()
{
	systemMatrix = Eigen::MatrixXd::Zero(vertCount, vertCount);
#if USE_OPENMP
#pragma omp parallel for default(none)
#endif
	for (auto vertexId = 0; vertexId < vertCount; vertexId++)
	{
		for (int neighbor : neighborsMatrix[vertexId])
		{
			systemMatrix(vertexId, vertexId) += weightMatrix(vertexId, neighbor);
			systemMatrix(vertexId, neighbor) = -weightMatrix(vertexId, neighbor);
		}
	}
	initialSystemMatrix = systemMatrix.replicate(1, 1);
#if SPARSE
	sparseMatrix = systemMatrix.sparseView();
#endif
}

/**
 * iteration update of system matrix.
 * Filter out all handles and fixed vertices and put them into the right hand side of LES.
 * @param fixedPoints
 * @param handle
 * @param handlePosition
 */
void ARAP::updateSystemMatrix(std::set<int> &fixedPoints, std::set<int> &handles)
{
	// reset system matrix
	systemMatrix = initialSystemMatrix;
	//filter constrained vertices (handles, fixed vertices)
#if USE_OPENMP
#pragma omp parallel for default(none)
#endif
	for (auto constraint : constraints)
	{
		systemMatrix.row(constraint).setZero();
		systemMatrix(constraint, constraint) = 1;
	}

#if SPARSE
	sparseMatrix = systemMatrix.sparseView();
#endif
}

/**
 * Calculate the right hand side of the equation system
 * @param fixedPoints
 * @param handle
 */
void ARAP::calcB(std::set<int> &fixedPoints, std::set<int> &handles, Eigen::Vector3d &move)
{
	bMatrix.setZero();
	Eigen::Vector3d sum(0.0f, 0.0f, 0.0f);
	for (auto vID : freeVariables)
	{
		sum.setZero();
		for (int neighborID : neighborsMatrix[vID])
		{
			double w = weightMatrix(vID, neighborID);
			Eigen::Matrix3d rot = cellRotations[vID] + cellRotations[neighborID];
			Eigen::Vector3d vert = vertices.row(vID) - vertices.row(neighborID);
			sum += 0.5 * w * rot * vert;
		}
		bMatrix.row(vID) = sum;
	}
	for (auto vID : constraints)
	{
		bMatrix.row(vID) = displacedVertices.row(vID);
	}
}

/**
 * estimate cell rotations
 */
void ARAP::estimateRotation()
{
	//#pragma omp parallel for
	for (auto i = 0; i < vertCount; i++)
	{
		cellRotations[i] = estimateRotationForCell(i);
	}
}

/**
 * estimate rotation for a single cell with vertexid
 * @param vertexId
 * @return Rotation Matrix for one vertex
 */
Eigen::Matrix3d ARAP::estimateRotationForCell(int vertexId)
{
	Eigen::Matrix3d result = Eigen::Matrix3d::Identity();
	auto neighbors = neighborsMatrix[vertexId];

	Eigen::Vector3d pPrimei = displacedVertices.row(vertexId);
	Eigen::Vector3d pi = vertices.row(vertexId);
	Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
#if USE_OPENMP
// define Eigen3d Matrix reduction for + operator
#pragma omp declare reduction(+                             \
							  : Eigen::Matrix3d             \
							  : omp_out = omp_out + omp_in) \
	initializer(omp_priv = Eigen::Matrix3d::Zero())
#pragma omp parallel for default(none) shared(neighbors, vertexId, pi, pPrimei) reduction(+ \
																						  : covariance)
#endif
	for (auto neighbor : neighbors)
	{
		Eigen::Vector3d pj = vertices.row(neighbor);
		Eigen::Vector3d pPrimej = displacedVertices.row(neighbor);
		double wij = weightMatrix(vertexId, neighbor);
		covariance += wij * (pi - pj) * ((pPrimei - pPrimej)).transpose();
	}

	// https://eigen.tuxfamily.org/dox/classEigen_1_1JacobiSVD.html
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);

	Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();
	auto transposeU = svd.matrixU().transpose();
	auto det = (svd.matrixV() * transposeU).determinant();

	// correct the sign
	if (det > 0)
	{
		identity(2, 2) = 1;
	}
	else if (det < 0)
	{
		identity(2, 2) = -1;
	}
	else if (det == 0)
	{
		identity(2, 2) = 0;
	}
	
	return svd.matrixV() * identity *transposeU;
}

/**
 * rotation is fixed. solve LES to get new vert positions (sysmatrix * pPrime = b)
 * @return resulting vertices positions
 */
Eigen::MatrixXd ARAP::estimateTranslation()
{
	Eigen::MatrixXd result;
#if SPARSE
	sparseMatrix.makeCompressed();
	Eigen::SparseLU<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>> solver;
	solver.compute(sparseMatrix);
	if (solver.info() != Eigen::Success)
	{
		PRINT("Decomposition failed!");
	}
	result = solver.solve(bMatrix);
	if (solver.info() != Eigen::Success)
	{
		PRINT("Solving failed");
	}
#else
	Eigen::MatrixXd system_matrix = systemMatrix;
	static Eigen::JacobiSVD<Eigen::MatrixXd> svd(system_matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
	result = svd.solve(bMatrix);
#endif
	displacedVertices = result.replicate(1, 1);
	return result;
}

/**
 * Compute the energy level over cells
 */
double ARAP::computeEnergy()
{
	double energy = 0;
#if USE_OPENMP
#pragma omp parallel for default(none) reduction(+ \
												 : energy)
#endif
	for (int i = 0; i < vertCount; i++)
	{
		double local_energy = 0;
		for (auto neighbor : neighborsMatrix[i])
		{
			Eigen::Matrix3d r = cellRotations[i];
			Eigen::Vector3d displaced = displacedVertices.row(i) - displacedVertices.row(neighbor);
			Eigen::Vector3d original = vertices.row(i) - vertices.row(neighbor);
			local_energy += weightMatrix(i, neighbor) * (displaced - r * original).squaredNorm();
		}
		energy += weightMatrix(i, i) * local_energy;
	}
	return energy;
}

Eigen::MatrixXd ARAP::getWeightMatrix()
{
	return weightMatrix;
}

Eigen::MatrixXd ARAP::getSystemMatrix()
{
	return systemMatrix;
}
