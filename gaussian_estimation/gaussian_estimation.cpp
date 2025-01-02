#include <random>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>

#include <Eigen/Core>

#include "NoisyMeasurementEdge.hpp"
#include "VertexBias.hpp"

int main() {
	const Eigen::Vector2d mean = { 70.0, 20.0 };
	const Eigen::Vector2d stddev = { 100.0, 2.0 };

	std::random_device rd;
	std::mt19937 e2(rd());
	std::normal_distribution<> distX(mean.x(), stddev.x());
	std::normal_distribution<> distY(mean.y(), stddev.y());

	// Set up the parameters of the simulation
	const int numberOfTimeSteps = 10;

	g2o::SparseOptimizer optimizer;
	{
		// Set up the optimiser and block solver
		optimizer.setVerbose(true);

		typedef g2o::BlockSolver<g2o::BlockSolverTraits<2, 2>> BlockSolver;

		g2o::OptimizationAlgorithm *optimizationAlgorithm =
			new g2o::OptimizationAlgorithmGaussNewton(std::make_unique<BlockSolver>(
				std::make_unique<g2o::LinearSolverEigen<BlockSolver::PoseMatrixType>>()));

		optimizer.setAlgorithm(optimizationAlgorithm);
	}

	constexpr int N = 10;

	Eigen::Vector<double, N> valuesX;
	for (int i = 0; i < N; i++) {
		valuesX(i) = distX(e2);
	}
	const double meanEstimateX = valuesX.mean();
	const double stddevEstimateX = (valuesX.array() - meanEstimateX).square().sum() / (N - 1);

	Eigen::Vector<double, N> valuesY;
	for (int i = 0; i < N; i++) {
		valuesY(i) = distY(e2);
	}
	const double meanEstimateY = valuesY.mean();
	const double stddevEstimateY =(valuesY.array() - meanEstimateY).square().sum() / (N - 1);

	for (int i = 0; i < N; i++) {
		std::cout << "point obtained (" << valuesX(i) << "," << valuesY(i) << ")\n";
	}

	std::cout << "initial mean estimate " << "[" << meanEstimateX << ", " << meanEstimateY << "]" << "\n";
	std::cout << "initial stddev estimate " << "[" << std::sqrt(stddevEstimateX) << ", " << std::sqrt(stddevEstimateY) << "]" << "\n";

	auto stateNode = new VertexBias();
	stateNode->setEstimate({ meanEstimateY, meanEstimateY });
	stateNode->setId(0);
	optimizer.addVertex(stateNode);

	// Estimate some initial covariance
	const Eigen::Vector2d stddevEstimate = Eigen::Vector2d { stddevEstimateX, stddevEstimateY };

	for (int i = 0; i < numberOfTimeSteps; i++) {
		const Eigen::Vector2d point { distX(e2), distY(e2) };
		std::cout << "point obtained (" << point(0) << "," << point(1) << ")\n";

		auto noisyMeasurement = new NoisyMeasurementEdge();
		noisyMeasurement->setVertex(0, stateNode);
		noisyMeasurement->setMeasurement(point);
		noisyMeasurement->setInformation(stddevEstimate.asDiagonal().inverse());
		optimizer.addEdge(noisyMeasurement);
	}

	// Configure and set things going
	optimizer.initializeOptimization();
	optimizer.setVerbose(true);
	optimizer.optimize(5);
	std::cerr << "number of vertices: " << optimizer.vertices().size() << "\n";
	std::cerr << "number of edges: " << optimizer.edges().size() << "\n";

	optimizer.save("end.g2o");

	// Print the results
	std::cout << "real final mean " << "[" << mean(0) << ", " << mean(1) << "]" << '\n';
	std::cout << "real final stddev " << "[" << stddev(0) << ", " << stddev(1) << "]" << "\n";

	const auto vertex = dynamic_cast<VertexBias *>(optimizer.vertices().find(0)->second);
	const auto estimate = vertex->estimate();
	const auto edge = dynamic_cast<NoisyMeasurementEdge *>(*vertex->edges().begin());
	const auto covariance = edge->information().inverse().diagonal();

	std::cout << "estimated final mean " << "[" << estimate(0) << ", " << estimate(1) << "]" << '\n';
	std::cout << "estimated final stddev " << "[" << std::sqrt(covariance(0)) << ", " << std::sqrt(covariance(1)) << "]";
}
