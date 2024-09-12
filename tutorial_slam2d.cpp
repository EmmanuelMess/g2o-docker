// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, H. Strasdat, W. Burgard
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

// This example consists of a single constant velocity target which
// moves under piecewise constant velocity in 3D. Its position is
// measured by an idealised GPS receiver.

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/stuff/sampler.h>

#include <iostream>
#include <vector>

#include "RegisterG2o.hpp"
#include "targetTypes6D.hpp"
#include "EigenTypes.hpp"
#include "PositionGenerator.hpp"

using namespace Eigen;
using namespace g2o;

template<typename Vector>
static void printVector(const Vector& state) {
	std::cout << "[ ";
	for (int m = 0; m < state.rows(); m++) {
		std::cout << state[m] << " ";
	}
	std::cout << "]\n";
}

int main() {
	// Set up the parameters of the simulation
	const int numberOfTimeSteps = 1000;
	const double processNoiseSigma = 1;
	const double accelerometerNoiseSigma = 1;
	const double gpsNoiseSigma = 1;
	const double dt = 1;

	std::vector<Vector6d> realStates;

	PositionGenerator positionGenerator(processNoiseSigma, dt);

	SparseOptimizer optimizer;
	{
		// Set up the optimiser and block solver
		optimizer.setVerbose(true);

		typedef BlockSolver<BlockSolverTraits<6, 6>> BlockSolver;

		OptimizationAlgorithm *optimizationAlgorithm =
			new OptimizationAlgorithmGaussNewton(std::make_unique<BlockSolver>(
				std::make_unique<LinearSolverEigen<BlockSolver::PoseMatrixType>>()));

		optimizer.setAlgorithm(optimizationAlgorithm);
	}

	// Set up last estimate
	VertexPositionVelocity3D *lastStateNode;

	{
		{
			Vector6d state;
			state.setZero();
			state.head(3) = positionGenerator.getPosition();
			state.tail(3) = positionGenerator.getVelocity();

			realStates.push_back(state);
		}

		// Construct the first vertex; this corresponds to the initial
		// condition and register it with the optimiser
		auto stateNode = new VertexPositionVelocity3D();
		stateNode->setEstimate(Vector6d::Zero());
		stateNode->setId(0);
		optimizer.addVertex(stateNode);

		// Set up last estimate
		lastStateNode = stateNode;
	}

	// Iterate over the simulation steps
	for (int k = 1; k <= numberOfTimeSteps; ++k) {
		std::cout << "iteration " << k << " time " << (dt * k) << "\n";

		// Simulate the next step; update the state and compute the observation
		positionGenerator.next();

		{
			Vector6d state;
			state.setZero();
			state.head(3) = positionGenerator.getPosition();
			state.tail(3) = positionGenerator.getVelocity();

			realStates.push_back(state);
		}

		// Construct the accelerometer measurement
		const Vector3d accelerometerNoise(sampleGaussian(), sampleGaussian(), sampleGaussian());
		const Vector3d accelerometerMeasurement = positionGenerator.getProcessNoise() + accelerometerNoiseSigma * accelerometerNoise;

		// Construct the GPS observation
		const Vector3d gpsNoise(sampleGaussian(), sampleGaussian(), sampleGaussian());
		const Vector3d gpsMeasurement = positionGenerator.getPosition() + gpsNoiseSigma * gpsNoise;

		std::cout << "real position ";
		printVector(positionGenerator.getPosition());
		std::cout << "real velocity ";
		printVector(positionGenerator.getVelocity());
		std::cout << "accelerometer ";
		printVector(accelerometerMeasurement);
		std::cout << "gps position ";
		printVector(gpsMeasurement);

		// Construct vertex which corresponds to the current state of the target
		auto stateNode = new VertexPositionVelocity3D();

		stateNode->setId(k);
		stateNode->setMarginalized(false);
		optimizer.addVertex(stateNode);

		auto targetOdometryEdge = new TargetOdometry3DEdge(dt, accelerometerNoiseSigma);
		targetOdometryEdge->setVertex(0, lastStateNode);
		targetOdometryEdge->setVertex(1, stateNode);
		auto vPrev = lastStateNode;
		auto vCurr = stateNode;
		targetOdometryEdge->setMeasurement(accelerometerMeasurement);
		optimizer.addEdge(targetOdometryEdge);

		// compute the initial guess via the odometry
		g2o::OptimizableGraph::VertexSet vPrevSet;
		vPrevSet.insert(vPrev);
		targetOdometryEdge->initialEstimate(vPrevSet, vCurr);

		lastStateNode = stateNode;

		// Add the GPS observation
		auto goe = new GPSObservationEdgePositionVelocity3D(gpsMeasurement, gpsNoiseSigma);
		goe->setVertex(0, stateNode);
		optimizer.addEdge(goe);
	}

	// Configure and set things going
	optimizer.initializeOptimization();
	optimizer.setVerbose(true);
	optimizer.optimize(5);
	std::cerr << "number of vertices:" << optimizer.vertices().size() << std::endl;
	std::cerr << "number of edges:" << optimizer.edges().size() << std::endl;

	optimizer.save("end.g2o");

	// Print the results

	std::cout << "real final position ";
	printVector(positionGenerator.getPosition());
	std::cout << "estimated final position ";
	printVector(dynamic_cast<VertexPositionVelocity3D *>(optimizer.vertices().find(numberOfTimeSteps - 1)->second)->estimate().head(3));

	double error = 0;

	for (const auto& [id, vertex] : optimizer.vertices()) {
		Eigen::Vector3d positionEst = dynamic_cast<VertexPositionVelocity3D *>(vertex)->estimate().head(3);
		Eigen::Vector3d positionReal = realStates[id].head(3);
		error += (positionReal - positionEst).array().abs().sum();
	}

	std::cout << "Error " << error;
}