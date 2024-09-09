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
	int numberOfTimeSteps = 1000;
	const double processNoiseSigma = 1;
	const double accelerometerNoiseSigma = 1;
	const double gpsNoiseSigma = 1;
	const double dt = 1;

	PositionGenerator position(processNoiseSigma, dt);

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
	Vector6d state;

	{
		// Sample the start location of the target
		state.setZero();
		for (int k = 0; k < 3; k++) {
			state[k] = 1000 * sampleGaussian();
		}

		// Construct the first vertex; this corresponds to the initial
		// condition and register it with the optimiser
		auto stateNode = new VertexPositionVelocity3D();
		stateNode->setEstimate(state);
		stateNode->setId(0);
		optimizer.addVertex(stateNode);

		// Set up last estimate
		lastStateNode = stateNode;
	}

	// Iterate over the simulation steps
	for (int k = 1; k <= numberOfTimeSteps; ++k) {
		// Simulate the next step; update the state and compute the observation
		position.next();

		// Construct the accelerometer measurement
		const Vector3d accelerometerNoise = { accelerometerNoiseSigma * sampleGaussian(), accelerometerNoiseSigma * sampleGaussian(), accelerometerNoiseSigma * sampleGaussian() };
		const Vector3d accelerometerMeasurement = position.getProcessNoise().head(3) + accelerometerNoise;

		// Construct the GPS observation
		const Vector3d gpsNoise = { gpsNoiseSigma * sampleGaussian(), gpsNoiseSigma * sampleGaussian(), gpsNoiseSigma * sampleGaussian() };
		const Vector3d gpsMeasurement = state.head(3) + gpsNoise;

		std::cout << "position ";
		printVector(position.getPosition());
		std::cout << "accelerometer ";
		printVector(accelerometerMeasurement);
		std::cout << "gps ";
		printVector(gpsMeasurement);

		// Construct vertex which corresponds to the current state of the target
		auto stateNode = new VertexPositionVelocity3D();

		stateNode->setId(k);
		stateNode->setMarginalized(false);
		optimizer.addVertex(stateNode);

		auto toe = new TargetOdometry3DEdge(dt, accelerometerNoiseSigma);
		toe->setVertex(0, lastStateNode);
		toe->setVertex(1, stateNode);
		auto vPrev = dynamic_cast<VertexPositionVelocity3D *>(lastStateNode);
		auto vCurr = dynamic_cast<VertexPositionVelocity3D *>(stateNode);
		toe->setMeasurement(accelerometerMeasurement);
		optimizer.addEdge(toe);

		// compute the initial guess via the odometry
		g2o::OptimizableGraph::VertexSet vPrevSet;
		vPrevSet.insert(vPrev);
		toe->initialEstimate(vPrevSet, vCurr);

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

	std::cout << "state=";
	printVector(state);

	Vector6d v1 = dynamic_cast<VertexPositionVelocity3D *>(optimizer.vertices().find((std::max)(numberOfTimeSteps - 2, 0))->second)->estimate();
	Vector6d v2 = dynamic_cast<VertexPositionVelocity3D *>(optimizer.vertices().find((std::max)(numberOfTimeSteps - 1, 0))->second)->estimate();
	std::cout << "v1=";
	printVector(v1);
	std::cout << "v2=";
	printVector(v2);
	std::cout << "delta state=";
	printVector(v2 - v1);
}