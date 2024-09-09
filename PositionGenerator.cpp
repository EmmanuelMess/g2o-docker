#include "PositionGenerator.hpp"

#include <g2o/stuff/sampler.h>

PositionGenerator::PositionGenerator(double processNoiseSigma, double dt)
		: processNoiseSigma(processNoiseSigma)
		, dt(dt) {
	Vector6d state;

	for (int k = 0; k < 3; k++) {
		state[k] = 1000 * g2o::sampleGaussian();
	}

	currentState = state;
}

void PositionGenerator::next() {
	processNoise = Eigen::Vector3d(processNoiseSigma * g2o::sampleGaussian(), processNoiseSigma * g2o::sampleGaussian(),
																 processNoiseSigma * g2o::sampleGaussian());

	for (int m = 0; m < 3; m++) {
		currentState[m] += dt * (currentState[m + 3] + 0.5 * dt * processNoise[m]);
	}

	for (int m = 0; m < 3; m++) {
		currentState[m + 3] += dt * processNoise[m];
	}


}

Vector6d PositionGenerator::getPosition() const {
	return currentState;
}

Eigen::Vector3d PositionGenerator::getProcessNoise() const {
	return processNoise;
}