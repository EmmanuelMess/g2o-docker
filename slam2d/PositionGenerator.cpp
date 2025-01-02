#include "PositionGenerator.hpp"

#include <g2o/stuff/sampler.h>

PositionGenerator::PositionGenerator(double processNoiseSigma, double dt)
		: processNoiseSigma(processNoiseSigma)
		, dt(dt) {
	position =  Eigen::Vector3d(1000 * g2o::sampleGaussian(), 1000 * g2o::sampleGaussian(), 1000 * g2o::sampleGaussian());
	velocity.setZero();
}

void PositionGenerator::next() {
	processNoise = Eigen::Vector3d(processNoiseSigma * g2o::sampleGaussian(), processNoiseSigma * g2o::sampleGaussian(),
																 processNoiseSigma * g2o::sampleGaussian());

	position = position + dt * velocity + 0.5 *  dt * dt * processNoise;
	velocity = velocity + dt * processNoise;
}

Eigen::Vector3d PositionGenerator::getPosition() const {
	return position;
}

Eigen::Vector3d PositionGenerator::getVelocity() const {
	return velocity;
}

Eigen::Vector3d PositionGenerator::getProcessNoise() const {
	return processNoise;
}