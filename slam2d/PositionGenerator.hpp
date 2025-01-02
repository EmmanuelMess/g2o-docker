#ifndef G2OTEST_POSITIONGENERATOR_HPP
#define G2OTEST_POSITIONGENERATOR_HPP

#include "EigenTypes.hpp"

class PositionGenerator {
public:
	PositionGenerator(double processNoiseSigma, double dt);

	void next();

	[[nodiscard]] Eigen::Vector3d getPosition() const;
	[[nodiscard]] Eigen::Vector3d getVelocity() const;
	[[nodiscard]] Eigen::Vector3d getProcessNoise() const;

private:
	double processNoiseSigma;
	double dt;
	Eigen::Vector3d position;
	Eigen::Vector3d velocity;
	Eigen::Vector3d processNoise;
};

#endif //G2OTEST_POSITIONGENERATOR_HPP
