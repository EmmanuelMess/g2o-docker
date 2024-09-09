#ifndef G2OTEST_POSITIONGENERATOR_HPP
#define G2OTEST_POSITIONGENERATOR_HPP

#include "EigenTypes.hpp"

class PositionGenerator {
public:
	PositionGenerator(double processNoiseSigma, double dt);

	void next();

	Vector6d getPosition() const;
	Eigen::Vector3d getProcessNoise() const;

private:
	double processNoiseSigma;
	double dt;
	Vector6d currentState;
	Eigen::Vector3d processNoise;
};

#endif //G2OTEST_POSITIONGENERATOR_HPP
