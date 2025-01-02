#ifndef EDGENOISYMEASUREMENT_HPP
#define EDGENOISYMEASUREMENT_HPP
#include <g2o/core/base_binary_edge.h>

#include "VertexBias.hpp"


class NoisyMeasurementEdge : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, VertexBias> {
public:
	NoisyMeasurementEdge();

	void computeError() override;

	bool read(std::istream& is) override;

	bool write(std::ostream& os) const override;
};



#endif //EDGENOISYMEASUREMENT_HPP
