#include "VertexBias.hpp"

#include "NoisyMeasurementEdge.hpp"

void VertexBias::setToOriginImpl() {
	_estimate.setZero();
}

void VertexBias::oplusImpl(const double* update) {
	_estimate += Eigen::Vector2d { update[0], update[1] };

	// This is cursed, as the estimate is the bias of the gaussian of unknown variance, once the bias is updated, the
	// variance estimation must also be updated, which means that all edge covariances change
	NoisyMeasurementEdge::Measurement stddevSquared = NoisyMeasurementEdge::Measurement::Zero();
	const auto n = this->_edges.size();
	for (auto edge : this->_edges) {
		auto* noisyMeasurementEdge = dynamic_cast<NoisyMeasurementEdge*>(edge);
		noisyMeasurementEdge->computeError();
		stddevSquared += NoisyMeasurementEdge::Measurement(noisyMeasurementEdge->error().array().square() / (n-1));
	}

	const auto information = stddevSquared.asDiagonal().inverse();
	for (auto edge : this->_edges) {
		auto* noisyMeasurementEdge = dynamic_cast<NoisyMeasurementEdge*>(edge);
		noisyMeasurementEdge->setInformation(information);
	}

	std::cout << "estimated partial mean " << "[" << _estimate(0) << ", " << _estimate(1) << "]" << "\n";
	std::cout << "estimated partial stddev " << "[" << std::sqrt(stddevSquared(0)) << ", " << std::sqrt(stddevSquared(1)) << "]" << "\n";
}

bool VertexBias::read(std::istream& is) {
	for (int i = 0; i < _estimate.size() && is.good(); i++) {
		is >> _estimate(i);
	}
	return is.good() || is.eof();
}

bool VertexBias::write(std::ostream& os) const {
	const auto vector = estimate();
	for (int m = 0; m < vector.rows(); m++) {
		os << vector[m] << " ";
	}
	return os.good();
}